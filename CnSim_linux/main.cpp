//------   system   ------//
#include <stdlib.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/mman.h>	  // memory mapping
#include <sys/resource.h> //stack memory extension

//------   shared memory header   ------//
#include <sys/types.h> // Type
#include <fcntl.h>	   // File Control

//------   signal header   ------//
#include <signal.h>

//------   RT header   ------//
#include "rt_posix.h"
#include <limits.h> // Limits
#include <sched.h>	// Scheduler

//------   end of cannon control header and variables (shpark)   ------//

//------   Can Interface Header and Variables (JSShin)   ------//
#include "CAN_Manager.h"

CCAN_Manager m_canManager;
//------   End of Can Interface Header and Variables (JSShin)   ------//

//------ CDSL Controller code ------//
#include "controller.h"
ManualController m_manual_controller;
PDController m_PD_controller;
FLController m_FL_controller;
DOBController m_DOB_controller;
RealWorldConfigurer m_real_world_configurer;

#include "ReferenceGenerator.h"
ReferenceGenerator m_reference_generator;
LowPassFilter m_low_pass_filter;
#include <array>
int loop_counter = 0;
int error_counter = 0;

double theta1_dot_d = 0.0; double theta2_dot_d = 0.0;
double theta1_dot_d_filtered = 0.0; double theta2_dot_d_filtered = 0.0;
double jPos1_prev = 0.0; double jPos2_prev = 0.0;
double theta1_dot_d_prev = 0.0; double theta2_dot_d_prev = 0.0;
// double global_theta_2_fixed = 0.0; //temp
// double gear_ratio =  m_canManager.m_Dev[0].m_Gear_Ratio;

std::array<double, 2> u_hat;
std::array<double, 2> u_FL;
std::array<double, 2> estimated_disturbance;

#include <fstream>
//------ CDSL Controller code ------//

std::ofstream output_file("cdsl_data.csv");

// USER Paramter //
#define DEBUG false

int RT_Timeout_Error_Flag = false;
int RT_Shutdown_Flag = false;

void signal_handler(int sig)
{
	printf("\n Ctrl+C Pressed. Shutdown this program! \r\n\n");

	RT_Shutdown_Flag = true; // RT Shutdown Flag

	m_canManager.Finalize(); // Finalize CAN

	munlockall(); // Unlock Memory

	exit(1);
}

static void *run_rtCycle(void *pParam)
{
	struct Period_Info PInfo;

	struct timespec clockCheck;
	unsigned long s_time, e_time, op_time; // monitor one loop time (ns)
	unsigned long robot_time[2];		   // monitor real robot operation time

	// set cycle time and initialize next period
	rt_posix_init_periodic(&PInfo, long(1) * 1000); // long(1) -> 1ms

	int MAX_DOF = 2;

	while (1)
	{
		clock_gettime(CLOCK_REALTIME, &clockCheck); // Start time
		s_time = clockCheck.tv_sec * 1000000000 + clockCheck.tv_nsec;

		// RX process
		// Read Encoder
		double jPos[2];
		m_canManager.Recv_PDO_Data();
		
		for (int i = 0; i < MAX_DOF; i++)
		{
			// Pulse (4096/rev = 2^12, 12bit Incremental Encoder)
			// Input Pulse/Pulse(1rev) * Radian(1rev) / Reduction Ratio
			jPos[i] = (((((double)m_canManager.m_Dev[i].m_ActPos)/4096.0) * (2.0 * 3.141592)) / m_canManager.m_Dev[i].m_Gear_Ratio); // Inc to Radian
			jPos[i] = jPos[i]*(-1.0); // Convert Direction
			// printf("[%d]Actual Position : %d \r\n\n",i,m_canManager.m_Dev[i].m_ActPos);
			
			// printf("[%d]rad Position : %d \r\n\n",i,jPos[i]);
		}
	
		//-------------CDSL_Control Field------------//
		//1. basic settings

		int control_torque[2];
		control_torque[0] = 0; control_torque[1] = 0;
		
		//---1-1. loop_counter
		loop_counter++;
		double current_time = loop_counter * 0.004;
		printf("\npassed time(ms): %f", current_time);
		// printf("\nfrom KIRO, passed time: %f\n", robot_time[2]);

		//---1-2. numerical differeince calculation
		if (loop_counter > 1) {
			double sampling_time = 0.004;
			theta1_dot_d = (jPos[0] - jPos1_prev)/sampling_time;
			theta2_dot_d = (jPos[1] - jPos2_prev)/sampling_time;
		}
		jPos1_prev = jPos[0];  jPos2_prev = jPos[1];
		// printf("\n current joint_1 velocity is : %f", theta_dot_d);

		//---1-3. low pass filter
		theta1_dot_d_filtered = m_low_pass_filter.calculate_lowpass_filter(theta1_dot_d, theta1_dot_d_prev);
		theta1_dot_d_prev = theta1_dot_d_filtered;
		printf("\n filtered joint_1 velocity is : %f", theta1_dot_d_filtered);

		theta2_dot_d_filtered = m_low_pass_filter.calculate_lowpass_filter(theta2_dot_d, theta2_dot_d_prev);
		theta2_dot_d_prev = theta2_dot_d_filtered;
		printf("\n filtered joint_1 velocity is : %f", theta2_dot_d_filtered);

		//2. reference generation
		double theta1_desired_d = m_reference_generator.get_position(current_time);
		double theta1_dot_desired_d = m_reference_generator.get_velocity(current_time);
		double theta1_ddot_desired_d = m_reference_generator.get_acceleration(current_time);

		double theta2_desired_d = m_reference_generator.get_position(current_time);
		double theta2_dot_desired_d = m_reference_generator.get_velocity(current_time);

		printf("\nreference theta2_desired_d is : %f", theta2_desired_d);
		printf("\nreference theta2_dot_desired_d is : %f\n", theta2_dot_desired_d);
		printf("\ncurrent joint_1 is  : %f\n", jPos[0]);
		printf("\ncurrent joint_2 is  : %f\n", jPos[1]);

		//---2-1. position, velocity error define
		double joint_error_1 = theta1_desired_d - jPos[0];
		double joint_error_1_dot = theta1_dot_desired_d - theta1_dot_d_filtered; 

		double joint_error_2 = theta2_desired_d - jPos[1];
		double joint_error_2_dot = theta2_dot_desired_d - theta2_dot_d_filtered; 



		//3. control mode selection
		int controlmode = ctrl_fl_dob; // 0: Manual, 1: PD, 2: FL, 3: FL + DOB
		switch (controlmode)
		{
		case ctrl_manual:
			{	
			control_torque[0] = m_manual_controller.calculateTau(0); // Example input torque
			control_torque[1] = m_manual_controller.calculateTau(0);
			// printf("\ninput of manual controller is (%d, %d)\n", control_torque[0], control_torque[1]);
			break;
			}
		
		case ctrl_pd:
			{
			control_torque[0] = static_cast<int>(m_PD_controller.calculateTau(0, joint_error_1, joint_error_1_dot));
			control_torque[1] = 0.0; // static_cast<int>(m_PD_controller.calculateTau(1, joint_error_2, joint_error_2_dot)); 
			printf("\ninput of pd controller is (%d, %d)\n", control_torque[0], control_torque[1]);
			
			// derivate term debugging
			double propo_term_torque = m_PD_controller.tauPropo(0, joint_error_1);
			double deriv_term_torque = m_PD_controller.tauDeriv(0, joint_error_1_dot);

			// csv file output writing
			output_file << theta1_desired_d <<"," <<jPos[0] <<"," 
					    << theta1_dot_desired_d << "," << theta1_dot_d_filtered << "," 
						<< control_torque[0] << ","
						<< propo_term_torque << "," << deriv_term_torque << "," 
						<< current_time  << std::endl;
			break;
			}

		case ctrl_fl:
			{
			double global_theta_2_fixed = 0.0;
			double gear_ratio =  m_canManager.m_Dev[0].m_Gear_Ratio;
				
			std::array<double, 4>torque_calculate = m_FL_controller.calculateTau(theta1_ddot_desired_d, joint_error_1, joint_error_1_dot,
                                							 					 jPos[0], global_theta_2_fixed, 
			 																	 theta1_dot_d_filtered, theta2_dot_d_filtered);

			double torque_temp[4];																	 
			torque_temp[0] = m_real_world_configurer.Nm_to_permil(torque_calculate[0], gear_ratio);
			torque_temp[1] = m_real_world_configurer.Nm_to_permil(torque_calculate[1], gear_ratio);
			torque_temp[2] = m_real_world_configurer.Nm_to_permil(torque_calculate[2], gear_ratio);
			torque_temp[3] = m_real_world_configurer.Nm_to_permil(torque_calculate[3], gear_ratio);
			
			double h_torque[2]; // for debug, h term in permil
			h_torque[0] = m_real_world_configurer.InvertTorquesign(torque_temp[2]);
			h_torque[1] = m_real_world_configurer.InvertTorquesign(torque_temp[3]);

			double sat_torque_permil[2];
			sat_torque_permil[0] = m_real_world_configurer.TorqueSaturation(torque_temp[0], 1300);
			sat_torque_permil[1] = m_real_world_configurer.TorqueSaturation(torque_temp[1], 2000);

			// real control input in permil + saturation applied  of FL controller 
			control_torque[0] = static_cast<int>(m_real_world_configurer.InvertTorquesign(sat_torque_permil[0]));
			control_torque[1] = static_cast<int>(m_real_world_configurer.InvertTorquesign(sat_torque_permil[1]));

			printf("\ninput of FL controller in main is (%d, %d)\n", control_torque[0], control_torque[1]);

			// csv file output writing
			output_file << theta1_desired_d <<"," <<jPos[0] <<"," 
					    << theta1_dot_desired_d << "," << theta1_dot_d_filtered << "," 
						<< control_torque[0] << ","
						<< h_torque[0] << "," 
						<< current_time  << std::endl;
						
			break;
			}


		case ctrl_fl_dob:
			{
				
			double global_theta_2_fixed = 0.0;
			double gear_ratio =  m_canManager.m_Dev[0].m_Gear_Ratio;
			
			//FL 제어기 먼저 실행
			std::array<double, 4> torque_calculate = m_FL_controller.calculateTau(theta1_ddot_desired_d, joint_error_1, joint_error_1_dot,
                                							 					 	  jPos[0], global_theta_2_fixed, 
			 																	      theta1_dot_d_filtered, theta2_dot_d_filtered); // torque_Nm
			printf("u_FL_1 : {%f}\n" , u_FL[0]);
			printf("u_FL_2 : {%f}\n" , u_FL[1]);
			
			printf("u_hat_1 : {%f}\n" , u_hat[0]);
			printf("u_hat_2 : {%f}\n" , u_hat[1]);

			u_FL[0] = torque_calculate[0]; // [Nm]
			u_FL[1] = torque_calculate[1]; // [Nm]
		
			//DOB torque generation [Nm]
			// double time_constant = 5e-3; // 튜닝 파라미터
			double time_constant = 0.;
			estimated_disturbance = m_DOB_controller.EstimateDisturbance(jPos[0], jPos[1], theta1_dot_d_filtered, theta2_dot_d_filtered, u_hat, time_constant);
			printf("estimated_disturbance_1 : {%f} \n" , estimated_disturbance[0]);
			printf("estimated_disturbance_2 : {%f} \n" , estimated_disturbance[1]);

			estimated_disturbance[0] = m_real_world_configurer.TorqueSaturation(estimated_disturbance[0], 20); // Nm
			estimated_disturbance[1] = 0.0; // m_real_world_configurer.TorqueSaturation(estimated_disturbance[1], 20 * 1e-3); // Nm
			
			u_hat[0] = u_FL[0] - estimated_disturbance[0]; //update
			u_hat[1] = u_FL[1] - estimated_disturbance[1]; //update
			


			double disturb_comp_temp[2];
			disturb_comp_temp[0] = m_real_world_configurer.Nm_to_permil(u_hat[0], gear_ratio);
			disturb_comp_temp[1] = m_real_world_configurer.Nm_to_permil(u_hat[1], gear_ratio);
			
			// printf("\ndisturb_comp_temp[0] is (%f)", disturb_comp_temp[0]);
			// printf("\ndisturb_comp_temp[1] is (%f)\n", disturb_comp_temp[1]);
			
			double ditsurbance_comp_input[2];
			ditsurbance_comp_input[0] = m_real_world_configurer.TorqueSaturation(disturb_comp_temp[0], 1300);
			ditsurbance_comp_input[1] = m_real_world_configurer.TorqueSaturation(disturb_comp_temp[1], 2000);
			
			// printf("\nditsurbance_comp_input[0] is (%f)", ditsurbance_comp_input[0]);
			// printf("\nditsurbance_comp_input[1] is (%f)\n", ditsurbance_comp_input[1]);

			ditsurbance_comp_input[0] = m_real_world_configurer.InvertTorquesign(ditsurbance_comp_input[0]);
			ditsurbance_comp_input[1] = m_real_world_configurer.InvertTorquesign(ditsurbance_comp_input[1]);

			
			// printf("\nditsurbance_comp_input[0]_afterinvert is (%f)", ditsurbance_comp_input[0]);
			// printf("\nditsurbance_comp_input[1]_afterinvert is (%f)\n", ditsurbance_comp_input[1]);
			
			///
			control_torque[0] = static_cast<int>(ditsurbance_comp_input[0]);
			control_torque[1] = static_cast<int>(ditsurbance_comp_input[1]);

			printf("\ninput of DOB controller is (%d, %d)\n", control_torque[0], control_torque[1]);
			
			// int control_torque_debug[2];
			// control_torque_debug[0] = static_cast<int>(ditsurbance_comp_input[0]);
			// control_torque_debug[1] = static_cast<int>(ditsurbance_comp_input[1]);
			
			//printf("\ninput of DOB controller is (%d, %d)\n", control_torque_debug[0], control_torque_debug[1]);
			
			// csv file output writing
			output_file << theta1_desired_d <<"," <<jPos[0] <<"," 
					    << theta1_dot_desired_d << "," << theta1_dot_d_filtered << "," 
						<< u_hat[0] << "," << estimated_disturbance[0]  << "," << current_time  << std::endl;
						
			break;
			}

		default:
			{
			control_torque[0] = 0;
			control_torque[1] = 0;
			printf("\n control mode is not defined, so torque is set to 0\n");
			break;
			}
		}
		

		//-------------CDSL_Control Field Ends-------------//

		// TX process
		// Torque Command
		for (int i = 0; i < MAX_DOF; i++)
		{
			m_canManager.m_Dev[i].m_PWR_S = 1; // Servo ON(1), Servo Off(0)
			m_canManager.m_Dev[i].m_CtrlWord = m_canManager.m_Dev[i].Power(m_canManager.m_Dev[i].m_PWR_S);
				
			switch(m_canManager.m_Dev[i].m_MoOp)
			{
				default:
				case CSP: 
				m_canManager.m_Dev[i].m_TargetPos = 0;
				m_canManager.m_Dev[i].m_TargetPos = 0;
				printf("[%d]Target Position : %d \r\n\n",i,m_canManager.m_Dev[i].m_TargetPos);
				break;
				case CST: // Thousand Per Rated Torque (Rated Torque : 52.8mNm = 1000, MAXON EC-i 40)
				m_canManager.m_Dev[i].m_TargetTrq = control_torque[i]; // Input Torque
				m_canManager.m_Dev[i].m_TargetTrq = control_torque[i]; // Input Torque
				printf("[%d]Target Torque : %d \r\n",i,m_canManager.m_Dev[i].m_TargetTrq);
				break;
			}
			m_canManager.Send_PDO_Data(i);
		}


		clock_gettime(CLOCK_REALTIME, &clockCheck); // end time
		e_time = clockCheck.tv_sec * 1000000000 + clockCheck.tv_nsec;
		op_time = e_time - s_time;
		
		//what
		// printf("\ncontrol loop calculation time: %d", op_time);
		// printf("\nperiod_ns: %d", PInfo.period_ns);
		//is it 

		if (op_time > PInfo.period_ns)
		{
			RT_Timeout_Error_Flag = true;
			printf("\e[31m [cannon cycle time over] [%ld] us \e[0m \n\n", op_time / 1000);
			error_counter++;
		}
		
		printf("\nerror_counter is  (%d)\n", error_counter);
		// wait next period
		rt_posix_wait_period(&PInfo);
	}
	return NULL;
}

int main(int nArgc, char *ppArgv[])
{
	printf("=================== \n");
	printf(">> Start program << \n");
	printf("=================== \n\n\n");

	signal(SIGINT, signal_handler); // ctrl+C
	signal(SIGTERM, signal_handler);

	m_canManager.m_Dev[0].m_Gear_Ratio = 4440;
	m_canManager.m_Dev[1].m_Gear_Ratio = 4440;

	int opMode;

	opMode = 1; // Position Mode(0) or Torque Mode(1)

	// Initialize CAN Controller
	m_canManager.Initialize(opMode);

	//csv file generation for PD 
	// output_file << "joint_pos_desired, joint_pos, joint_vel_desired, joint_vel_filtered, target_torque, propo_term_torque, deriv_term_torque, current_time" << std::endl;
	
	// //csv file generation for FL
	// output_file << "joint_pos_desired_1, joint_pos_1, joint_vel_desired_1, joint_vel_filtered_1, input torque_1, h term torque_1, current_time" << std::endl;
	
	// csv file generation for FL_DOB
	output_file << "joint_pos_desired_1, joint_pos_1, joint_vel_desired_1, joint_vel_filtered_1, input torque_1_w.DOB[Nm], DOB Torque Term[Nm], current_time" << std::endl;


	//------   create thread
	pthread_t run_rt_cannon_thread; // real time loop for cannon control

	int status; // thread status

	// lock memory
	if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1)
	{
		printf("Memory lock all failed! : %m\n");
		exit(-2);
	}

	status = rt_posix_create(&run_rt_cannon_thread, PTHREAD_STACK_MIN, SCHED_RR, 99, PTHREAD_EXPLICIT_SCHED, run_rtCycle, NULL);

	//------   block while run
	while (1)
	{
		usleep(100000);
	}

	status = pthread_join(run_rt_cannon_thread, NULL);

	// unlock memory
	munlockall();

	printf("program finished\n");

	return 0;
}
