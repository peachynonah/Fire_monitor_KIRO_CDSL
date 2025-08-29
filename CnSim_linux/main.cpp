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


//------ CDSL Controller code starts ------//
#include "controller.h"
ManualController m_manual_controller;
PDController m_PD_controller;
FLController m_FL_controller;
DOBController m_DOB_controller;
RealWorldConfigurer m_real_world_configurer;

#include "ReferenceGenerator.h"
// CDSL control part includes this classes 
// ReferenceGenerator (for generating polynomial reference depend by time)
LowPassFilter m_low_pass_filter;
#include <array>
int loop_counter = 0;
int error_counter = 0;
int control_torque[2];

// part  1
std::array<double,2> theta_dot_d = {0.0 , 0.0};
std::array<double,2> theta_dot_d_filtered = {0.0 , 0.0};
std::array<double,2> jPos_prev = {0.0 , 0.0};
double theta1_dot_d_prev = 0.0; double theta2_dot_d_prev = 0.0;

std::array<double, 2> joint_error = {0.0 , 0.0};
std::array<double, 2> joint_error_dot = {0.0 , 0.0};

// controller part declaration
std::array<double, 3> PD_control_return_joint1 = {0.0, 0.0, 0.0};
std::array<double, 3> PD_control_return_joint2 = {0.0, 0.0, 0.0};

std::array<double, 2> propo_term_torque = {0.0 , 0.0};
std::array<double, 2> deriv_term_torque = {0.0 , 0.0};
std::array<double, 2> sat_torque_PD = {0.0 , 0.0};

// double global_theta_2_fixed = 0.0; //temp
// double gear_ratio =  m_canManager.m_Dev[0].m_Gear_Ratio;

std::array<double, 2> u_hat;
std::array<double, 2> u_FL;
std::array<double, 2> estimated_disturbance;
std::array<double, 2> applied_tau;
std::array<double, 2> estimated_disturbance_not_limited;
#include <fstream>

std::ofstream output_file("cdsl_data.csv");

//reference generator declaration
std::array<double, 2> initial_joint_position_ = {0.0, -0.06}; // initial position setting
double pulse_error = 0.3925;
std::array<double, 2> reference_start_position_ = {initial_joint_position_[0], initial_joint_position_[1] + pulse_error};

// start at reference_start_position_[0], finish at minus pi/2 : clockwise
// std::array<double, 6> alpha_coeffs_joint1 = {reference_start_position_[0], 0.0, 0.0, -0.001475203162, 0.00010058203, -1.8287642508e-06};
// alpha coefficients: stay at current position
std::array<double, 6> alpha_coeffs_joint1 = {reference_start_position_[0], 0.0, 0.0, 0.0, 0.0, 0.0};
double final_operating_time = 10.0;
ReferenceGenerator m_reference_generator_joint1(clockwise, alpha_coeffs_joint1, final_operating_time);

// start at reference_start_position_[0], finish at minus pi/6 : under = clockwise
// std::array<double, 6> alpha_coeffs_joint2 = {reference_start_position_[1], 0.0, 0.0, -0.0004917343873, 3.35273445e-05, -6.09588083e-07};
// alpha coefficient-6.09588083e-07

// 10-7) reference is moving only pi/8, for 10 second, clockwise
// std::array<double, 6> alpha_coeffs_joint2 = {reference_start_position_[1], 0.0, 0.0, -0.003926990816987245, 0.0005890486225480869, -2.3561944901923487e-05};
// ReferenceGenerator m_reference_generator_joint2(anticlockwise, alpha_coeffs_joint2, final_operating_time);

//10-8) reference is moving only pi/6, for 22 second, clockwise
// std::array<double, 6> alpha_coeffs_joint2 = {reference_start_position_[1], 0.0, 0.0, -0.0004917343873011824, 3.352734458871698e-05, -6.095880834312178e-07};
// ReferenceGenerator m_reference_generator_joint2(clockwise, alpha_coeffs_joint2, final_operating_time);

// // reference is moving only pi/8, for 10 second, clockwise, starting with initial velocity 0.08[rad/sec] 
std::array<double, 6> alpha_coeffs_joint2 = {reference_start_position_[1], -0.08, 0.0, 0.0008730091830127596, -5.095137745191394e-05, 4.3805509807655447e-07};
ReferenceGenerator m_reference_generator_joint2(anticlockwise, alpha_coeffs_joint2, final_operating_time);

//------ CDSL Controller code ends ------//

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
		control_torque[0] = 0; 
		control_torque[1] = 0;
		
		//---1-1. loop_counter
		loop_counter++;
		double current_time = loop_counter * 0.004;
		printf("\npassed time(ms): %f", current_time);
		// printf("\nfrom KIRO, passed time: %f\n", robot_time[2]);

		//---1-2. numerical differeince calculation
		if (loop_counter > 1) {
			double sampling_time = 0.004;
			theta_dot_d[0] = (jPos[0] - jPos_prev[0])/sampling_time;
			theta_dot_d[1] = (jPos[1] - jPos_prev[1])/sampling_time;
		}
			jPos_prev[0] = jPos[0];  
			jPos_prev[1] = jPos[1];
		// printf("\n current joint_1 velocity is : %f", theta_dot_d);

		//---1-3. low pass filter
		theta_dot_d_filtered[0] = m_low_pass_filter.calculate_lowpass_filter(theta_dot_d[0], theta1_dot_d_prev, 0.008);
		theta1_dot_d_prev = theta_dot_d_filtered[0];
		printf("\n filtered joint_1 velocity is : %f", theta_dot_d_filtered[0]);

		theta_dot_d_filtered[1] = m_low_pass_filter.calculate_lowpass_filter(theta_dot_d[1], theta2_dot_d_prev, 0.008);
		theta2_dot_d_prev = theta_dot_d_filtered[1];
		printf("\n filtered joint_1 velocity is : %f", theta_dot_d_filtered[1]);


		std::array<double, 2> theta_desired_d, theta_dot_desired_d, theta_ddot_desired_d;

		theta_desired_d[0] = m_reference_generator_joint1.get_position(current_time);
		theta_dot_desired_d[0] = m_reference_generator_joint1.get_velocity(current_time);
		theta_ddot_desired_d[0] = m_reference_generator_joint1.get_acceleration(current_time);

		theta_desired_d[1] = m_reference_generator_joint2.get_position(current_time);
		theta_dot_desired_d[1] = m_reference_generator_joint2.get_velocity(current_time);
		theta_ddot_desired_d[1] = m_reference_generator_joint2.get_acceleration(current_time);

		printf("\ncurrent joint_1 is  : %f\n", jPos[0]);
		printf("\ncurrent joint_2 is  : %f\n", jPos[1]);

		//---2-1. position, velocity error define		
		for (int i=0; i<MAX_DOF; i++){
	     	joint_error[i] = theta_desired_d[i] - jPos[i];
		 	joint_error_dot[i] = theta_dot_desired_d[i] - theta_dot_d_filtered[i]; 
			}

		// joint_error[0] = theta_desired_d[0] - jPos[0];
		// joint_error_dot[0] = theta_dot_desired_d[0] - theta_dot_d_filtered[0]; 

		// joint_error[1] = theta_desired_d[1] - jPos[1];
		// joint_error_dot[1] = theta_dot_desired_d[1] - theta_dot_d_filtered[1]; 


		//3. control mode selection
		int controlmode = ctrl_pd; // 0: Manual, 1: PD, 2: FL, 3: FL + DOB
		switch (controlmode)
		{
		case ctrl_manual:
			{	
			control_torque[0] = m_manual_controller.calculateTau(0); // Example input torque
			control_torque[1] = m_manual_controller.calculateTau(500);
			// printf("\ninput of manual controller is (%d, %d)\n", control_torque[0], control_torque[1]);
			break;
			}
		
		case ctrl_pd:
			{
			PD_control_return_joint1 = 	m_PD_controller.calculateTau(0, theta_desired_d[0], jPos[0], theta_dot_desired_d[0], theta_dot_d_filtered[0]);
			PD_control_return_joint2 = 	m_PD_controller.calculateTau(1, theta_desired_d[1], jPos[1], theta_dot_desired_d[1], theta_dot_d_filtered[1]);

			propo_term_torque[0] = m_real_world_configurer.InvertTorquesign(PD_control_return_joint1[0]);
			deriv_term_torque[0] = m_real_world_configurer.InvertTorquesign(PD_control_return_joint1[1]);

			propo_term_torque[1] = m_real_world_configurer.InvertTorquesign(PD_control_return_joint2[0]);
			deriv_term_torque[1] = m_real_world_configurer.InvertTorquesign(PD_control_return_joint2[1]);

			sat_torque_PD[0] = m_real_world_configurer.TorqueSaturation(PD_control_return_joint1[2], 1300);
			sat_torque_PD[1] = m_real_world_configurer.TorqueSaturation(PD_control_return_joint2[2], 2500);

			// real control input in permil + saturation applied  of FL controller 
			control_torque[0] = static_cast<int>(m_real_world_configurer.InvertTorquesign(sat_torque_PD[0]));
			control_torque[1] = static_cast<int>(m_real_world_configurer.InvertTorquesign(sat_torque_PD[1]));

			// control_torque[0] = 0.0;
			// control_torque[1] = 0.0;

			// csv file output writing
			output_file << theta_desired_d[0] <<"," <<jPos[0] <<","
						<< theta_desired_d[1] <<"," <<jPos[1] <<","
					    << theta_dot_desired_d[0] << "," << theta_dot_d_filtered[0] << "," 
					    << theta_dot_desired_d[1] << "," << theta_dot_d_filtered[1] << "," 
						<< control_torque[0] << ","
						<< control_torque[1] << ","
						<< propo_term_torque[0] << "," << deriv_term_torque[0] << "," 
						<< propo_term_torque[1] << "," << deriv_term_torque[1] << "," 
						<< current_time  << std::endl;
			break;
			}

		case ctrl_fl:
			{
			double global_theta_2_fixed = 0.0;
			double gear_ratio =  m_canManager.m_Dev[0].m_Gear_Ratio;
				
			std::array<double, 4>torque_calculate = m_FL_controller.calculateTau(theta_ddot_desired_d[0], joint_error[0], joint_error_dot[0],
                                							 					 jPos[0], global_theta_2_fixed, 
			 																	 theta_dot_d_filtered[0], theta_dot_d_filtered[1]);

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
			sat_torque_permil[1] = m_real_world_configurer.TorqueSaturation(torque_temp[1], 2700);

			// real control input in permil + saturation applied  of FL controller 
			control_torque[0] = static_cast<int>(m_real_world_configurer.InvertTorquesign(sat_torque_permil[0]));
			control_torque[1] = static_cast<int>(m_real_world_configurer.InvertTorquesign(sat_torque_permil[1]));

			printf("\ninput of FL controller in main is (%d, %d)\n", control_torque[0], control_torque[1]);

			// csv file output writing
			output_file << theta_desired_d[0] <<"," <<jPos[0] <<"," 
					    << theta_dot_desired_d[0] << "," << theta_dot_d_filtered[0] << "," 
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
			std::array<double, 4> torque_calculate = m_FL_controller.calculateTau(theta_ddot_desired_d[0], joint_error[0], joint_error_dot[0],
                                							 					 	  jPos[0], global_theta_2_fixed, 
			 																	      theta_dot_d_filtered[0], theta_dot_d_filtered[1]); // torque_Nm
			// printf("u_FL_1 : {%f}\n" , u_FL[0]);
			// printf("u_FL_2 : {%f}\n" , u_FL[1]);
			
			// printf("u_hat_1 : {%f}\n" , u_hat[0]);
			// printf("u_hat_2 : {%f}\n" , u_hat[1]);

			u_FL[0] = torque_calculate[0]; // [Nm]
			u_FL[1] = torque_calculate[1]; // [Nm]
		
			//DOB torque generation [Nm]
			// double time_constant = 5e-3; // 튜닝 파라미터
			double time_constant = 0.9;
			estimated_disturbance = m_DOB_controller.EstimateDisturbance(jPos[0], jPos[1], theta_dot_d_filtered[0], theta_dot_d_filtered[1], u_hat, time_constant);
			printf("estimated_disturbance_1 : {%f} \n" , estimated_disturbance[0]);
			printf("estimated_disturbance_2 : {%f} \n" , estimated_disturbance[1]);

			// estimated_disturbance_not_limited[0] = estimated_disturbance[0];
			// estimated_disturbance_not_limited[1] = estimated_disturbance[1];

			estimated_disturbance[0] = m_real_world_configurer.TorqueSaturation(estimated_disturbance[0], 120); // Nm
			estimated_disturbance[1] = 0.0; // m_real_world_configurer.TorqueSaturation(estimated_disturbance[1], 20 * 1e-3); // Nm

			u_hat[0] = u_FL[0] - estimated_disturbance[0]; //update
			u_hat[1] = u_FL[1] - estimated_disturbance[1]; //update
			// u_hat[0] = u_FL[0]; //update
			// u_hat[1] = u_FL[1]; //update		

			applied_tau[0] = u_FL[0]  - estimated_disturbance[0];
			applied_tau[1] = u_FL[1]  - estimated_disturbance[1];			
			// applied_tau[0] = u_FL[0]; //update
			// applied_tau[1] = u_FL[1]; //update

			double disturb_comp_temp[2];
			disturb_comp_temp[0] = m_real_world_configurer.Nm_to_permil(applied_tau[0], gear_ratio);
			disturb_comp_temp[1] = m_real_world_configurer.Nm_to_permil(applied_tau[1], gear_ratio);
			
			// printf("\ndisturb_comp_temp[0] is (%f)", disturb_comp_temp[0]);
			// printf("\ndisturb_comp_temp[1] is (%f)\n", disturb_comp_temp[1]);
			
			double ditsurbance_comp_input[2];
			ditsurbance_comp_input[0] = m_real_world_configurer.TorqueSaturation(disturb_comp_temp[0], 1000);
			ditsurbance_comp_input[1] = m_real_world_configurer.TorqueSaturation(disturb_comp_temp[1], 1000);
			
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
			
			// // csv file output writing
			output_file << theta_desired_d[0] <<"," <<jPos[0] <<"," 
					    << theta_dot_desired_d[0] << "," << theta_dot_d_filtered[0] << "," 
						<< applied_tau[0] << "," 
						<< estimated_disturbance[0]  
						<< "," << current_time  << std::endl;

			// csv file output writing(2)
			// output_file << theta_desired_d[0] <<"," <<jPos[0] <<"," 
			// 		    << theta_dot_desired_d[0] << "," << theta_dot_d_filtered[0] << "," 
			// 			<< applied_tau[0] << "," 
			// 			<< estimated_disturbance[0] << estimated_disturbance_not_limited[0] << "," 
			// 			<< "," << current_time  << std::endl;
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

	//////////////CDSL control field

	//csv file generation for PD 
	output_file << "joint1_pos_desired, joint1_pos, "
				<< "joint2_pos_desired, joint2_pos, "
			    << "joint1_vel_desired, joint1_vel_filtered, "
				<< "joint2_vel_desired, joint2_vel_filtered, "
				<< "target_torque_joint1, "
				<< "target_torque_joint2, "
				<< "propo_term_torque_joint1, deriv_term_torque_joint1, "
				<< "propo_term_torque_joint2, deriv_term_torque_joint2, "
				<< "current_time"
				<< std::endl;
	
	// //csv file generation for FL
	// output_file << "joint_pos_desired_1, joint_pos_1, joint_vel_desired_1, joint_vel_filtered_1, input torque_1, h term torque_1, current_time" << std::endl;
	
	// csv file generation for FL_DOB
	// output_file << "joint_pos_desired_1, joint_pos_1, joint_vel_desired_1, joint_vel_filtered_1, input torque_1_w.DOB[Nm], DOB Torque Term[Nm], current_time" << std::endl;
	
	// csv file generation for FL_DOB_2(estimated_disturbance term compare)
	// output_file << "joint_pos_desired_1, joint_pos_1, joint_vel_desired_1, joint_vel_filtered_1, input torque_1_w.DOB[Nm], DOB Torque Term[Nm], DOB Torque_Not_Limited Term[Nm], current_time" << std::endl;

	//////////////CDSL control field ends


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
