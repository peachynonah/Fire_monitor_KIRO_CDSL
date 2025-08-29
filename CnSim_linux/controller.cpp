# include <iostream>
# include "controller.h"
# include "model_dynamics.h"
// # include <vector>
# include <array>

using namespace std;

ModelDynamics m_model_dynamics;

// functions
double torque_saturate(double torque, double max_torque_norm) {
    double max_torque = max_torque_norm;
    double min_torque = -max_torque_norm;

    if (torque > max_torque) {
        return max_torque;
    } else if (torque < min_torque) {
        return min_torque;
    }
    
    return torque;
}

double Nm_to_permil(double tau, double gear_ratio){    
    tau = 1e3* tau; // Nm to mNm
    tau = (tau/gear_ratio);
    tau = ((1e3 / 52.8)* tau);

    return tau;
}



RealWorldConfigurer::RealWorldConfigurer(){

}

double RealWorldConfigurer::Nm_to_permil(double tau_in_Nm_linkside, double gear_ratio){
    double tau_in_mNm_linkside = 1e3* tau_in_Nm_linkside; // Nm to mNm
    double tau_in_mNm_mortorside = (tau_in_mNm_linkside/gear_ratio);
    double tau_in_permil = (1e3 / 52.8)* tau_in_mNm_mortorside;

    return tau_in_permil;
}

double RealWorldConfigurer::TorqueSaturation(double torque_permil,double torque_limit){
    double max_torque = torque_limit;
    double min_torque = -torque_limit;

    if (torque_permil > max_torque) {
        return max_torque;
    } else if (torque_permil < min_torque) {
        return min_torque;
    }
    
    return torque_permil;
}

double RealWorldConfigurer::InvertTorquesign(double torque_before_inverted){
    double torque_inverted = - torque_before_inverted;
    return torque_inverted;
}

// double get_velocity_numerical(double theta_d_curr, double theta_d_prev, double sampling_period){
//     double theta_dot_d = (theta_d_curr - theta_d_prev) / (sampling_period);
//     return theta_dot_d;
// }


//controller implementations

ManualController::ManualController() {
    // Constructor to initialize the manual controller
}

int ManualController::calculateTau(int input_tau) {
    // Implement the calculation for tau
    tau = torque_saturate(input_tau, 990); // Example max torque norm
    return tau;
}


PDController::PDController() {
    // Initialize gains
    Kp_PD[0] = 0.0; Kp_PD[1] = 5500.0; 
    Kd_PD[0] = 0.0; Kd_PD[1] = 1500.0; 
}

std::array<double,3> PDController::calculateTau(int index, double theta_desired_d, double theta_current, 
                                                double theta_dot_desired_d, double theta_dot_d_filtered){

    tau_propo[index] = Kp_PD[index] * (theta_desired_d - theta_current); 
    tau_deriv[index] =  Kd_PD[index] * (theta_dot_desired_d - theta_dot_d_filtered);
    tau[index] = tau_propo[index] + tau_deriv[index];
    // printf("\nin controller, generated torque is  : %f\n", tau[index]);    
    return {tau_propo[index], tau_deriv[index], tau[index]};
}

// //to develop
// double PDController::tauPropo(int index, double joint_error) {
//     tau_propo[index] = Kp_PD[index] * joint_error;
//     // printf("\nin controller, generated torque is  : %f\n", tau[index]);
//     tau_propo[index] = - torque_saturate(tau_propo[index], 2700);    
//     return tau_propo[index];
// }

// double PDController::tauDeriv(int index, double joint_error_dot) {
//     tau_deriv[index] = Kd_PD[index] * joint_error_dot;
//     // printf("\nin controller, generated torque is  : %f\n", tau[index]);
//     tau_deriv[index] = - torque_saturate(tau_deriv[index], 2700);    
//     return tau_deriv[index];
//}


FLController::FLController() {
    // Initialize gains
    Kp_FL[0] = 2500.0; Kp_FL[1] = 0.0;
    Kd_FL[0] = 3500.0; Kd_FL[1] = 0.0;
}

std::array<double, 4>FLController::calculateTau(double theta1_ddot_desired_d, double joint1_error, double joint1_error_dot, 
                                  double theta1, double theta2, double theta1_dot, double theta2_dot){
    tau[0] = 0.0;
    tau[1] = 0.0;
    //ModelReference: This is a placeholder for FL controller's tau calculation
    std::array<double, 4> mass_matrix = m_model_dynamics.get_mass_matrix(theta1, theta2);
    double m11 = mass_matrix[0]; double m12 = mass_matrix[1]; 
    double m21 = mass_matrix[2]; double m22 = mass_matrix[3]; 
    // printf("\nin FL controller, the mass matrix {m11, m12, m21, m22 is}: (%f, %f, %f, %f)\n", m11, m12, m21, m22);	
    
    std::array<double, 2> nonlinear_dynamics_term = m_model_dynamics.get_nonlinear_dynamics(theta1, theta2, theta1_dot, theta2_dot);
    double h1 = nonlinear_dynamics_term[0];
    double h2 = nonlinear_dynamics_term[1];
   // printf("\nin FL controller, the nonlinear term h {h1, h2 is}: (%f, %f)\n", h1, h2);

    //temp..
    double joint2_error = 0.0; double joint2_error_dot = 0.0; double theta2_ddot_desired_d = 0.0;
    double temp1 = theta1_ddot_desired_d + Kp_FL[0] * joint1_error + Kd_FL[0] * joint1_error_dot;
    double temp2 = theta2_ddot_desired_d + Kp_FL[1] * joint2_error + Kd_FL[1] * joint2_error_dot;
 
    tau_nonlinear_term[0] = h1;
    tau_nonlinear_term[1] = h2;
    
    tau[0] = m11 * (temp1) + m12 * (temp2) + h1; // Nm
    tau[1] = m21 * (temp1) + m22 * (temp2) + h2;

    return {tau[0], tau[1], tau_nonlinear_term[0], tau_nonlinear_term[1]}; // in Nm, torque minus is required
}


DOBController::DOBController() {
}
LowPassFilter m_lowpassfilter_dob;

std::array<double, 2> DOBController::EstimateDisturbance(double theta1, double theta2,
    double theta1_dot, double theta2_dot, std::array<double, 2> u_hat, double time_constant){
    static bool flag = false;
    if (flag == false){
        flag = true;
        for (int i=0; i<2; i++){
            y_a_prev[i] = 0.0;
            y_b_prev[i] = 0.0;
        }
    }
    x_b[0] = theta1_dot;
    x_b[1] = theta2_dot;

    for (int i=0; i<2; i++){
        x_a[i] = u_hat[i];
        //x_b[i] = theta1_dot;
        
        y_a[i] = m_lowpassfilter_dob.calculate_lowpass_filter(x_a[i], y_a_prev[i], time_constant);
        y_b[i] = m_lowpassfilter_dob.calculate_lowpass_filter(x_b[i], y_b_prev[i], time_constant);
        y_b_dot[i] = (-y_b[i] + x_b[i]) / time_constant; //jPos_two_dot
        
        //update previous values
        y_a_prev[i] = y_a[i];
        y_b_prev[i] = y_b[i];
    }
	
    //estimated_disturbance = M(theta){y_b_dot - h(theta, theta_dot)} - y_a
    std::array<double, 4> mass_matrix = m_model_dynamics.get_mass_matrix(theta1, theta2);
    double m11 = mass_matrix[0]; double m12 = mass_matrix[1]; 
    double m21 = mass_matrix[2]; double m22 = mass_matrix[3]; 

    std::array<double, 2> nonlinear_dynamics_term = m_model_dynamics.get_nonlinear_dynamics(theta1, theta2, theta1_dot, theta2_dot);
    double h1 = nonlinear_dynamics_term[0];
    double h2 = nonlinear_dynamics_term[1];

    printf("m11 : {%f}\n", m11);
    printf("m12 : {%f}\n", m12);
    printf("m21 : {%f}\n", m21);
    printf("m22 : {%f}\n", m22);
    printf("h1 : {%f}\n", h1);
    printf("h2 : {%f}\n", h2);
    
    printf("y_a_prev[0] : {%f}\n", y_a_prev[0]);
    printf("y_a_prev[1] : {%f}\n", y_a_prev[1]);
    printf("y_b_dot[0] : {%f}\n", y_b_dot[0]);
    
    
    estimated_disturbance[0] = m11 * (y_b_dot[0]) + m12 * (y_b_dot[1]) + h1 - y_a[0];
    estimated_disturbance[1] = m21 * (y_b_dot[0]) + m22 * (y_b_dot[1]) + h2 - y_a[1];
    // estimated_disturbance[0] = m11 * (y_b_dot[0]) + m12 * (y_b_dot[1]) + h1;
    // estimated_disturbance[1] = - y_a[0]; 
    return estimated_disturbance;
}


// LowPassFilter::LowPassFilter(){
//     time_param = 0.444; // time_param = sampling_time / (time_constant + sampling_time) = 4ms / (5ms + 4ms)
//     unfiltered_value = 0.0;
//     filtered_value = 0.0;
//     filtered_value_prev = 0.0;
// }

// double LowPassFilter::calculate_lowpass_filter(double unfiltered_value, double filtered_value_prev) {
//     filtered_value = time_param * unfiltered_value + (1 - time_param) * filtered_value_prev;
//     return filtered_value;
// }//y_k = time_param * u_k + (1 - time_param) * y_k-1




LowPassFilter::LowPassFilter(){
    time_constant = 5e-3; // time_constant = 5ms
    // time_param = sampling_time / (time_constant + sampling_time) = 4ms / (5ms + 4ms) = 0.444
    unfiltered_value = 0.0;
    filtered_value = 0.0;
    filtered_value_prev = 0.0;
}

double LowPassFilter::calculate_lowpass_filter(double unfiltered_value, double filtered_value_prev, double time_constant) {
    double time_param = 4e-3 / (time_constant + 4e-3);
    filtered_value = time_param * unfiltered_value + (1 - time_param) * filtered_value_prev;
    return filtered_value;
}//y_k = time_param * u_k + (1 - time_param) * y_k-1