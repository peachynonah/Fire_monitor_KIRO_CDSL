#pragma once

#define ctrl_manual 0
#define ctrl_pd 1
#define ctrl_fl 2
#define ctrl_fl_dob 3


class ManualController {
    public:
        ManualController();
        int tau;
        
    public:
        int calculateTau(int input_tau = 0);
};


class PDController {

    public:
        double Kp_PD[2];
        double Kd_PD[2];
        PDController();
        
        double tau[2];
        double tau_propo[2];
        double tau_deriv[2];
        double error[2];

    public:
        double calculateTau(int index, double joint_error, double joint_error_dot);
        double tauPropo(int index, double joint_error);
        double tauDeriv(int index, double joint_error_dot);
    
};


class FLController {
    public:
        double Kp_FL[2];
        double Kd_FL[2];
        FLController();
        
        double tau[2];
        double error[2];

        double tauPropo(int index, double joint_error);
        double tauDeriv(int index, double joint_error_dot);
        double tau_propo[2];
        double tau_deriv[2];

    public:
        // double calculateTau(int index, double joint_error, double joint_error_dot);
        double  calculateTau(int index, double theta1_ddot_desired_d, double joint_error, double joint_error_dot, 
                                  double theta1, double theta2, double theta1_dot, double theta2_dot, double gear_ratio);
        
    
};

class DOBController {
    private:
        LowPassFilter lowpassfilter;
    public:
        double x_a[2], x_b[2];
        double y_a[2], y_b[2];
        double y_a_prev[2], y_b_prev[2];
        double y_b_dot[2];
        double theta1_dot[2], theta2_dot[2];
        std::array<double, 2> estimated_disturbance;
        std::array<double, 2> u_hat; // u_hat = tau_FL - estimated_disturbance
        bool flag = false;
        DOBController();
    public:
        std::array<double, 2> EstimateDisturbance(double theta1, double theta2, 
        double theta1_dot, double theta2_dot, std::array<double, 2> u_hat, double time_constant=5e-3);
};

class LowPassFilter {
    public:
        double time_param;
        double unfiltered_value;
        double filtered_value;
        double filtered_value_prev;
        LowPassFilter();

    public:
        double calculate_lowpass_filter(double unfiltered_value, double filtered_value_prev);
};