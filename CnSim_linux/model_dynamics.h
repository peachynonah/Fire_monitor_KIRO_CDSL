#pragma once
# include <iostream>
# include <cmath>
// # include <vector>
# include <array>

#define MM_to_M 1e-3

class ModelDynamics {
    public:
        ModelDynamics();
        //Robot properties
        double DH_param_dist[2];
        double link_mass[2];
        double com_x[2];
        double com_y[2];
        double com_z[2];
        double grav_acc;
        double L_2;
        double x21, y21, z21;
        double mass_matrix[2][2];
        double nonlinear_dynamics_term[2];

    public:
        std::array<double, 4> get_mass_matrix(double theta1, double theta2);
        std::array<double, 2> get_nonlinear_dynamics(double theta1, double theta2, double theta1_dot, double theta2_dot);
        double h1_cori, h1_cent, h1_grav;
        double h2_cori, h2_cent, h2_grav;
};

