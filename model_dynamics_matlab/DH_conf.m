function [T_im1_i] = DH_conf(alpha_im1, a_im1, d_i, theta_i)
% DH_TRANS  Denavit–Hartenberg homogeneous transform (two forms)

    % 직접 입력한 T_im1_i
    T_im1_i = [ ...
                cos(theta_i),               -sin(theta_i),                      0,                    a_im1;
 sin(theta_i)*cos(alpha_im1), cos(theta_i)*cos(alpha_im1),        -sin(alpha_im1),      -sin(alpha_im1)*d_i;
 sin(theta_i)*sin(alpha_im1), cos(theta_i)*sin(alpha_im1),         cos(alpha_im1),       cos(alpha_im1)*d_i;
                           0,                           0,                      0,                1       ];
end