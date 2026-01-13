syms d_1 d_2 theta_1(t) theta_2(t) ...
     x_11 y_11 z_11 ...
     x_22 y_22 z_22 ...
     m1 m2 g_c

%% Frame translation of Center of Mass
% Translation Matrix
T_01 = DH_conf(0, 0, d_1, theta_1(t)); 
T_12 = DH_conf((sym(pi)/2), 0, d_2, theta_2(t));

% Link 1, Center of Mass frame change to global frame 
P_cm1_fr1 = [x_11; y_11; z_11; 1];
P_cm1_fr0 = T_01 * P_cm1_fr1;
P_cm1_fr0 = simplify(P_cm1_fr0);
disp('P_cm1_fr0 is = '); disp(vpa(P_cm1_fr0 , 6));

% Link 2, Center of Mass frame change to global frame
P_cm2_fr2 = [x_22; y_22; z_22; 1];
P_cm2_fr0 = T_01 * T_12* P_cm2_fr2;
P_cm2_fr0 = simplify(P_cm2_fr0);
% disp('P_cm2_fr0 is = '); disp(vpa(P_cm2_fr0 , 6));

%% Lagrangian variable 
% Lagrangian variable L1
x_1 = P_cm1_fr0(1); y_1 = P_cm1_fr0(2); z_1 = P_cm1_fr0(3);
dx_1 = diff(x_1, t); dy_1 = diff(y_1, t); dz_1 = diff(z_1, t);
Kin_1 = (1/2) * m1 * (dx_1.^2 + dy_1.^2 + dz_1.^2);
Pot_1 = m1 * g_c * y_1;
Lag_1 = Kin_1 - Pot_1;
% disp('Lagrangian variable of link 1 is ='); disp(Lag_1);

% Lagrangian variable L2
x_2 = P_cm2_fr0(1); y_2 = P_cm2_fr0(2);  z_2 = P_cm2_fr0(3);
dx_2 = diff(x_2, t); dy_2 = diff(y_2, t); dz_2 = diff(z_2, t);
Kin_2 = (1/2) * m2 * (dx_2.^2 + dy_2.^2 + dz_2.^2);
Pot_2 = m2 * g_c * y_2;
Lag_2 = Kin_2 - Pot_2;
% disp('Lagrangian variable of link 2 is ='); disp(Lag_2);

Lag_tot = Lag_1 + Lag_2;

%% Lagrangian Dynamics
% Lagrangian Dynamics tau 1
Lag_diff_theta_1 = diff(Lag_tot, theta_1);
Lag_diff_thetadot_1 = diff(Lag_tot, diff(theta_1, t));
tau_1 = diff(Lag_diff_thetadot_1, t) - Lag_diff_theta_1;

Lag_diff_theta_1 = simplify(Lag_diff_theta_1);
Lag_diff_thetadot_1 = simplify(Lag_diff_thetadot_1);
tau_1 = simplify(tau_1);

% disp('tau_1 is ='); disp(tau_1);

% Lagrangian Dynamics tau 2
Lag_diff_theta_2 = diff(Lag_tot, theta_2);
Lag_diff_thetadot_2 = diff(Lag_tot, diff(theta_2, t));
tau_2 = diff(Lag_diff_thetadot_2, t) - Lag_diff_theta_2;

Lag_diff_theta_2 = simplify(Lag_diff_theta_2);
Lag_diff_thetadot_2 = simplify(Lag_diff_thetadot_2);
tau_2 = simplify(tau_2);

% disp('tau_2 is ='); disp(tau_2);

%% 수식 정리
syms th1(t) th2(t) thd1(t) thd2(t) thdd1(t) thdd2(t)

tau_1_c =  subs(tau_1, [theta_1(t), theta_2(t)], ...
                     [    th1(t),     th2(t)]);
tau_1_c =  subs(tau_1_c, [diff(th1(t), t), diff(th2(t), t)], ...
                     [        thd1(t),         thd2(t)]);
tau_1_c =  subs(tau_1_c, [diff(thd1(t), t), diff(thd2(t), t)], ...
                     [        thdd1(t),         thdd2(t)]);

% disp('tau_1_clean is ='); disp(tau_1_c);

tau_2_c =  subs(tau_2, [theta_1(t), theta_2(t)], ...
                     [    th1(t),     th2(t)]);
tau_2_c =  subs(tau_2_c, [diff(th1(t), t), diff(th2(t), t)], ...
                     [        thd1(t),         thd2(t)]);
tau_2_c =  subs(tau_2_c, [diff(thd1(t), t), diff(thd2(t), t)], ...
                     [        thdd1(t),         thdd2(t)]);

disp('tau_2_clean is ='); disp(tau_2_c);


%% Mass matrix 계수 추출

[m_11, T_1] = coeffs(tau_1_c, thdd1(t));
[m_12, T_2] = coeffs(tau_1_c, thdd2(t));
% 출력된 m_11 벡터의 첫번째 원소가 m_11
% disp('m_11 is ='); disp(m_11); disp(T_1);
% disp('m_12 is ='); disp(m_12); disp(T_2);

[m_21, T_3] = coeffs(tau_2_c, thdd1(t));
[m_22, T_4] = coeffs(tau_2_c, thdd2(t));
% disp('m_21 is ='); disp(m_21); disp(T_3);
% disp('m_22 is ='); disp(m_22); disp(T_4);