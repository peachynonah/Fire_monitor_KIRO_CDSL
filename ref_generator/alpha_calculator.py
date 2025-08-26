import numpy as np

# --- 1. 행렬 정의 ---
time_ref_start = 0.0; time_ref_fin = 10.0; 
tilde_time_sf = time_ref_fin - time_ref_start
# sampling_period = 0.001

## joint 1 
[theta_ref_start_1, theta_dot_ref_start_1, theta_ddot_ref_start_1] = [0.0, 0.0, 0.0]
[theta_ref_fin_1, theta_dot_ref_fin_1, theta_ddot_ref_fin_1] = [0.0, 0.0, 0.0]
theta_ref_fin_1 = (np.pi/2)

## joint 2 
[theta_ref_start_2, theta_dot_ref_start_2, theta_ddot_ref_start_2] = [0.0, 0.0, 0.0]
[theta_ref_fin_2, theta_dot_ref_fin_2, theta_ddot_ref_fin_2] = [0.0, 0.0, 0.0]
theta_ref_fin_2 = (np.pi/2)

## matrix calculation
tsf = tilde_time_sf
PHI = np.array([
    [0,                 0,        0,        0,        0,     1],
    [0,                 0,        0,        0,        1,     0],
    [0,                 0,        0,        2,        0,     0],  
    [   tsf**5,    tsf**4,   tsf**3,   tsf**2, tsf**1,       1],
    [ 5*tsf**4,  4*tsf**3, 3*tsf**2,    2*tsf,      1,       0],
    [20*tsf**3, 12*tsf**2,    6*tsf,        2,      0,       0]
])

print("Matrix PHI:")
print(PHI)
PHI_inverse = np.linalg.inv(PHI)

# --- 2. 결과 출력 ---
THETA_star_1 = np.array([theta_ref_start_1, 
                        theta_dot_ref_start_1, 
                        theta_ddot_ref_start_1,
                        theta_ref_fin_1, 
                        theta_dot_ref_fin_1, 
                        theta_ddot_ref_fin_1])

alpha_1 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
alpha_1 = np.dot(PHI_inverse, THETA_star_1)

print("\nalpha_5:", alpha_1[0])
print("\nalpha_4:", alpha_1[1])
print("\nalpha_3:", alpha_1[2])
print("\nalpha_2:", alpha_1[3])
print("\nalpha_1:", alpha_1[4])
print("\nalpha_0:", alpha_1[5])
