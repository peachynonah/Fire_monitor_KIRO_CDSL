#include "ReferenceGenerator.h"
#include <iostream>
#include <cmath>
#include <stdexcept>

// 생성자
ReferenceGenerator::ReferenceGenerator() {
    for (int i = 0; i < 6; i++) {
        alpha_coeffs[i] = 0.0;
    }
    time_ref_start = 0.0;
    time_ref_fin = 10.0;

    alpha_coeffs[5] = 6.637836e-06;
    alpha_coeffs[4] = -0.000282108;
    alpha_coeffs[3] = 0.003197224;
    alpha_coeffs[2] = 0.0;
    alpha_coeffs[1] = 0.0;
    alpha_coeffs[0] = 0.0;
}


// 특정 시간 t에 대한 목표 위치를 계산
double ReferenceGenerator::get_position(double current_time) {
    // 궤적 시작 전에는 초기 위치를 반환
    if (current_time < time_ref_start) {
        return 0.0;
    }

    // 궤적 끝난 후에는 최종 위치를 반환
    if (current_time >= time_ref_fin) {
        double t_diff_final = time_ref_fin - time_ref_start;
        return alpha_coeffs[5] * std::pow(t_diff_final, 5) +
               alpha_coeffs[4] * std::pow(t_diff_final, 4) +
               alpha_coeffs[3] * std::pow(t_diff_final, 3) +
               alpha_coeffs[2] * std::pow(t_diff_final, 2) +
               alpha_coeffs[1] * t_diff_final +
               alpha_coeffs[0];
    }
    
    // 궤적 구간 내에서는 다항식 계산
    double t_diff = current_time - time_ref_start;
    return alpha_coeffs[5] * std::pow(t_diff, 5) +
           alpha_coeffs[4] * std::pow(t_diff, 4) +
           alpha_coeffs[3] * std::pow(t_diff, 3) +
           alpha_coeffs[2] * std::pow(t_diff, 2) +
           alpha_coeffs[1] * t_diff +
           alpha_coeffs[0];
}

// // 특정 시간 t에 대한 목표 속도를 계산
// double ReferenceGenerator::get_velocity(double t) {
//     if (t < time_ref_start || t >= time_ref_fin) {
//         return 0.0;
//     }
    
//     double t_diff = t - time_ref_start;
//     return 5 * alpha_coeffs[0] * std::pow(t_diff, 4) +
//            4 * alpha_coeffs[1] * std::pow(t_diff, 3) +
//            3 * alpha_coeffs[2] * std::pow(t_diff, 2) +
//            2 * alpha_coeffs[3] * t_diff +
//            alpha_coeffs[4];
// }

// // 특정 시간 t에 대한 목표 가속도를 계산
// double ReferenceGenerator::get_acceleration(double t) {
//     if (t < time_ref_start || t >= time_ref_fin) {
//         return 0.0;
//     }
    
//     double t_diff = t - time_ref_start;
//     return 20 * alpha_coeffs[0] * std::pow(t_diff, 3) +
//            12 * alpha_coeffs[1] * std::pow(t_diff, 2) +
//            6 * alpha_coeffs[2] * t_diff +
//            2 * alpha_coeffs[3];
// }