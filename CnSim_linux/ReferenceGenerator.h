#pragma once
#include <array>

#define clockwise 0
#define anticlockwise 1


class ReferenceGenerator {
public:
    // 생성자
    ReferenceGenerator(int rotate_direction, std::array<double, 6> alpha_coeffs_, double final_operating_time);
    
    // 5차 다항식 궤적 계수를 계산하는 함수
    // Input:
    //   t_s: 시작 시간
    //   t_f: 끝 시간
    //   theta_s: 시작 위치
    //   theta_dot_s: 시작 속도
    //   theta_ddot_s: 시작 가속도
    //   theta_f: 끝 위치
    //   theta_dot_f: 끝 속도
    //   theta_ddot_f: 끝 가속도

    // 특정 시간 t에 대한 목표 위치를 계산
    double get_position(double current_time);
    
    // 특정 시간 t에 대한 목표 속도를 계산
    double get_velocity(double current_time);
    
    // 특정 시간 t에 대한 목표 가속도를 계산
    double get_acceleration(double current_time);

private:
    double alpha_coeffs[6];
    double time_ref_start, time_ref_fin;
    double current_joint1_position;
};