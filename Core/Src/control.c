/*
 * control.c
 *
 *  Created on: Mar 14, 2024
 *      Author: akswnd98
 */

#include "math_utils.h"
#include "control.h"
#include "servo.h"
#include "kinematics.h"
#include <stdio.h>

extern float phi_ref;
extern float phi;
float phi_error_P_gain = 1;
extern float theta_ref;
extern float theta;
float theta_error_P_gain = 1;

extern float phi_dot;
extern float theta_dot;
extern float psi_dot;
float phi_dot_FF_gain = 100;
float phi_dot_error_P_gain = 0;
float phi_dot_error_I_gain = 0;
float theta_dot_FF_gain = 100;
float theta_dot_error_P_gain = 0;
float theta_dot_error_I_gain = 0;
float psi_dot_error_P_gain = 1;

int servo1_offset = 0;
int servo2_offset = 0;
int servo3_offset = 0;
int servo4_offset = 0;

float get_phi_dot_ref () {
	return (phi_ref - phi) * phi_error_P_gain;
}

float get_theta_dot_ref () {
	return (theta_ref - theta) * theta_error_P_gain;
}

float get_phi_dot_dot_ref (float phi_dot_ref) {
	return phi_dot_ref * phi_dot_FF_gain + (phi_dot_ref - phi_dot) * phi_dot_error_P_gain;
}

float get_theta_dot_dot_ref (float theta_dot_ref) {
	return theta_dot_ref * theta_dot_FF_gain + (theta_dot_ref - theta_dot) * theta_dot_error_P_gain;
}

float get_psi_dot_dot_ref () {
	return 0.0;
}

void get_w_dot_ref (float w_dot_ref[3]) {
	float eta_dot_dot[3] = {
		get_phi_dot_dot_ref(get_phi_dot_ref()),
		get_theta_dot_dot_ref(get_theta_dot_ref()),
		get_psi_dot_dot_ref()
	};
	float C[3][3];
	get_C(phi, theta, C);
	mul_mat_vec_3d(C, eta_dot_dot, w_dot_ref);
}

int saturate (int val, int bottom, int top) {
	return max(min(val, top), bottom);
}

int update_control_surface_angle_cnt = 0;
void update_control_surface_angle () {
	float w_dot_ref[3];
	get_w_dot_ref(w_dot_ref);
	update_servo1_pwm(saturate((int)w_dot_ref[0] + 1500 + servo1_offset, 1500 + servo1_offset - 400, 1500 + servo1_offset + 400));
	update_servo2_pwm(saturate(-(int)w_dot_ref[0] + 1500 + servo2_offset, 1500 + servo2_offset - 400, 1500 + servo2_offset + 400));
	update_servo3_pwm(saturate((int)w_dot_ref[1] + 1500 + servo3_offset, 1500 + servo3_offset - 400, 1500 + servo3_offset + 400));
	update_servo4_pwm(saturate((int)w_dot_ref[2] + 1500 + servo4_offset, 1500 + servo4_offset - 400, 1500 + servo4_offset + 400));
}
