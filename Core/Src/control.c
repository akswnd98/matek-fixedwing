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
#include "math.h"

float cruise_speed = 20.0;
extern float g;

extern float phi_ref;
extern float phi;
extern float theta_ref;
extern float theta;

float phi_error_P_gain = 1.0;
float w_x_FF_gain = 100.0;
float w_x_error_P_gain = 0.0;
float w_x_error_I_gain = 0.0;

float theta_error_P_gain = 1.0;
float w_y_FF_gain = 100.0;
float w_y_error_P_gain = 0.0;
float w_y_error_I_gain = 0.0;

float w_z_FF_gain = 100.0;
float w_z_error_P_gain = 0.0;
float w_z_error_I_gain = 0.0;

extern float gyro_x;
extern float gyro_y;
extern float gyro_z;

float w_ref[3] = {0.0, 0.0, 0.0};
float w_dot_ref[3] = {0.0, 0.0, 0.0};

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

float get_psi_dot_ref () {
	return g / (cruise_speed + 0.00001) * tan(phi_ref) * cos(theta_ref);
}

void get_w_ref (float w_ref[3]) {
	float eta_dot[3] = {
		get_phi_dot_ref(),
		get_theta_dot_ref(),
		get_psi_dot_ref()
	};
	float C[3][3];
	get_C(phi, theta, C);
	mul_mat_vec_3d(C, eta_dot, w_ref);
}

void get_w_dot_ref (float w_ref[3], float w_dot_ref[3]) {
	w_dot_ref[0] = w_ref[0] * w_x_FF_gain + (w_ref[0] - gyro_x) * w_x_error_P_gain;
	w_dot_ref[1] = w_ref[1] * w_y_FF_gain + (w_ref[1] - gyro_y) * w_y_error_P_gain;
	w_dot_ref[2] = w_ref[2] * w_z_FF_gain + (w_ref[2] - gyro_z) * w_z_error_P_gain;
}

int saturate (int val, int bottom, int top) {
	return max(min(val, top), bottom);
}

int update_control_surface_angle_cnt = 0;
void update_control_surface_angle () {
	get_w_ref(w_ref);
	get_w_dot_ref(w_ref, w_dot_ref);
	update_servo1_pwm(saturate((int)w_dot_ref[0] + 1500 + servo1_offset, 1500 + servo1_offset - 400, 1500 + servo1_offset + 400));
	update_servo2_pwm(saturate(-(int)w_dot_ref[0] + 1500 + servo2_offset, 1500 + servo2_offset - 400, 1500 + servo2_offset + 400));
	update_servo3_pwm(saturate((int)w_dot_ref[1] + 1500 + servo3_offset, 1500 + servo3_offset - 400, 1500 + servo3_offset + 400));
	update_servo4_pwm(saturate((int)w_dot_ref[2] + 1500 + servo4_offset, 1500 + servo4_offset - 400, 1500 + servo4_offset + 400));
}
