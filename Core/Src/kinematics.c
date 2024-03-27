/*
 * kinematics.c
 *
 *  Created on: Mar 11, 2024
 *      Author: akswnd98
 */

#include "math.h"
#include "math_utils.h"
#include "kinematics.h"

extern float acc_x;
extern float acc_y;
extern float acc_z;
extern float gyro_x;
extern float gyro_y;
extern float gyro_z;

float phi = 0;
float theta = 0;
float phi_dot = 0;
float theta_dot = 0;
float psi_dot = 0;

float dt = 0.001;
float H = 0.004;

void get_C (float phi, float theta, float C[3][3]) {
	C[0][0] = 1.0;
	C[0][1] = 0.0;
	C[0][2] = -sin(theta);

	C[1][0] = 0.0;
	C[1][1] = cos(phi);
	C[1][2] = sin(phi) * cos(theta);

	C[2][0] = 0.0;
	C[2][1] = -sin(phi);
	C[2][2] = cos(phi) * cos(theta);
}

void get_C_inv (float phi, float theta, float C_inv[3][3]) {
	C_inv[0][0] = 1.0;
	C_inv[0][1] = sin(phi) * tan(theta);
	C_inv[0][2] = cos(phi) * tan(theta);

	C_inv[1][0] = 0.0;
	C_inv[1][1] = cos(phi);
	C_inv[1][2] = -sin(phi);

	C_inv[2][0] = 0.0;
	C_inv[2][1] = sin(phi) / cos(theta);
	C_inv[2][2] = cos(phi) / cos(theta);
}

void update_euler_state () {
	float new_phi_high_freq = phi + phi_dot * dt;
	float new_theta_high_freq = theta + theta_dot * dt;
	float new_phi_low_freq = atan(acc_y / (acc_z + 0.00001));
	float new_theta_low_freq = asin(acc_x / sqrt(acc_x * acc_x + acc_y * acc_y + acc_z * acc_z));
	phi = new_phi_low_freq * H + new_phi_high_freq * (1.0 - H);
	theta = new_theta_low_freq * H + new_theta_high_freq * (1.0 - H);

	float C_inv[3][3];
	get_C_inv(phi, theta, C_inv);

	float w[3] = {gyro_x, gyro_y, gyro_z};

	float eta_dot[3];
	mul_mat_vec_3d(C_inv, w, eta_dot);
	phi_dot = eta_dot[0];
	theta_dot = eta_dot[1];
	psi_dot = eta_dot[2];
}
