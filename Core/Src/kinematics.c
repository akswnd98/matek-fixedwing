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

float I[3][3] = {
		{1, 0, 0},
		{0, 1, 0},
		{0, 0, 1}
};

float dt = 0.001;
float H = 0.004;

void mul_mat_vec_3d (float mat[3][3], float vec[3], float rst[3]) {
	for (int i = 0; i < 3; i++) {
		rst[i] = 0;
	}
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			rst[i] += mat[i][j] * vec[j];
		}
	}
}

void mul_mat_vec_4d (float mat[4][4], float vec[4], float rst[4]) {
	for (int i = 0; i < 4; i++) {
		rst[i] = 0;
	}
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			rst[i] += mat[i][j] * vec[j];
		}
	}
}

float dot_vec_3d (float vec1[3], float vec2[3]) {
	float ret = 0;
	for (int i = 0; i < 3; i++) {
		ret += vec1[i] * vec2[i];
	}
	return ret;
}

float dot_vec_4d (float vec1[4], float vec2[4]) {
	float ret = 0;
	for (int i = 0; i < 4; i++) {
		ret += vec1[i] * vec2[i];
	}
	return ret;
}

void add_vec_3d (float vec1[3], float vec2[3], float rst[3]) {
	for (int i = 0; i < 3; i++) {
		rst[i] = vec1[i] + vec2[i];
	}
}

void add_vec_4d (float vec1[4], float vec2[4], float rst[4]) {
	for (int i = 0; i < 4; i++) {
		rst[i] = vec1[i] + vec2[i];
	}
}

void sub_vec_3d (float vec1[3], float vec2[3], float rst[3]) {
	for (int i = 0; i < 3; i++) {
		rst[i] = vec1[i] - vec2[2];
	}
}

void sub_vec_4d (float vec1[4], float vec2[4], float rst[4]) {
	for (int i = 0; i < 4; i++) {
		rst[i] = vec1[i] - vec2[2];
	}
}

void dot_scalar_vec (float scalar, float vec[], float rst[], int dim) {
	for (int i = 0; i < dim; i++) {
		rst[i] = scalar * vec[i];
	}
}

void dot_scalar_vec_3d (float scalar, float vec[3], float rst[3]) {
	for (int i = 0; i < 3; i++) {
		rst[i] = scalar * vec[i];
	}
}

void dot_scalar_vec_4d (float scalar, float vec[4], float rst[4]) {
	for (int i = 0; i < 4; i++) {
		rst[i] = scalar * vec[i];
	}
}

void get_C (float phi, float theta, float C[3][3]) {
	C[0][0] = cos(theta);
	C[0][1] = 0;
	C[0][2] = -sin(theta) * cos(phi);

	C[1][0] = 0;
	C[1][1] = 1;
	C[1][2] = sin(phi);

	C[2][0] = sin(theta);
	C[2][1] = 0;
	C[2][2] = cos(phi) * cos(theta);
}

void get_C_inv (float phi, float theta, float C_inv[3][3]) {
	C_inv[0][0] = cos(theta);
	C_inv[0][1] = 0;
	C_inv[0][2] = sin(theta);

	C_inv[1][0] = sin(theta) * tan(phi);
	C_inv[1][1] = 1;
	C_inv[1][2] = -cos(theta) * tan(phi);

	C_inv[2][0] = -sin(theta) / cos(phi);
	C_inv[2][1] = 0;
	C_inv[2][2] = cos(theta) / cos(phi);
}

void get_C_dot (float phi, float theta, float eta_dot[3], float C_dot[3][3]) {
	C_dot[0][0] = -sin(theta);
	C_dot[0][1] = 0;
	C_dot[0][2] = sin(phi) * sin(theta) - cos(phi) * cos(theta);

	C_dot[1][0] = 0;
	C_dot[1][1] = 0;
	C_dot[1][2] = cos(phi);

	C_dot[2][0] = cos(theta);
	C_dot[2][1] = 0;
	C_dot[2][2] = -sin(phi) * cos(theta) - sin(theta) * cos(phi);
}



void update_euler_state () {
	float new_phi_high_freq = phi + phi_dot * dt;
	float new_theta_high_freq = theta + theta_dot * dt;
	float new_phi_low_freq = asin(-acc_y / sqrt(acc_x * acc_x + acc_y * acc_y + acc_z * acc_z));
	float new_theta_low_freq = atan(acc_x / (acc_z + 0.00001));
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
