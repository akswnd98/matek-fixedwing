/*
 * math.c
 *
 *  Created on: Mar 14, 2024
 *      Author: akswnd98
 */

#include "math_utils.h"

int max (int a, int b) {
	return a > b ? a : b;
}

int min (int a, int b) {
	return a < b ? a : b;
}

float max_f (float a, float b) {
	return a > b ? a : b;
}

float min_f (float a, float b) {
	return a < b ? a : b;
}

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
