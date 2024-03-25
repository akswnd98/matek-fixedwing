/*
 * kinematics.h
 *
 *  Created on: Mar 11, 2024
 *      Author: akswnd98
 */

#ifndef INC_KINEMATICS_H_
#define INC_KINEMATICS_H_

void mul_mat_vec_3d (float mat[3][3], float vec[3], float rst[3]);
void mul_mat_vec_4d (float mat[4][4], float vec[4], float rst[4]);
float dot_vec_3d (float vec1[3], float vec2[3]);
float dot_vec_4d (float vec1[4], float vec2[4]);
void add_vec_3d (float vec1[3], float vec2[3], float rst[3]);
void add_vec_4d (float vec1[4], float vec2[4], float rst[4]);
void sub_vec_3d (float vec1[3], float vec2[3], float rst[3]);
void sub_vec_4d (float vec1[4], float vec2[4], float rst[4]);
void dot_scalar_vec_3d (float scalar, float vec[3], float rst[3]);
void dot_scalar_vec_4d (float scalar, float vec[4], float rst[4]);
void get_R (float eta[3], float R[3][3]);
void get_R_inv (float eta[3], float R_inv[3][3]);
void get_C (float phi, float theta, float C[3][3]);
void get_C_inv (float phi, float theta, float C_inv[3][3]);
void get_C_dot (float phi, float theta, float eta_dot[3], float C_dot[3][3]);

void update_euler_state ();

#endif /* INC_KINEMATICS_H_ */
