/*
 * math.h
 *
 *  Created on: Mar 14, 2024
 *      Author: akswnd98
 */

#ifndef INC_MATH_UTILS_H_
#define INC_MATH_UTILS_H_

int max (int a, int b);
int min (int a, int b);
float max_f (float a, float b);
float min_f (float a, float b);
void mul_mat_vec_3d (float mat[3][3], float vec[3], float rst[3]);
void mul_mat_vec_4d (float mat[4][4], float vec[4], float rst[4]);
float dot_vec_3d (float vec1[3], float vec2[3]);
float dot_vec_4d (float vec1[4], float vec2[4]);
void add_vec_3d (float vec1[3], float vec2[3], float rst[3]);
void add_vec_4d (float vec1[4], float vec2[4], float rst[4]);
void sub_vec_3d (float vec1[3], float vec2[3], float rst[3]);
void sub_vec_4d (float vec1[4], float vec2[4], float rst[4]);
void dot_scalar_vec (float scalar, float vec[], float rst[], int dim);
void dot_scalar_vec_3d (float scalar, float vec[3], float rst[3]);
void dot_scalar_vec_4d (float scalar, float vec[4], float rst[4]);

#endif /* INC_MATH_UTILS_H_ */
