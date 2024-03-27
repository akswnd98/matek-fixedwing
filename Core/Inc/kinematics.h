/*
 * kinematics.h
 *
 *  Created on: Mar 11, 2024
 *      Author: akswnd98
 */

#ifndef INC_KINEMATICS_H_
#define INC_KINEMATICS_H_

void get_R (float eta[3], float R[3][3]);
void get_R_inv (float eta[3], float R_inv[3][3]);
void get_C (float phi, float theta, float C[3][3]);
void get_C_inv (float phi, float theta, float C_inv[3][3]);
void get_C_dot (float phi, float theta, float eta_dot[3], float C_dot[3][3]);

void update_euler_state ();

#endif /* INC_KINEMATICS_H_ */
