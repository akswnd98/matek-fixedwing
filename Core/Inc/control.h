/*
 * control.h
 *
 *  Created on: Mar 14, 2024
 *      Author: akswnd98
 */

#ifndef INC_CONTROL_H_
#define INC_CONTROL_H_

#define SAT(VAL, BOTTOM_SAT, TOP_SAT) MIN(MAX(BOTTOM_SAT, VAL), TOP_SAT)
#define SERVO_BOTTOM_SAT 1000
#define SERVO_TOP_SAT 2000

float get_phi_dot_ref ();
float get_theta_dot_ref ();
float get_phi_dot_dot_ref (float phi_dot_ref);
float get_theta_dot_dot_ref (float theta_dot_ref);
float get_psi_dot_dot_ref ();
void get_w_dot_ref (float w_dot_ref[3]);
int saturate (int val, int bottom, int top);
void update_control_surface_angle ();

#endif /* INC_CONTROL_H_ */
