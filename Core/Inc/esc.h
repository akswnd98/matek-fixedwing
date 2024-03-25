/*
 * esc.h
 *
 *  Created on: Mar 6, 2024
 *      Author: akswnd98
 */

#ifndef INC_ESC_H_
#define INC_ESC_H_

void start_esc ();
void terminate_esc ();
void update_throttle (int throttle);
void skip_arming_mode ();
void do_calibration ();

#endif /* INC_ESC_H_ */
