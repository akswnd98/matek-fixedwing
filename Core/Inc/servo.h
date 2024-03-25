/*
 * servo.h
 *
 *  Created on: Mar 7, 2024
 *      Author: akswnd98
 */

#ifndef INC_SERVO_H_
#define INC_SERVO_H_

void start_servos ();

void start_servo1 ();
void update_servo1_pwm (int signal);
void start_servo2 ();
void update_servo2_pwm (int signal);
void start_servo3 ();
void update_servo3_pwm (int signal);
void start_servo4 ();
void update_servo4_pwm (int signal);
void start_servo5 ();
void update_servo5_pwm (int signal);

#endif /* INC_SERVO_H_ */
