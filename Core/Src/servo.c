/*
 * servo.c
 *
 *  Created on: Mar 7, 2024
 *      Author: akswnd98
 */

#include "servo.h"
#include "main.h"

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim8;

void start_servos () {
	start_servo1();
	start_servo2();
	start_servo3();
	start_servo4();
}

void start_servo1 () {
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
}

void update_servo1_pwm (int signal) {
	htim3.Instance->CCR3 = signal;
}

void start_servo2 () {
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
}

void update_servo2_pwm (int signal) {
	htim3.Instance->CCR4 = signal;
}

void start_servo3 () {
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
}

void update_servo3_pwm (int signal) {
	htim8.Instance->CCR3 = signal;
}

void start_servo4 () {
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
}

void update_servo4_pwm (int signal) {
	htim8.Instance->CCR4 = signal;
}
