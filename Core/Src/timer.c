/*
 * timer.c
 *
 *  Created on: Mar 5, 2024
 *      Author: akswnd98
 */

#include "main.h"
#include "timer.h"

extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;

void start_euler_state_update_tim () {
	HAL_TIM_Base_Start_IT(&htim5);
}

void start_control_surface_angle_update_tim () {
	HAL_TIM_Base_Start_IT(&htim6);
}

void start_debug_tim () {
	HAL_TIM_Base_Start_IT(&htim7);
}

uint8_t euler_state_update_it = 0;
uint8_t control_surface_angle_update_it = 0;
uint8_t debug_it = 0;
void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *htim) {
	if (htim->Instance == htim5.Instance) {
		euler_state_update_it = 1;
	}
	if (htim->Instance == htim6.Instance) {
		control_surface_angle_update_it = 1;
	}
	if (htim->Instance == htim7.Instance) {
		debug_it = 1;
	}
}
