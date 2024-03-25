/*
 * timer.h
 *
 *  Created on: Mar 5, 2024
 *      Author: akswnd98
 */

#ifndef INC_TIMER_H_
#define INC_TIMER_H_

void start_euler_state_update_tim ();
void start_control_surface_angle_update_tim ();
void start_debug_tim();
void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *htim);

#endif /* INC_TIMER_H_ */
