/*
 * esc.c
 *
 *  Created on: Mar 6, 2024
 *      Author: akswnd98
 */

#include "main.h"
#include "esc.h"

extern TIM_HandleTypeDef htim4;

void start_esc () {
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
}

void terminate_esc () {
  HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2);
}

void update_throttle (int throttle) {
  htim4.Instance->CCR2 = throttle;
}

void skip_arming_mode () {
  update_throttle(50);
  HAL_Delay(100);
  update_throttle(0);
  HAL_Delay(3000);
  update_throttle(1000);
  HAL_Delay(3000);
}

void do_calibration () {
  update_throttle(1980);
  HAL_Delay(10000);
  update_throttle(1010);
  HAL_Delay(10000);
}
