/*
 * gain_receive.h
 *
 *  Created on: Mar 18, 2024
 *      Author: akswnd98
 */

#ifndef INC_TUNING_RECEIVE_H_
#define INC_TUNING_RECEIVE_H_

#include "main.h"

#define NUM_OF_TUNINGS 16

#define PHI_ERROR_P_IDX 0
#define W_X_FF_IDX 1
#define W_X_ERROR_P_IDX 2
#define W_X_ERROR_I_IDX 3

#define THETA_ERROR_P_IDX 4
#define W_Y_FF_IDX 5
#define W_Y_ERROR_P_IDX 6
#define W_Y_ERROR_I_IDX 7

#define W_Z_FF_IDX 8
#define W_Z_ERROR_P_IDX 9
#define W_Z_ERROR_I_IDX 10

#define SERVO1_OFFSET_IDX 11
#define SERVO2_OFFSET_IDX 12
#define SERVO3_OFFSET_IDX 13
#define SERVO4_OFFSET_IDX 14

#define CRUISE_SPEED_IDX 15

#define PHI_ERROR_P_GAIN_FS 100.0
#define W_X_FF_GAIN_FS 500.0
#define W_X_ERROR_P_GAIN_FS 200.0
#define W_X_ERROR_I_GAIN_FS 100.0

#define THETA_ERROR_P_GAIN_FS 100.0
#define W_Y_FF_GAIN_FS 500.0
#define W_Y_ERROR_P_GAIN_FS 200.0
#define W_Y_ERROR_I_GAIN_FS 100.0

#define W_Z_FF_GAIN_FS 500.0
#define W_Z_ERROR_P_GAIN_FS 200.0
#define W_Z_ERROR_I_GAIN_FS 100.0

#define CRUISE_SPEED_FS 50

extern uint16_t tuning[20];

void process_tuning_receive (uint8_t data);

#endif /* INC_TUNING_RECEIVE_H_ */
