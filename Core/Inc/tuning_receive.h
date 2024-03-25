/*
 * gain_receive.h
 *
 *  Created on: Mar 18, 2024
 *      Author: akswnd98
 */

#ifndef INC_TUNING_RECEIVE_H_
#define INC_TUNING_RECEIVE_H_

#include "main.h"

#define NUM_OF_TUNINGS 14

#define PHI_ERROR_P_IDX 0
#define PHI_DOT_FF_IDX 1
#define PHI_DOT_ERROR_P_IDX 2
#define PHI_DOT_ERROR_I_IDX 3
#define THETA_ERROR_P_IDX 4
#define THETA_DOT_FF_IDX 5
#define THETA_DOT_ERROR_P_IDX 6
#define THETA_DOT_ERROR_I_IDX 7
#define PSI_DOT_ERROR_P_IDX 8
#define SERVO1_OFFSET 10
#define SERVO2_OFFSET 11
#define SERVO3_OFFSET 12
#define SERVO4_OFFSET 13

#define PHI_ERROR_P_GAIN_FS 100.0
#define PHI_DOT_FF_GAIN_FS 500.0
#define PHI_DOT_ERROR_P_GAIN_FS 200.0
#define PHI_DOT_ERROR_I_GAIN_FS 100.0
#define THETA_ERROR_P_GAIN_FS 100.0
#define THETA_DOT_FF_GAIN_FS 500.0
#define THETA_DOT_ERROR_P_GAIN_FS 200.0
#define THETA_DOT_ERROR_I_GAIN_FS 100.0
#define PSI_DOT_ERROR_P_GAIN_FS 100.0

extern uint16_t tuning[20];

void process_tuning_receive (uint8_t data);

#endif /* INC_TUNING_RECEIVE_H_ */
