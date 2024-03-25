/*
 * gain_receive.c
 *
 *  Created on: Mar 18, 2024
 *      Author: akswnd98
 */

#include "main.h"
#include <stdio.h>
#include "tuning_receive.h"
#include "math.h"
#include "math_utils.h"

extern float phi_error_P_gain;
extern float phi_dot_FF_gain;
extern float phi_dot_error_P_gain;
extern float phi_dot_error_I_gain;
extern float theta_error_P_gain;
extern float theta_dot_FF_gain;
extern float theta_dot_error_P_gain;
extern float theta_dot_error_I_gain;
extern float psi_dot_error_P_gain;

extern int servo1_offset;
extern int servo2_offset;
extern int servo3_offset;
extern int servo4_offset;

uint8_t tuning_rx_buf[100];
int tuning_payload_cnt = 0;
uint16_t tuning[20] = {0, };
uint16_t tuning_crc = 0;

void process_tuning_receive (uint8_t data) {
  if (tuning_payload_cnt == 0 && data == 0x20) {
    tuning_rx_buf[tuning_payload_cnt] = data;
    tuning_payload_cnt++;
  } else if (tuning_payload_cnt == 1 && data == 0x40) {
    tuning_rx_buf[tuning_payload_cnt] = data;
    tuning_payload_cnt++;
  } else if (tuning_payload_cnt >= 2 && tuning_payload_cnt < 2 + 2 * NUM_OF_TUNINGS) {
    tuning_rx_buf[tuning_payload_cnt] = data;
    tuning_payload_cnt++;
  } else if (tuning_payload_cnt == 2 + 2 * NUM_OF_TUNINGS) {
    tuning_crc &= 0xff00;
    tuning_crc |= data;
    tuning_payload_cnt++;
  } else if (tuning_payload_cnt == 2 + 2 * NUM_OF_TUNINGS + 1) {
    uint16_t check_sum = 0;
    for (int i = 0; i < (NUM_OF_TUNINGS + 1) * 2; i++) {
      check_sum += tuning_rx_buf[i];
    }
    tuning_crc &= 0x00ff;
    tuning_crc |= ((uint16_t)data << 8);
    if (0xffff - check_sum == tuning_crc) {
      for (int i = 0, j = 2; i < NUM_OF_TUNINGS; i++, j += 2) {
        tuning[i] = *(uint16_t *)(tuning_rx_buf + j) & 0xfff;
      }
      phi_error_P_gain = min_f(PHI_ERROR_P_GAIN_FS, (float)tuning[PHI_ERROR_P_IDX] / 2000.0 * PHI_ERROR_P_GAIN_FS);
      phi_dot_FF_gain = min_f(PHI_DOT_FF_GAIN_FS, (float)tuning[PHI_DOT_FF_IDX] / 2000.0 * PHI_DOT_FF_GAIN_FS);
      phi_dot_error_P_gain = min_f(PHI_DOT_ERROR_P_GAIN_FS, (float)tuning[PHI_DOT_ERROR_P_IDX] / 2000.0 * PHI_DOT_ERROR_P_GAIN_FS);
      phi_dot_error_I_gain = min_f(PHI_DOT_ERROR_I_GAIN_FS, (float)tuning[PHI_DOT_ERROR_I_IDX] / 2000.0 * PHI_DOT_ERROR_I_GAIN_FS);

      theta_error_P_gain = min_f(THETA_ERROR_P_GAIN_FS, (float)tuning[THETA_ERROR_P_IDX] / 2000.0 * THETA_ERROR_P_GAIN_FS);
      theta_dot_FF_gain = min_f(THETA_DOT_FF_GAIN_FS, (float)tuning[THETA_DOT_FF_IDX] / 2000.0 * THETA_DOT_FF_GAIN_FS);
      theta_dot_error_P_gain = min_f(THETA_DOT_ERROR_P_GAIN_FS, (float)tuning[THETA_DOT_ERROR_P_IDX] / 2000.0 * THETA_DOT_ERROR_P_GAIN_FS);
      theta_dot_error_I_gain = min_f(THETA_DOT_ERROR_I_GAIN_FS, (float)tuning[THETA_DOT_ERROR_I_IDX] / 2000.0 * THETA_DOT_ERROR_I_GAIN_FS);

      psi_dot_error_P_gain = min_f(PSI_DOT_ERROR_P_GAIN_FS, (float)tuning[PSI_DOT_ERROR_P_IDX] / 2000.0 * PSI_DOT_ERROR_P_GAIN_FS);

      servo1_offset = tuning[SERVO1_OFFSET];
      servo2_offset = tuning[SERVO2_OFFSET];
      servo3_offset = tuning[SERVO3_OFFSET];
      servo4_offset = tuning[SERVO4_OFFSET];
    }
    tuning_payload_cnt = 0;
  } else {
    tuning_payload_cnt = 0;
  }
}

