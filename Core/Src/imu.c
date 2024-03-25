/*
 * sensor.c
 *
 *  Created on: Mar 5, 2024
 *      Author: akswnd98
 */

#include <math_utils.h>
#include "imu.h"
#include "main.h"
#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "uart_utils.h"
#include "math.h"

#define _USE_MATH_DEFINES

#define ACCEL_FS 16.0
#define GYRO_FS 2000.0

extern SPI_HandleTypeDef hspi1;

float g = 9.80665;

uint8_t imu_buf[12];
int16_t raw_acc_x = 0;
int16_t raw_acc_y = 0;
int16_t raw_acc_z = 0;
int16_t raw_gyro_x = 0;
int16_t raw_gyro_y = 0;
int16_t raw_gyro_z = 0;
float acc_y = 0;
float acc_x = 0;
float acc_z = 0;
float gyro_x = 0;
float gyro_y = 0;
float gyro_z = 0;

uint8_t gyro_offset[6];
int16_t gyro_x_offset = 0;
int16_t gyro_y_offset = 0;
int16_t gyro_z_offset = 0;

uint8_t acc_offset[6];
int16_t acc_x_offset = 0;
int16_t acc_y_offset = 0;
int16_t acc_z_offset = 0;

void read_imu (uint8_t addr, uint8_t *data, uint8_t len) {
  uint8_t tmp = 0x80 | addr;
  while(HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, &tmp, 1, 100);
  HAL_SPI_Receive(&hspi1, data, len, 100);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

void write_imu (uint8_t addr, uint8_t *data) {
  while(HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, &addr, 1, 100);
  HAL_SPI_Transmit(&hspi1, data, 1, 100);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

void init_imu () {
  int delay_time = 100;
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
  HAL_Delay(delay_time);

  uint8_t tx_val = 0x00;
  uint8_t rx_val;

  HAL_Delay(delay_time);

  read_imu(WHO_AM_I, &rx_val, 1);
  HAL_Delay(delay_time);

  tx_val = CALC_PWR_MGMT0_BITS(3, 3);
  write_imu(PWR_MGMT0, &tx_val);
  HAL_Delay(delay_time);

  tx_val = CALC_GYRO_CONFIG0_BITS(0b0100, 0b000);
  write_imu(GYRO_CONFIG0, &tx_val);
  HAL_Delay(delay_time);

  tx_val = CALC_ACCEL_CONFIG0_BITS(0b0100, 0b000);
  write_imu(ACCEL_CONFIG0, &tx_val);
  HAL_Delay(delay_time);

  tx_val = 0x01;
  write_imu(REG_BANK_SEL, &tx_val);
  HAL_Delay(delay_time);

  tx_val = CALC_GYRO_CONFIG_STATIC2_BITS(1, 0);
  write_imu(GYRO_CONFIG_STATIC2, &tx_val);
  HAL_Delay(delay_time);

  tx_val = CALC_GYRO_CONFIG_STATIC3_BITS(30);
  write_imu(GYRO_CONFIG_STATIC3, &tx_val);
  HAL_Delay(delay_time);

  tx_val = CALC_GYRO_CONFIG_STATIC4_BITS(896);
  write_imu(GYRO_CONFIG_STATIC4, &tx_val);
  HAL_Delay(delay_time);

  tx_val = CALC_GYRO_CONFIG_STATIC5_BITS(896, 5);
  write_imu(GYRO_CONFIG_STATIC5, &tx_val);
  HAL_Delay(delay_time);

  tx_val = 0x02;
  write_imu(REG_BANK_SEL, &tx_val);
  HAL_Delay(delay_time);

  tx_val = CALC_ACCEL_CONFIG_STATIC2_BITS(0, 30);
  write_imu(ACCEL_CONFIG_STATIC2, &tx_val);
  HAL_Delay(delay_time);

  tx_val = CALC_ACCEL_CONFIG_STATIC3_BITS(896);
  write_imu(ACCEL_CONFIG_STATIC3, &tx_val);
  HAL_Delay(delay_time);

  tx_val = CALC_ACCEL_CONFIG_STATIC4_BITS(896, 5);
  write_imu(ACCEL_CONFIG_STATIC4, &tx_val);
  HAL_Delay(delay_time);

  tx_val = 0x00;
  write_imu(REG_BANK_SEL, &tx_val);
  HAL_Delay(delay_time);
}

void update_imu_value () {
  read_imu(ACCEL_DATA_X1, imu_buf, 12);

  raw_acc_x = (int16_t)(((uint16_t)imu_buf[0] << 8) | (uint16_t)imu_buf[1]);
  // acc_x = acc_x_sensor / 32767.0 * 4.0 * g;
  raw_acc_y = (int16_t)(((uint16_t)imu_buf[2] << 8) | (uint16_t)imu_buf[3]);
  raw_acc_z = (int16_t)(((uint16_t)imu_buf[4] << 8) | (uint16_t)imu_buf[5]);

  raw_gyro_x = (int16_t)(((uint16_t)imu_buf[6] << 8) | (uint16_t)imu_buf[7]);
  // gyro_x_sensor = gyro_x_sensor / 32767.0 * 1000.0 * M_PI / 180.0;
  raw_gyro_y = (int16_t)(((uint16_t)imu_buf[8] << 8) | (uint16_t)imu_buf[9]);
  raw_gyro_z = (int16_t)(((uint16_t)imu_buf[10] << 8) | (uint16_t)imu_buf[11]);

	acc_x = (float)raw_acc_x;
	acc_y = (float)raw_acc_y;
	acc_z = (float)raw_acc_z;

	gyro_x = (float)raw_gyro_x;
	gyro_y = (float)raw_gyro_y;
	gyro_z = (float)raw_gyro_z;

  adjust_imu_unit();
}

void adjust_imu_unit () {
	adjust_acc_unit();
	adjust_gyro_unit();
}

void adjust_acc_unit () {
	acc_x = acc_x / 32767.0 * ACCEL_FS * g;
	acc_y = acc_y / 32767.0 * ACCEL_FS * g;
	acc_z = acc_z / 32767.0 * ACCEL_FS * g;
}

void adjust_gyro_unit () {
	gyro_x = gyro_x / 32767.0 * GYRO_FS * M_PI / 180;
	gyro_y = gyro_y / 32767.0 * GYRO_FS * M_PI / 180;
	gyro_z = gyro_z / 32767.0 * GYRO_FS * M_PI / 180;
}
