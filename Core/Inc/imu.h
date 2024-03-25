/*
 * sensor.h
 *
 *  Created on: Mar 5, 2024
 *      Author: akswnd98
 */

#include "main.h"

#ifndef INC_IMU_H_
#define INC_IMU_H_

#define WHO_AM_I 0x75
#define REG_BANK_SEL 0x76
#define ACCEL_DATA_X1 0x1F
#define ACCEL_DATA_X0 0x20
#define ACCEL_DATA_Y1 0x21
#define ACCEL_DATA_Y0 0x22
#define ACCEL_DATA_Z1 0x23
#define ACCEL_DATA_Z0 0x24
#define GYRO_DATA_X1 0x25
#define GYRO_DATA_X0 0x26
#define GYRO_DATA_Y1 0x27
#define GYRO_DATA_Y0 0x28
#define GYRO_DATA_Z1 0x29
#define GYRO_DATA_Z0 0x2A

#define PWR_MGMT0 0x4E
#define CALC_PWR_MGMT0_BITS(ACCEL_MODE, GYRO_MODE) ((ACCEL_MODE) | ((GYRO_MODE) << 2))

#define GYRO_CONFIG0 0x4F
#define CALC_GYRO_CONFIG0_BITS(GYRO_ODR, GYRO_FS_SEL) ((GYRO_ODR) | ((GYRO_FS_SEL) << 5))

#define ACCEL_CONFIG0 0x50
#define CALC_ACCEL_CONFIG0_BITS(ACCEL_ODR, ACCEL_FS_SEL) ((ACCEL_ODR) | ((ACCEL_FS_SEL) << 5))

#define GYRO_CONFIG_STATIC2 0x0B
#define CALC_GYRO_CONFIG_STATIC2_BITS(GYRO_NF_DIS, GYRO_AAF_DIS) ((GYRO_NF_DIS) | ((GYRO_AAF_DIS) << 1))

#define GYRO_CONFIG_STATIC3 0x0C
#define CALC_GYRO_CONFIG_STATIC3_BITS(GYRO_AAF_DELT) (GYRO_AAF_DELT)

#define GYRO_CONFIG_STATIC4 0x0D
#define CALC_GYRO_CONFIG_STATIC4_BITS(GYRO_AAF_DELTSQR) ((GYRO_AAF_DELTSQR) & 0xff)

#define GYRO_CONFIG_STATIC5 0x0E
#define CALC_GYRO_CONFIG_STATIC5_BITS(GYRO_AAF_DELTSQR, GYRO_AAF_BITSHIFT) (((GYRO_AAF_DELTSQR) >> 8) | ((GYRO_AAF_BITSHIFT) << 4))

#define ACCEL_CONFIG_STATIC2 0x03
#define CALC_ACCEL_CONFIG_STATIC2_BITS(ACCEL_AAF_DIS, ACCEL_AAF_DELT) (((ACCEL_AAF_DELT) << 1) | (ACCEL_AAF_DIS))

#define ACCEL_CONFIG_STATIC3 0x04
#define CALC_ACCEL_CONFIG_STATIC3_BITS(ACCEL_AAF_DELTSQR) ((ACCEL_AAF_DELTSQR) & 0xff)

#define ACCEL_CONFIG_STATIC4 0x05
#define CALC_ACCEL_CONFIG_STATIC4_BITS(ACCEL_AAF_DELTSQR, ACCEL_AAF_BITSHIFT) (((ACCEL_AAF_DELTSQR) >> 8) | ((ACCEL_AAF_BITSHIFT) << 4))

void read_imu (uint8_t addr, uint8_t *data, uint8_t len);
void write_imu (uint8_t addr, uint8_t *data);
void init_imu ();

void update_imu_value ();
void adjust_imu_unit ();
void adjust_acc_unit ();
void adjust_gyro_unit ();

#endif /* INC_IMU_H_ */
