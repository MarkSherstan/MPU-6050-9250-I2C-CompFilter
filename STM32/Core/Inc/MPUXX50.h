/*
 * MPUXX50.h
 *
 *  Created on: Feb 27, 2022
 *      Author: MarkSherstan
 */

#ifndef MPUXX50_H_
#define MPUXX50_H_

// Libs
#include <stdint.h>
#include "I2C.h"

// IMU configuration
#define AD0_LOW 	 0x68 << 1
#define AD0_HIGH 	 0x69 << 1
#define WHO_AM_I 	 0x75
#define GYRO_CONFIG  0x1B
#define ACCEL_CONFIG 0x1C
#define PWR_MGMT_1   0x6B

// Accelerometer, temperature, and gyroscope data out registries
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40
#define TEMP_OUT_H 	 0x41
#define TEMP_OUT_L   0x42
#define GYRO_XOUT_H  0x43
#define GYRO_XOUT_L  0x44
#define GYRO_YOUT_H  0x45
#define GYRO_YOUT_L  0x46
#define GYRO_ZOUT_H  0x47
#define GYRO_ZOUT_L  0x48

// Full scale ranges -> enum?
#define AFS_2G 	0
#define AFS_4G 	1
#define AFS_8G 	2
#define AFS_16G 3

#define GFS_250DPS 	0
#define GFS_500DPS 	1
#define GFS_1000DPS 2
#define GFS_2000DPS 3

// Structures
// struct
// {
// 	float ax, ay, az, gx, gy, gz;
// };

// struct
// {
// 	float x, y, z;
// } gyroCal;

// struct
// {
// 	float r, p, y;
// } attitude;

// Functions
void init(uint8_t addr, uint8_t aScale, uint8_t gScale);

#endif /* MPUXX50_H_ */
