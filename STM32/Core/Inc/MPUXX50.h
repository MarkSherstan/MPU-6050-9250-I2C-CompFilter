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
#include <stdbool.h>
#include <math.h>
#include "I2C.h"

#define PI 3.141592654

#define dt 	0.004
#define tau	0.98

// IMU configuration
#define AD0_LOW 	 0x68
#define AD0_HIGH 	 0x69
#define GYRO_CONFIG  0x1B
#define ACCEL_CONFIG 0x1C
#define PWR_MGMT_1   0x6B

#define WHO_AM_I 	 				0x75
#define WHO_AM_I_6050_ANS 0x68
#define WHO_AM_I_9250_ANS 0x71

// #define SMPL_R8_DIV	0x19

#define I2C_TIMOUT_MS 1000

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
struct SensorRaw
{
	int16_t ax, ay, az, gx, gy, gz;
} sensorRaw;

struct SensorProcessed
{
	float ax, ay, az, gx, gy, gz;
} sensorProcessed;

struct GyroCal
{
	float x, y, z;
} gyroCal;

struct Attitude
{
	float r, p, y;
} attitude;

// Variables
HAL_StatusTypeDef ret;
uint8_t _addr, _aScale, _gScale;
float aRes, gRes;

// Functions
void IMU_init(uint8_t addr, uint8_t aScale, uint8_t gScale);
void IMU_begin(void);
void IMU_calibrateGyro(uint16_t numCalPoints);
void readRawData(void);
void setGyroFullScaleRange(uint8_t gScale);
void setAccFullScaleRange(uint8_t aScale);

void IMU_calcAttitude(void);
void readProcessedData(void);

#endif /* MPUXX50_H_ */
