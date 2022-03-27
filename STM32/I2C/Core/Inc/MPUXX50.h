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
#include <math.h>
#include "I2C.h"

// Constants
#define RAD2DEG 57.2957795131

// IMU configuration and data out registry
#define AD0_LOW 	 	 	0x68
#define AD0_HIGH 	 	 	0x69
#define GYRO_CONFIG  		0x1B
#define ACCEL_CONFIG 		0x1C
#define PWR_MGMT_1   		0x6B
#define WHO_AM_I 	 	    0x75
#define WHO_AM_I_6050_ANS   0x68
#define WHO_AM_I_9250_ANS   0x71
#define I2C_TIMOUT_MS 		1000
#define ACCEL_XOUT_H        0x3B

// Full scale ranges
enum gyroscopeFullScaleRange{GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS};
enum accelerometerFullScaleRange{AFS_2G, AFS_4G, AFS_8G, AFS_16G};

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
uint8_t _addr;
float _dt, _tau;
float aRes, gRes;

// Functions
uint8_t MPU_begin(I2C_HandleTypeDef *I2Cx, uint8_t addr, uint8_t aScale, uint8_t gScale, float tau, float dt);
void MPU_calibrateGyro(I2C_HandleTypeDef *I2Cx, uint16_t numCalPoints);
void MPU_calcAttitude(I2C_HandleTypeDef *I2Cx);

void readRawData(I2C_HandleTypeDef *I2Cx);
void readProcessedData(I2C_HandleTypeDef *I2Cx);
void setGyroFullScaleRange(I2C_HandleTypeDef *I2Cx, uint8_t gScale);
void setAccFullScaleRange(I2C_HandleTypeDef *I2Cx, uint8_t aScale);

#endif /* MPUXX50_H_ */
