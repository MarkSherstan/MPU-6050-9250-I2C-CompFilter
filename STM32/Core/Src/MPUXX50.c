/*
 * MPUXX50.c
 *
 *  Created on: Feb 27, 2022
 *      Author: MarkSherstan
 */

#include "MPUXX50.h"

/// @brief Set the IMU address and full scale ranges.
/// @param addr Hex address based on AD0 pin - 0x68 low or 0x69 high.
/// @param aScale Set accelerometer full scale range: 0 for ±2g, 1 for ±4g, 2 for ±8g, and 3 for ±16g.
/// @param gScale Set gyroscope full scale range: 0 for ±250°/s, 1 for ±500°/s, 2 for ±1000°/s, and 3 for ±2000°/s.
void IMU_init(uint8_t addr, uint8_t aScale, uint8_t gScale)
{
	// Save values
	_addr = addr << 1;
	_aScale = aScale;
	_gScale = gScale;

	// Start up I2C
	MX_I2C1_Init();
}

/// @brief Check for connection, reset IMU, and set full range scale.
void IMU_begin(void)
{
	// Find who the IMU is
	buf[0] = WHO_AM_I;
    ret = HAL_I2C_Master_Transmit(&hi2c1, _addr, buf, 1, HAL_MAX_DELAY);

    if ( ret == HAL_OK ) {
    	ret = HAL_I2C_Master_Receive(&hi2c1, _addr, buf, 1, HAL_MAX_DELAY);
    	if ( ret == HAL_OK ) {
    	    if (buf[0] == WHO_AM_I_ANS)
    	    {
    	        write2bytes(PWR_MGMT_1, 0x00);
    	        setAccFullScaleRange(_aScale);
    	        setGyroFullScaleRange(_gScale);
    	    }
    	}
    }
}

/// @brief Set the accelerometer full scale range.
/// @param aScale Set 0 for ±2g, 1 for ±4g, 2 for ±8g, and 3 for ±16g.
void setAccFullScaleRange(uint8_t aScale)
{
    switch (aScale)
    {
    case AFS_2G:
        aRes = 16384.0;
        write2bytes(ACCEL_CONFIG, 0x00);
        break;
    case AFS_4G:
        aRes = 8192.0;
        write2bytes(ACCEL_CONFIG, 0x08);
        break;
    case AFS_8G:
        aRes = 4096.0;
        write2bytes(ACCEL_CONFIG, 0x10);
        break;
    case AFS_16G:
        aRes = 2048.0;
        write2bytes(ACCEL_CONFIG, 0x18);
        break;
    default:
        aRes = 8192.0;
        write2bytes(ACCEL_CONFIG, 0x08);
        break;
    }
}

/// @brief Set the gyroscope full scale range.
/// @param gScale Set 0 for ±250°/s, 1 for ±500°/s, 2 for ±1000°/s, and 3 for ±2000°/s.
void setGyroFullScaleRange(uint8_t gScale)
{
    switch (gScale)
    {
    case GFS_250DPS:
        gRes = 131.0;
        write2bytes(GYRO_CONFIG, 0x00);
        break;
    case GFS_500DPS:
        gRes = 65.5;
        write2bytes(GYRO_CONFIG, 0x08);
        break;
    case GFS_1000DPS:
        gRes = 32.8;
        write2bytes(GYRO_CONFIG, 0x10);
        break;
    case GFS_2000DPS:
        gRes = 16.4;
        write2bytes(GYRO_CONFIG, 0x18);
        break;
    default:
        gRes = 65.5;
        write2bytes(GYRO_CONFIG, 0x08);
        break;
    }
}

/// @brief Write bytes to specific registers on the IMU.
/// @param byte0 The main register to be written.
/// @param byte1 The command to be written.
bool write2bytes(uint8_t byte0, uint8_t byte1)
{
	buf[0] = byte0;
	buf[1] = byte1;

    ret = HAL_I2C_Master_Transmit(&hi2c1, _addr, buf, 2, HAL_MAX_DELAY);

    if ( ret != HAL_OK ) {
    	return false;
    } else {
    	return true;
    }
}

/// @brief Read raw data from IMU
void readRawData()
{
    // Subroutine for reading the raw data
	buf[0] = ACCEL_XOUT_H;
	ret = HAL_I2C_Master_Transmit(&hi2c1, _addr, buf, 1, HAL_MAX_DELAY);

    if ( ret == HAL_OK ) {
    	ret = HAL_I2C_Master_Receive(&hi2c1, _addr, buf, 14, HAL_MAX_DELAY);
    	if ( ret == HAL_OK ) {
    	    // Read raw data
    	    sensorRaw.ax = buf[0] << 8 | buf[1];
    	    sensorRaw.ay = buf[2] << 8 | buf[3];
    	    sensorRaw.az = buf[4] << 8 | buf[5];

    	    // temperature = buf[6] << 8 | buf[7];

    	    sensorRaw.gx = buf[8]  << 8 | buf[9];
    	    sensorRaw.gy = buf[10] << 8 | buf[11];
    	    sensorRaw.gz = buf[12] << 8 | buf[13];
    	}
    }
}

/// @brief Find offsets for each axis of gyroscope.
/// @param numCalPoints Number of data points to average.
void IMU_calibrateGyro(uint16_t numCalPoints)
{
    // Save specified number of points
    for (uint16_t ii = 0; ii < numCalPoints; ii++)
    {
        readRawData();
        gyroCal.x += sensorRaw.gx;
        gyroCal.y += sensorRaw.gy;
        gyroCal.z += sensorRaw.gz;
        HAL_Delay(3);
    }

    // Average the saved data points to find the gyroscope offset
    gyroCal.x /= (float)numCalPoints;
    gyroCal.y /= (float)numCalPoints;
    gyroCal.z /= (float)numCalPoints;
}

/// @brief Calculate the real world sensor values
void readProcessedData(void)
{
    // Get raw values from the IMU
    readRawData();

    // Convert accelerometer values to g's
    sensorProcessed.ax = sensorRaw.ax / aRes;
    sensorProcessed.ay = sensorRaw.ay / aRes;
    sensorProcessed.az = sensorRaw.az / aRes;

    // Compensate for gyro offset
    sensorProcessed.gx = sensorRaw.gx - gyroCal.x;
    sensorProcessed.gy = sensorRaw.gy - gyroCal.y;
    sensorProcessed.gz = sensorRaw.gz - gyroCal.z;

    // Convert gyro values to deg/s
    sensorProcessed.gx /= gRes;
    sensorProcessed.gy /= gRes;
    sensorProcessed.gz /= gRes;
}

///// @brief Calculate the attitude of the sensor in degrees using a complementary filter
///// @param tau Time constant relating to the weighting of gyroscope vs accelerometer.
//void calcAttitude(float tau)
//{
//    // Find dt
//    float dt = (micros() - timer) / 1e-6;
//    timer = micros();
//
//    // Read calibrated data
//    readProcessedData();
//
//    // Complementary filter
//    float accelPitch = atan2(sensorProcessed.ay, sensorProcessed.az) * (180 / M_PI);
//    float accelRoll = atan2(sensorProcessed.ax, sensorProcessed.az) * (180 / M_PI);
//
//    attitude.r = tau * (attitude.r - sensorProcessed.gy * dt) + (1 - tau) * accelRoll;
//    attitude.p = tau * (attitude.p + sensorProcessed.gx * dt) + (1 - tau) * accelPitch;
//    attitude.y += sensorProcessed.gz * dt;
//}
//
///// @brief Starts a timer to be used for the complementary filter
//void startTimer(void)
//{
//    timer = micros();
//}
