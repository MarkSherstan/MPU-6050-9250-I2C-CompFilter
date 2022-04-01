/*
 * MPUXX50.c
 *
 *  Created on: Feb 27, 2022
 *      Author: MarkSherstan
 */

#include "MPUXX50.h"

/// @brief Set the IMU address, check for connection, reset IMU, and set full range scale.
/// @param I2Cx Pointer to I2C structure config.
/// @param addr Hex address based on AD0 pin - 0x68 low or 0x69 high.
/// @param aScale Set accelerometer full scale range: 0 for ±2g, 1 for ±4g, 2 for ±8g, and 3 for ±16g.
/// @param gScale Set gyroscope full scale range: 0 for ±250°/s, 1 for ±500°/s, 2 for ±1000°/s, and 3 for ±2000°/s.
/// @param tau Set tau value for the complementary filter (typically 0.98).
/// @param dt Set sampling rate in seconds determined by the timer interrupt.
uint8_t MPU_begin(I2C_HandleTypeDef *I2Cx, uint8_t addr, uint8_t aScale, uint8_t gScale, float tau, float dt)
{
    // Save values
    _addr = addr << 1;
    _tau = tau;
    _dt = dt;

    // Initialize variables
    uint8_t check;
    uint8_t select;

    // Confirm device
    HAL_I2C_Mem_Read(I2Cx, _addr, WHO_AM_I, 1, &check, 1, I2C_TIMOUT_MS);

    // TODO: If 9250 or 6050 fails could it trigger the opposite check???
    if ((check == WHO_AM_I_9250_ANS) || (check == WHO_AM_I_6050_ANS))
    {
        // Startup / reset the sensor
        select = 0x00;
        HAL_I2C_Mem_Write(I2Cx, _addr, PWR_MGMT_1, 1, &select, 1, I2C_TIMOUT_MS);

        // Set the full scale ranges
        setAccFullScaleRange(I2Cx, aScale);
        setGyroFullScaleRange(I2Cx, gScale);

        return 1;
    }
    else
    {
        return 0;
    }
}

/// @brief Set the accelerometer full scale range.
/// @param I2Cx Pointer to I2C structure config.
/// @param aScale Set 0 for ±2g, 1 for ±4g, 2 for ±8g, and 3 for ±16g.
void setAccFullScaleRange(I2C_HandleTypeDef *I2Cx, uint8_t aScale)
{
    // Variable init
    uint8_t select;

    // Set the value
    switch (aScale)
    {
    case AFS_2G:
        aRes = 16384.0;
        select = 0x00;
        HAL_I2C_Mem_Write(I2Cx, _addr, ACCEL_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    case AFS_4G:
        aRes = 8192.0;
        select = 0x08;
        HAL_I2C_Mem_Write(I2Cx, _addr, ACCEL_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    case AFS_8G:
        aRes = 4096.0;
        select = 0x10;
        HAL_I2C_Mem_Write(I2Cx, _addr, ACCEL_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    case AFS_16G:
        aRes = 2048.0;
        select = 0x18;
        HAL_I2C_Mem_Write(I2Cx, _addr, ACCEL_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    default:
        aRes = 8192.0;
        select = 0x08;
        HAL_I2C_Mem_Write(I2Cx, _addr, ACCEL_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    }
}

/// @brief Set the gyroscope full scale range.
/// @param I2Cx Pointer to I2C structure config.
/// @param gScale Set 0 for ±250°/s, 1 for ±500°/s, 2 for ±1000°/s, and 3 for ±2000°/s.
void setGyroFullScaleRange(I2C_HandleTypeDef *I2Cx, uint8_t gScale)
{
    // Variable init
    uint8_t select;

    // Set the value
    switch (gScale)
    {
    case GFS_250DPS:
        gRes = 131.0;
        select = 0x00;
        HAL_I2C_Mem_Write(I2Cx, _addr, GYRO_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    case GFS_500DPS:
        gRes = 65.5;
        select = 0x08;
        HAL_I2C_Mem_Write(I2Cx, _addr, GYRO_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    case GFS_1000DPS:
        gRes = 32.8;
        select = 0x10;
        HAL_I2C_Mem_Write(I2Cx, _addr, GYRO_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    case GFS_2000DPS:
        gRes = 16.4;
        select = 0x18;
        HAL_I2C_Mem_Write(I2Cx, _addr, GYRO_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    default:
        gRes = 65.5;
        select = 0x08;
        HAL_I2C_Mem_Write(I2Cx, _addr, GYRO_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    }
}

/// @brief Read raw data from IMU.
/// @param I2Cx Pointer to I2C structure config.
void readRawData(I2C_HandleTypeDef *I2Cx)
{
    uint8_t buf[14];

    // Subroutine for reading the raw data
    HAL_I2C_Mem_Read(I2Cx, _addr, ACCEL_XOUT_H, 1, buf, 14, I2C_TIMOUT_MS);

    // Bit shift the data
    sensorRaw.ax = buf[0] << 8 | buf[1];
    sensorRaw.ay = buf[2] << 8 | buf[3];
    sensorRaw.az = buf[4] << 8 | buf[5];

    // temperature = buf[6] << 8 | buf[7];

    sensorRaw.gx = buf[8] << 8 | buf[9];
    sensorRaw.gy = buf[10] << 8 | buf[11];
    sensorRaw.gz = buf[12] << 8 | buf[13];
}

/// @brief Find offsets for each axis of gyroscope.
/// @param I2Cx Pointer to I2C structure config.
/// @param numCalPoints Number of data points to average.
void MPU_calibrateGyro(I2C_HandleTypeDef *I2Cx, uint16_t numCalPoints)
{
    // Init
    int32_t x = 0;
    int32_t y = 0;
    int32_t z = 0;

    // Zero guard
    if (numCalPoints == 0)
    {
        numCalPoints = 1;
    }

    // Save specified number of points
    for (uint16_t ii = 0; ii < numCalPoints; ii++)
    {
        readRawData(I2Cx);
        x += sensorRaw.gx;
        y += sensorRaw.gy;
        z += sensorRaw.gz;
        HAL_Delay(3);
    }

    // Average the saved data points to find the gyroscope offset
    gyroCal.x = (float)x / (float)numCalPoints;
    gyroCal.y = (float)y / (float)numCalPoints;
    gyroCal.z = (float)z / (float)numCalPoints;
}

/// @brief Calculate the real world sensor values.
/// @param I2Cx Pointer to I2C structure config.
void readProcessedData(I2C_HandleTypeDef *I2Cx)
{
    // Get raw values from the IMU
    readRawData(I2Cx);

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

/// @brief Calculate the attitude of the sensor in degrees using a complementary filter.
/// @param I2Cx Pointer to I2C structure config.
void MPU_calcAttitude(I2C_HandleTypeDef *I2Cx)
{
    // Read processed data
    readProcessedData(I2Cx);

    // Complementary filter
    float accelPitch = atan2(sensorProcessed.ay, sensorProcessed.az) * RAD2DEG;
    float accelRoll = atan2(sensorProcessed.ax, sensorProcessed.az) * RAD2DEG;

    attitude.r = _tau * (attitude.r - sensorProcessed.gy * _dt) + (1 - _tau) * accelRoll;
    attitude.p = _tau * (attitude.p + sensorProcessed.gx * _dt) + (1 - _tau) * accelPitch;
    attitude.y += sensorProcessed.gz * _dt;
}
