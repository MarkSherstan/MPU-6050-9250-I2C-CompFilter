/*
 * MPUXX50.cpp
 *
 *  Created on: Apr 26, 2022
 *      Author: MarkSherstan
 */

#include "MPUXX50.h"

/// @brief MPUXX50 I2C constructor
/// @param pI2Cx Pointer to I2C structure config
MPUXX50::MPUXX50(I2C_HandleTypeDef *pI2Cx, uint8_t addr)
{
    _pI2Cx = pI2Cx;
    _addr = addr << 1;
}

/// @brief Boot up the IMU and ensure we have a valid connection
/// @return Success [1] or fail [0]
uint8_t MPUXX50::begin()
{
    // Initialize variables
    uint8_t check, select;

    // Set attitude to zero conditions
    attitude.r = 0;
    attitude.p = 0;
    attitude.y = 0;

    // Confirm device
    HAL_I2C_Mem_Read(_pI2Cx, _addr, WHO_AM_I, 1, &check, 1, I2C_TIMOUT_MS);

    // TODO: If 9250 or 6050 fails could it trigger the opposite check???
    if ((check == WHO_AM_I_9250_ANS) || (check == WHO_AM_I_6050_ANS))
    {
        // Startup / reset the sensor
        select = 0x00;
        HAL_I2C_Mem_Write(_pI2Cx, _addr, PWR_MGMT_1, 1, &select, 1, I2C_TIMOUT_MS);

        // Set the full scale ranges
        writeAccFullScaleRange(_aFSR);
        writeGyroFullScaleRange(_gFSR);

        return 1;
    }
    else
    {
        return 0;
    }
}

/// @brief Set the accelerometer full scale range.
/// @param aScale Set 0 for ±2g, 1 for ±4g, 2 for ±8g, and 3 for ±16g.
void MPUXX50::writeAccFullScaleRange(uint8_t aFSR)
{
    // Variable init
    uint8_t select;

    // Set the value
    switch (aFSR)
    {
    case AFSR_2G:
        aScaleFactor = 16384.0;
        select = 0x00;
        HAL_I2C_Mem_Write(_pI2Cx, _addr, ACCEL_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    case AFSR_4G:
        aScaleFactor = 8192.0;
        select = 0x08;
        HAL_I2C_Mem_Write(_pI2Cx, _addr, ACCEL_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    case AFSR_8G:
        aScaleFactor = 4096.0;
        select = 0x10;
        HAL_I2C_Mem_Write(_pI2Cx, _addr, ACCEL_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    case AFSR_16G:
        aScaleFactor = 2048.0;
        select = 0x18;
        HAL_I2C_Mem_Write(_pI2Cx, _addr, ACCEL_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    default:
        aScaleFactor = 8192.0;
        select = 0x08;
        HAL_I2C_Mem_Write(_pI2Cx, _addr, ACCEL_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    }
}

/// @brief Set the gyroscope full scale range.
/// @param gScale Set 0 for ±250°/s, 1 for ±500°/s, 2 for ±1000°/s, and 3 for ±2000°/s.
void MPUXX50::writeGyroFullScaleRange(uint8_t gFSR)
{
    // Variable init
    uint8_t select;

    // Set the value
    switch (gFSR)
    {
    case GFSR_250DPS:
        gScaleFactor = 131.0;
        select = 0x00;
        HAL_I2C_Mem_Write(_pI2Cx, _addr, GYRO_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    case GFSR_500DPS:
        gScaleFactor = 65.5;
        select = 0x08;
        HAL_I2C_Mem_Write(_pI2Cx, _addr, GYRO_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    case GFSR_1000DPS:
        gScaleFactor = 32.8;
        select = 0x10;
        HAL_I2C_Mem_Write(_pI2Cx, _addr, GYRO_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    case GFSR_2000DPS:
        gScaleFactor = 16.4;
        select = 0x18;
        HAL_I2C_Mem_Write(_pI2Cx, _addr, GYRO_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    default:
        gScaleFactor = 65.5;
        select = 0x08;
        HAL_I2C_Mem_Write(_pI2Cx, _addr, GYRO_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    }
}

/// @brief Set the gyroscope full scale range
/// @param gFSR Desired yroscope full scale range
void MPUXX50::setGyroFullScaleRange(uint8_t gFSR)
{
    _gFSR = gFSR;
}

/// @brief Set the accelerometer full scale range
/// @param aFSR Desired accelerometer full scale range
void MPUXX50::setAccFullScaleRange(uint8_t aFSR)
{
    _aFSR = aFSR;
}

/// @brief Set the sampling duration (delta time) in seconds
/// @param dt Sampling time delta in seconds
void MPUXX50::setDeltaTime(float dt)
{
    _dt = dt;
}

/// @brief Time constant of the complementary filter
/// @param tau Time constant
void MPUXX50::setTau(float tau)
{
    _tau = tau;
}

/// @brief Find offsets for each axis of gyroscope
void MPUXX50::calibrateGyro(uint16_t numCalPoints)
{
    // Init
    RawData rawData;
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
        rawData = readRawData();
        x += rawData.gx;
        y += rawData.gy;
        z += rawData.gz;
        HAL_Delay(3);
    }

    // Average the saved data points to find the gyroscope offset
    gyroCal.x = (float)x / (float)numCalPoints;
    gyroCal.y = (float)y / (float)numCalPoints;
    gyroCal.z = (float)z / (float)numCalPoints;
}

/// @brief Read raw data from IMU
/// @return Structure containing raw accelerometer and gyroscope data
RawData MPUXX50::readRawData()
{
    // Data out and buffer init
    RawData rawData;
    uint8_t buf[14];

    // Subroutine for reading the raw data
    HAL_I2C_Mem_Read(_pI2Cx, _addr, ACCEL_XOUT_H, 1, buf, 14, I2C_TIMOUT_MS);

    // Bit shift the data
    rawData.ax = buf[0] << 8 | buf[1];
    rawData.ay = buf[2] << 8 | buf[3];
    rawData.az = buf[4] << 8 | buf[5];
    // temperature = buf[6] << 8 | buf[7];
    rawData.gx = buf[8] << 8 | buf[9];
    rawData.gy = buf[10] << 8 | buf[11];
    rawData.gz = buf[12] << 8 | buf[13];

    // Result
    return rawData;
}

/// @brief Process the raw data into real world sensor values
/// @return Structure containing processed accelerometer and gyroscope data
ProcessedData MPUXX50::processData()
{
    // Data out structure
    ProcessedData processedData;

    // Get raw values from the IMU
    RawData rawData = readRawData();

    // Convert accelerometer values to g's
    processedData.ax = rawData.ax / aScaleFactor;
    processedData.ay = rawData.ay / aScaleFactor;
    processedData.az = rawData.az / aScaleFactor;

    // Compensate for gyro offset
    processedData.gx = rawData.gx - gyroCal.x;
    processedData.gy = rawData.gy - gyroCal.y;
    processedData.gz = rawData.gz - gyroCal.z;

    // Convert gyro values to deg/s
    processedData.gx /= gScaleFactor;
    processedData.gy /= gScaleFactor;
    processedData.gz /= gScaleFactor;

    // Return structure
    return processedData;
}

/// @brief Calculate the attitude of the sensor in degrees using a complementary filter
/// @return Structure containing sensor attitude data
Attitude MPUXX50::calcAttitude()
{
    // Read processed data
    ProcessedData sensorData = processData();

    // Complementary filter
    float accelPitch = atan2(sensorData.ay, sensorData.az) * RAD2DEG;
    float accelRoll = atan2(sensorData.ax, sensorData.az) * RAD2DEG;

    attitude.r = _tau * (attitude.r - sensorData.gy * _dt) + (1 - _tau) * accelRoll;
    attitude.p = _tau * (attitude.p - sensorData.gx * _dt) + (1 - _tau) * accelPitch;
    attitude.y += (sensorData.gz * _dt);

    // Return
    return attitude;
}
