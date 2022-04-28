/*
 * MPU9250.c
 *
 *  Created on: Mar 13, 2022
 *      Author: MarkSherstan
 */

#include "MPU9250.h"

/// @brief Check for connection, reset IMU, and set full range scale
/// @param SPIx Pointer to SPI structure config
/// @param pMPU9250 Pointer to master MPU9250 struct
uint8_t MPU_begin(SPI_HandleTypeDef *SPIx, MPU9250_t *pMPU9250)
{
    // Initialize variables
    uint8_t check, addr, val;

    // Confirm device
    MPU_REG_READ(SPIx, pMPU9250, WHO_AM_I, &check, 1);
    if (check == WHO_AM_I_9250_ANS)
    {
        // Startup / reset the sensor
        addr = PWR_MGMT_1;
        val = 0x00;
        MPU_REG_WRITE(SPIx, pMPU9250, &addr, &val);

        // Disable I2C (SPI only)
        addr = USER_CTRL;
        val = 0x10;
        MPU_REG_WRITE(SPIx, pMPU9250, &addr, &val);

        // Set the full scale ranges
        MPU_writeAccFullScaleRange(SPIx, pMPU9250, pMPU9250->settings.aFullScaleRange);
        MPU_writeGyroFullScaleRange(SPIx, pMPU9250, pMPU9250->settings.gFullScaleRange);
        return 1;
    }
    else
    {
        return 0;
    }
}

/// @brief Read a specific registry address
/// @param SPIx Pointer to SPI structure config
/// @param pMPU9250 Pointer to master MPU9250 struct
/// @param pAddr Pointer to address to be written to
/// @param pVal Pointer of value to write to given address
void MPU_REG_WRITE(SPI_HandleTypeDef *SPIx, MPU9250_t *pMPU9250, uint8_t *pAddr, uint8_t *pVal)
{
    MPU_CS(pMPU9250, CS_SELECT);
    HAL_SPI_Transmit(SPIx, pAddr, 1, SPI_TIMOUT_MS);
    HAL_SPI_Transmit(SPIx, pVal, 1, SPI_TIMOUT_MS);
    MPU_CS(pMPU9250, CS_DESELECT);
}

/// @brief Read a specific registry address
/// @param SPIx Pointer to SPI structure config
/// @param pMPU9250 Pointer to master MPU9250 struct
/// @param addr Address to start reading at
/// @param pRxData Pointer to data buffer
/// @param RxSize Size of data buffer
void MPU_REG_READ(SPI_HandleTypeDef *SPIx, MPU9250_t *pMPU9250, uint8_t addr, uint8_t *pRxData, uint16_t RxSize)
{
    MPU_CS(pMPU9250, CS_SELECT);
    uint8_t writeAddr = addr | READWRITE;
    HAL_SPI_Transmit(SPIx, &writeAddr, 1, SPI_TIMOUT_MS);
    HAL_SPI_Receive(SPIx, pRxData, RxSize, SPI_TIMOUT_MS);
    MPU_CS(pMPU9250, CS_DESELECT);
}

/// @brief Set CS state to either start or end transmissions
/// @param pMPU9250 Pointer to master MPU9250 struct
/// @param state Set low to select, high to deselect
void MPU_CS(MPU9250_t *pMPU9250, uint8_t state)
{
    HAL_GPIO_WritePin(pMPU9250->settings.CS_PORT, pMPU9250->settings.CS_PIN, state);
}

/// @brief Set the accelerometer full scale range
/// @param SPIx Pointer to SPI structure config
/// @param pMPU9250 Pointer to master MPU9250 struct
/// @param aScale Set 0 for ±2g, 1 for ±4g, 2 for ±8g, and 3 for ±16g
void MPU_writeAccFullScaleRange(SPI_HandleTypeDef *SPIx, MPU9250_t *pMPU9250, uint8_t aScale)
{
    // Variable init
    uint8_t addr = ACCEL_CONFIG;
    uint8_t val;

    // Set the value
    switch (aScale)
    {
    case AFSR_2G:
        pMPU9250->sensorData.aScaleFactor = 16384.0;
        val = 0x00;
        MPU_REG_WRITE(SPIx, pMPU9250, &addr, &val);
        break;
    case AFSR_4G:
        pMPU9250->sensorData.aScaleFactor = 8192.0;
        val = 0x08;
        MPU_REG_WRITE(SPIx, pMPU9250, &addr, &val);
        break;
    case AFSR_8G:
        pMPU9250->sensorData.aScaleFactor = 4096.0;
        val = 0x10;
        MPU_REG_WRITE(SPIx, pMPU9250, &addr, &val);
        break;
    case AFSR_16G:
        pMPU9250->sensorData.aScaleFactor = 2048.0;
        val = 0x18;
        MPU_REG_WRITE(SPIx, pMPU9250, &addr, &val);
        break;
    default:
        pMPU9250->sensorData.aScaleFactor = 8192.0;
        val = 0x08;
        MPU_REG_WRITE(SPIx, pMPU9250, &addr, &val);
        break;
    }
}

/// @brief Set the gyroscope full scale range
/// @param SPIx Pointer to SPI structure config
/// @param pMPU9250 Pointer to master MPU9250 struct
/// @param gScale Set 0 for ±250°/s, 1 for ±500°/s, 2 for ±1000°/s, and 3 for ±2000°/s
void MPU_writeGyroFullScaleRange(SPI_HandleTypeDef *SPIx, MPU9250_t *pMPU9250, uint8_t gScale)
{
    // Variable init
    uint8_t addr = GYRO_CONFIG;
    uint8_t val;

    // Set the value
    switch (gScale)
    {
    case GFSR_250DPS:
        pMPU9250->sensorData.gScaleFactor = 131.0;
        val = 0x00;
        MPU_REG_WRITE(SPIx, pMPU9250, &addr, &val);
        break;
    case GFSR_500DPS:
        pMPU9250->sensorData.gScaleFactor = 65.5;
        val = 0x08;
        MPU_REG_WRITE(SPIx, pMPU9250, &addr, &val);
        break;
    case GFSR_1000DPS:
        pMPU9250->sensorData.gScaleFactor = 32.8;
        val = 0x10;
        MPU_REG_WRITE(SPIx, pMPU9250, &addr, &val);
        break;
    case GFSR_2000DPS:
        pMPU9250->sensorData.gScaleFactor = 16.4;
        val = 0x18;
        MPU_REG_WRITE(SPIx, pMPU9250, &addr, &val);
        break;
    default:
        pMPU9250->sensorData.gScaleFactor = 65.5;
        val = 0x08;
        MPU_REG_WRITE(SPIx, pMPU9250, &addr, &val);
        break;
    }
}

/// @brief Read raw data from IMU
/// @param SPIx Pointer to SPI structure config
/// @param pMPU9250 Pointer to master MPU9250 struct
void MPU_readRawData(SPI_HandleTypeDef *SPIx, MPU9250_t *pMPU9250)
{
    // Init buffer
    uint8_t buf[14];

    // Subroutine for reading the raw data
    MPU_REG_READ(SPIx, pMPU9250, ACCEL_XOUT_H, &buf[0], 14);

    // Bit shift the data
    pMPU9250->rawData.ax = buf[0] << 8 | buf[1];
    pMPU9250->rawData.ay = buf[2] << 8 | buf[3];
    pMPU9250->rawData.az = buf[4] << 8 | buf[5];
    // temperature = buf[6] << 8 | buf[7];
    pMPU9250->rawData.gx = buf[8] << 8 | buf[9];
    pMPU9250->rawData.gy = buf[10] << 8 | buf[11];
    pMPU9250->rawData.gz = buf[12] << 8 | buf[13];
}

/// @brief Find offsets for each axis of gyroscope
/// @param SPIx Pointer to SPI structure config
/// @param pMPU9250 Pointer to master MPU9250 struct
/// @param numCalPoints Number of data points to average
void MPU_calibrateGyro(SPI_HandleTypeDef *SPIx, MPU9250_t *pMPU9250, uint16_t numCalPoints)
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
        MPU_readRawData(SPIx, pMPU9250);
        x += pMPU9250->rawData.gx;
        y += pMPU9250->rawData.gy;
        z += pMPU9250->rawData.gz;
        HAL_Delay(3);
    }

    // Average the saved data points to find the gyroscope offset
    pMPU9250->gyroCal.x = (float)x / (float)numCalPoints;
    pMPU9250->gyroCal.y = (float)y / (float)numCalPoints;
    pMPU9250->gyroCal.z = (float)z / (float)numCalPoints;
}

/// @brief Calculate the real world sensor values
/// @param SPIx Pointer to SPI structure config
/// @param pMPU9250 Pointer to master MPU9250 struct
void MPU_readProcessedData(SPI_HandleTypeDef *SPIx, MPU9250_t *pMPU9250)
{
    // Get raw values from the IMU
    MPU_readRawData(SPIx, pMPU9250);

    // Convert accelerometer values to g's
    pMPU9250->sensorData.ax = pMPU9250->rawData.ax / pMPU9250->sensorData.aScaleFactor;
    pMPU9250->sensorData.ay = pMPU9250->rawData.ay / pMPU9250->sensorData.aScaleFactor;
    pMPU9250->sensorData.az = pMPU9250->rawData.az / pMPU9250->sensorData.aScaleFactor;

    // Compensate for gyro offset
    pMPU9250->sensorData.gx = pMPU9250->rawData.gx - pMPU9250->gyroCal.x;
    pMPU9250->sensorData.gy = pMPU9250->rawData.gy - pMPU9250->gyroCal.y;
    pMPU9250->sensorData.gz = pMPU9250->rawData.gz - pMPU9250->gyroCal.z;

    // Convert gyro values to deg/s
    pMPU9250->sensorData.gx /= pMPU9250->sensorData.gScaleFactor;
    pMPU9250->sensorData.gy /= pMPU9250->sensorData.gScaleFactor;
    pMPU9250->sensorData.gz /= pMPU9250->sensorData.gScaleFactor;
}

/// @brief Calculate the attitude of the sensor in degrees using a complementary filter
/// @param SPIx Pointer to SPI structure config
/// @param pMPU9250 Pointer to master MPU9250 struct
void MPU_calcAttitude(SPI_HandleTypeDef *SPIx, MPU9250_t *pMPU9250)
{
    // Read processed data
    MPU_readProcessedData(SPIx, pMPU9250);

    // Complementary filter
    float accelPitch = atan2(pMPU9250->sensorData.ay, pMPU9250->sensorData.az) * RAD2DEG;
    float accelRoll = atan2(pMPU9250->sensorData.ax, pMPU9250->sensorData.az) * RAD2DEG;

    pMPU9250->attitude.r = pMPU9250->attitude.tau * (pMPU9250->attitude.r - pMPU9250->sensorData.gy * pMPU9250->attitude.dt) + (1 - pMPU9250->attitude.tau) * accelRoll;
    pMPU9250->attitude.p = pMPU9250->attitude.tau * (pMPU9250->attitude.p - pMPU9250->sensorData.gx * pMPU9250->attitude.dt) + (1 - pMPU9250->attitude.tau) * accelPitch;
    pMPU9250->attitude.y += (pMPU9250->sensorData.gz * pMPU9250->attitude.dt);
}
