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
    MPU_REG_READ(SPIx, WHO_AM_I, &check, 1);
    if (check == WHO_AM_I_9250_ANS)
    {
        // Startup / reset the sensor
        addr = PWR_MGMT_1;
        val = 0x00;
        MPU_REG_WRITE(SPIx, &addr, &val);

        // Disable I2C (SPI only)
        addr = USER_CTRL;
        val = 0x10;
        MPU_REG_WRITE(SPIx, &addr, &val);
        
        // Set the full scale ranges
        setAccFullScaleRange(SPIx, pMPU9250, pMPU9250->settings.aFullScaleRange);
        setGyroFullScaleRange(SPIx, pMPU9250, pMPU9250->settings.gFullScaleRange);
        return 1;
    }
    else 
    {
        return 0;
    }
}

/// @brief Read a specific registry address
/// @param pAddr Pointer to address to be written to
/// @param pVal Pointer of value to write to given address
void MPU_REG_WRITE(SPI_HandleTypeDef *SPIx, uint8_t *pAddr, uint8_t *pVal)
{
	MPU_CS(CS_SELECT);
    HAL_SPI_Transmit(SPIx, pAddr, 1, SPI_TIMOUT_MS);
    HAL_SPI_Transmit(SPIx, pVal, 1, SPI_TIMOUT_MS);
	MPU_CS(CS_DESELECT);
}

/// @brief Read a specific registry address
/// @param SPIx Pointer to SPI structure config
/// @param addr Address to start reading at
/// @param pRxData Pointer to data buffer
/// @param RxSize Size of data buffer
void MPU_REG_READ(SPI_HandleTypeDef *SPIx, uint8_t addr, uint8_t *pRxData, uint16_t RxSize)
{
	MPU_CS(CS_SELECT);
	uint8_t writeAddr = addr | READWRITE;
	HAL_SPI_Transmit(SPIx, &writeAddr, 1, SPI_TIMOUT_MS);
	HAL_SPI_Receive(SPIx, pRxData, RxSize, SPI_TIMOUT_MS);
	MPU_CS(CS_DESELECT);
}

/// @brief Set CS state to either start or end transmissions
/// @param state Set low to select, high to deselect
void MPU_CS(uint8_t state)
{
//	HAL_GPIO_WritePin(mpuConfig.CS_PORT, mpuConfig.CS_PIN, state);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, state);
}

/// @brief Set the accelerometer full scale range
/// @param SPIx Pointer to SPI structure config
/// @param mpuStruct Pointer to master MPU9250 struct
/// @param aScale Set 0 for ±2g, 1 for ±4g, 2 for ±8g, and 3 for ±16g
void setAccFullScaleRange(SPI_HandleTypeDef *SPIx,  MPU9250_t *pMPU9250, uint8_t aScale)
{
    // Variable init
    uint8_t addr = ACCEL_CONFIG;
    uint8_t val;

    // Set the value
    switch (aScale)
    {
    case AFS_2G:
        pMPU9250->sensorData.aScaleFactor = 16384.0;
        val = 0x00;
        MPU_REG_WRITE(SPIx, &addr, &val);
        break;
    case AFS_4G:
        pMPU9250->sensorData.aScaleFactor = 8192.0;
        val = 0x08;
        MPU_REG_WRITE(SPIx, &addr, &val);
        break;
    case AFS_8G:
        pMPU9250->sensorData.aScaleFactor = 4096.0;
        val = 0x10;
        MPU_REG_WRITE(SPIx, &addr, &val);
        break;
    case AFS_16G:
        pMPU9250->sensorData.aScaleFactor = 2048.0;
        val = 0x18;
        MPU_REG_WRITE(SPIx, &addr, &val);
        break;
    default:
        pMPU9250->sensorData.aScaleFactor = 8192.0;
        val = 0x08;
        MPU_REG_WRITE(SPIx, &addr, &val);
        break;
    }
}

/// @brief Set the gyroscope full scale range
/// @param SPIx Pointer to SPI structure config
/// @param mpuStruct Pointer to master MPU9250 struct
/// @param gScale Set 0 for ±250°/s, 1 for ±500°/s, 2 for ±1000°/s, and 3 for ±2000°/s
void setGyroFullScaleRange(SPI_HandleTypeDef *SPIx,  MPU9250_t *pMPU9250, uint8_t gScale)
{
    // Variable init
    uint8_t addr = GYRO_CONFIG;
    uint8_t val;

    // Set the value
    switch (gScale)
    {
    case GFS_250DPS:
        pMPU9250->sensorData.gScaleFactor = 131.0;
        val = 0x00;
        MPU_REG_WRITE(SPIx, &addr, &val);
        break;
    case GFS_500DPS:
        pMPU9250->sensorData.gScaleFactor = 65.5;
        val = 0x08;
        MPU_REG_WRITE(SPIx, &addr, &val);
        break;
    case GFS_1000DPS:
        pMPU9250->sensorData.gScaleFactor = 32.8;
        val = 0x10;
        MPU_REG_WRITE(SPIx, &addr, &val);
        break;
    case GFS_2000DPS:
        pMPU9250->sensorData.gScaleFactor = 16.4;
        val = 0x18;
        MPU_REG_WRITE(SPIx, &addr, &val);
        break;
    default:
        pMPU9250->sensorData.gScaleFactor = 65.5;
        val = 0x08;
        MPU_REG_WRITE(SPIx, &addr, &val);
        break;
    }
}
