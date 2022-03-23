/*
 * MPU9250.c
 *
 *  Created on: Mar 13, 2022
 *      Author: MarkSherstan
 */

#include "MPU9250.h"

/// @brief Check for connection, reset IMU, and set full range scale.
/// @param aScale Set accelerometer full scale range: 0 for ±2g, 1 for ±4g, 2 for ±8g, and 3 for ±16g.
/// @param gScale Set gyroscope full scale range: 0 for ±250°/s, 1 for ±500°/s, 2 for ±1000°/s, and 3 for ±2000°/s.
/// @param tau Set tau value for the complementary filter (typically 0.98)
/// @param dt Set sampling rate in seconds determined by the timer interrupt 
uint8_t MPU_begin(SPI_HandleTypeDef *SPIx, uint8_t aScale, uint8_t gScale, float tau, float dt)
{
    // Save values
    
    // Initialize variables
    uint8_t check;
    uint8_t select;

    // Confirm device
    MPU_REG_READ(SPIx, WHO_AM_I, &check, 1);
    if ((check == WHO_AM_I_9250_ANS) || (check == WHO_AM_I_6050_ANS))
    {
        return 1;
    }
    else 
    {
        return 0;
    }
}

/// @brief Read a specific registry address
/// @param SPIx Pointer to SPI structure config
/// @param pReg Pointer containing address and value to write
void MPU_REG_WRITE(SPI_HandleTypeDef *SPIx, uint8_t *pReg)
{
	MPU_CS(CS_SELECT);
	HAL_SPI_Transmit(SPIx, pReg, 2, SPI_TIMOUT_MS);
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


// I2C_IF_DIS    Start-Up Time for Register Read/Write” in Section 6.3