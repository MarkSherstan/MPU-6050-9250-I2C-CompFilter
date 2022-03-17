/*
 * MPU9250.c
 *
 *  Created on: Mar 13, 2022
 *      Author: MarkSherstan
 */

#include "MPU9250.h"

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
/// @param pReg Pointer of address to start reading at
/// @param pRxData Pointer to data buffer
/// @param RxSize Size of data buffer
void MPU_REG_READ(SPI_HandleTypeDef *SPIx, uint8_t *pReg, uint8_t *pRxData, uint16_t RxSize)
{
	MPU_CS(CS_SELECT);

	HAL_SPI_Transmit(SPIx, pReg, 1, SPI_TIMOUT_MS);
	HAL_SPI_Receive(SPIx, pRxData, RxSize, SPI_TIMOUT_MS);

	MPU_CS(CS_DESELECT);
}

/// @brief Set CS state to either start or end transmissions
/// @param state Set low to select, high to deselect
void MPU_CS(uint8_t state)
{
	HAL_GPIO_WritePin(mpuConfig.CS_PORT, mpuConfig.CS_PIN, state);
}
