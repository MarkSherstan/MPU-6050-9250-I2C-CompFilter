/*
 * MPU9250.h
 *
 *  Created on: Mar 13, 2022
 *      Author: MarkSherstan
 */

#ifndef INC_MPU9250_H_
#define INC_MPU9250_H_


// Libraries
#include <stdint.h>
#include <math.h>
#include "SPI.h"

// Defines
#define CS_SELECT   	0
#define CS_DESELECT 	1
#define SPI_TIMOUT_MS	1000

// Structures
struct MPU_Config
{
	// TODO: Set this up as -> notation for practice
	uint8_t CS_PORT, CS_PIN;
} mpuConfig;

// Functions
void MPU_CS(uint8_t state);
void MPU_REG_READ(SPI_HandleTypeDef *SPIx, uint8_t *pReg, uint8_t *pRxData, uint16_t RxSize);
void MPU_REG_WRITE(SPI_HandleTypeDef *SPIx, uint8_t *pReg);


#endif /* INC_MPU9250_H_ */
