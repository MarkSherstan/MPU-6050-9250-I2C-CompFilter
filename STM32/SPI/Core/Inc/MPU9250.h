/*
 * MPU9250.h
 *
 *  Created on: Mar 13, 2022
 *      Author: MarkSherstan
 */

#ifndef INC_MPU9250_H_
#define INC_MPU9250_H_

// VDDIO (Jumper to VDD) -> 3V3
// AD0/SDO -> MISO PB4
// SCL/SCLK -> PB3
// SDA/SDI -> MOSI PB5
// CS -> CS (PB6)
// GND -> GND


// Libraries
#include <stdint.h>
#include <math.h>
#include "SPI.h"

// Defines
#define CS_SELECT   	0
#define CS_DESELECT 	1
#define SPI_TIMOUT_MS	1000

// Structures
//typedef struct MPU9250
//{
//	uint8_t CS_PORT, CS_PIN;
//} MPU9250_t;

// Functions
void MPU_CS(uint8_t state);
void MPU_REG_READ(SPI_HandleTypeDef *SPIx, uint8_t *pReg, uint8_t *pRxData, uint16_t RxSize);
void MPU_REG_WRITE(SPI_HandleTypeDef *SPIx, uint8_t *pReg);


#endif /* INC_MPU9250_H_ */
