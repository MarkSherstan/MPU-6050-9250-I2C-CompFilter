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
#define READWRITE 		0x80
#define CS_SELECT   	0
#define CS_DESELECT 	1
#define SPI_TIMOUT_MS	1000

#define WHO_AM_I 	 	    		0x75
#define WHO_AM_I_9250_ANS   0x71

// Full scale ranges
enum gyroscopeFullScaleRange{GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS};
enum accelerometerFullScaleRange{AFS_2G, AFS_4G, AFS_8G, AFS_16G};

// Structures
typedef struct MPU9250
{
	uint8_t CS_PORT, CS_PIN;
	uint8_t aScale, gScale;
	float tau, dt;
} MPU9250_t;

// Functions
void MPU_CS(uint8_t state);
void MPU_REG_READ(SPI_HandleTypeDef *SPIx, uint8_t addr, uint8_t *pRxData, uint16_t RxSize);
void MPU_REG_WRITE(SPI_HandleTypeDef *SPIx, uint8_t *pReg);
uint8_t MPU_begin(SPI_HandleTypeDef *SPIx, MPU9250_t *ParamStruct);

#endif /* INC_MPU9250_H_ */
