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
#define CS_SELECT   0
#define CS_DESELECT 1

// Structures
struct MPU_Config
{
	uint8_t CS_PORT, CS_PIN;
} mpuConfig;

// Functions
void MPU_init();

void MPU_CS(uint8_t state);


#endif /* INC_MPU9250_H_ */
