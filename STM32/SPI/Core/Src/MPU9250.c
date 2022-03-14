/*
 * MPU9250.c
 *
 *  Created on: Mar 13, 2022
 *      Author: MarkSherstan
 */

#include "MPU9250.h"

/// @brief Set CS state to either start or end transmissions
/// @param state Set low to select, high to deselect
void MPU_CS(uint8_t state)
{
	HAL_GPIO_WritePin(mpuConfig.CS_PORT, mpuConfig.CS_PIN, state);
}

