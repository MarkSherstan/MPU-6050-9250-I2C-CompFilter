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

// Constants
#define RAD2DEG 57.2957795131

// Defines
#define WHO_AM_I_9250_ANS 0x71
#define WHO_AM_I          0x75
#define USER_CTRL         0x6A
#define PWR_MGMT_1        0x6B
#define GYRO_CONFIG       0x1B
#define ACCEL_CONFIG      0x1C
#define ACCEL_XOUT_H      0x3B
#define READWRITE         0x80
#define CS_SELECT         0
#define CS_DESELECT       1
#define SPI_TIMOUT_MS     1000

// Full scale ranges
enum gyroscopeFullScaleRange
{
    GFSR_250DPS,
    GFSR_500DPS,
    GFSR_1000DPS,
    GFSR_2000DPS
};

enum accelerometerFullScaleRange
{
    AFSR_2G,
    AFSR_4G,
    AFSR_8G,
    AFSR_16G
};

// Master structure
typedef struct MPU9250
{
    struct RawData
    {
        int16_t ax, ay, az, gx, gy, gz;
    } rawData;

    struct SensorData
    {
        float aScaleFactor, gScaleFactor;
        float ax, ay, az, gx, gy, gz;
    } sensorData;

    struct GyroCal
    {
        float x, y, z;
    } gyroCal;

    struct Attitude
    {
        float tau, dt;
        float r, p, y;
    } attitude;

    struct Settings
    {
    	uint8_t aFullScaleRange, gFullScaleRange;
    	GPIO_TypeDef *CS_PORT;
        uint8_t CS_PIN;
    } settings;
} MPU9250_t;

// Functions
uint8_t MPU_begin(SPI_HandleTypeDef *SPIx, MPU9250_t *mpuStruct);
void MPU_REG_READ(SPI_HandleTypeDef *SPIx, MPU9250_t *pMPU9250, uint8_t addr, uint8_t *pRxData, uint16_t RxSize);
void MPU_REG_WRITE(SPI_HandleTypeDef *SPIx, MPU9250_t *pMPU9250, uint8_t *pAddr, uint8_t *pVal);
void MPU_writeGyroFullScaleRange(SPI_HandleTypeDef *SPIx,  MPU9250_t *pMPU9250, uint8_t gScale);
void MPU_writeAccFullScaleRange(SPI_HandleTypeDef *SPIx, MPU9250_t *pMPU9250, uint8_t aScale);
void MPU_calibrateGyro(SPI_HandleTypeDef *SPIx, MPU9250_t *pMPU9250, uint16_t numCalPoints);
void MPU_readProcessedData(SPI_HandleTypeDef *SPIx, MPU9250_t *pMPU9250);
void MPU_calcAttitude(SPI_HandleTypeDef *SPIx, MPU9250_t *pMPU9250);
void MPU_readRawData(SPI_HandleTypeDef *SPIx, MPU9250_t *pMPU9250);
void MPU_CS(MPU9250_t *pMPU9250, uint8_t state);

#endif /* INC_MPU9250_H_ */
