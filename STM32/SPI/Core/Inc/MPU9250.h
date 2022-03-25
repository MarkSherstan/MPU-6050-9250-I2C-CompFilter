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

// Constants
#define RAD2DEG 57.2957795131

// Defines
#define READWRITE       0x80
#define CS_SELECT       0
#define CS_DESELECT     1
#define SPI_TIMOUT_MS   1000

#define USER_CTRL           0x6A
#define PWR_MGMT_1   		0x6B
#define GYRO_CONFIG  		0x1B
#define ACCEL_CONFIG 		0x1C

#define ACCEL_XOUT_H        0x3B

#define WHO_AM_I          0x75
#define WHO_AM_I_9250_ANS 0x71

// Full scale ranges
enum gyroscopeFullScaleRange
{
    GFS_250DPS,
    GFS_500DPS,
    GFS_1000DPS,
    GFS_2000DPS
};

enum accelerometerFullScaleRange
{
    AFS_2G,
    AFS_4G,
    AFS_8G,
    AFS_16G
};

// Master structure
typedef struct MPU9250
{
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
        uint8_t CS_PORT, CS_PIN;
        uint8_t aFullScaleRange, gFullScaleRange;
    } settings;
} MPU9250_t;

// Functions
void MPU_CS(uint8_t state);
void MPU_REG_READ(SPI_HandleTypeDef *SPIx, uint8_t addr, uint8_t *pRxData, uint16_t RxSize);
void MPU_REG_WRITE(SPI_HandleTypeDef *SPIx, uint8_t *pAddr, uint8_t *pVal);
uint8_t MPU_begin(SPI_HandleTypeDef *SPIx, MPU9250_t *mpuStruct);
void setAccFullScaleRange(SPI_HandleTypeDef *SPIx, MPU9250_t *pMPU9250, uint8_t aScale);
void setGyroFullScaleRange(SPI_HandleTypeDef *SPIx,  MPU9250_t *pMPU9250, uint8_t gScale);
void readRawData(SPI_HandleTypeDef *SPIx, MPU9250_t *pMPU9250);
void MPU_calibrateGyro(SPI_HandleTypeDef *SPIx, MPU9250_t *pMPU9250, uint16_t numCalPoints);
void MPU_calcAttitude(SPI_HandleTypeDef *SPIx, MPU9250_t *pMPU9250);
void readProcessedData(SPI_HandleTypeDef *SPIx, MPU9250_t *pMPU9250);

#endif /* INC_MPU9250_H_ */
