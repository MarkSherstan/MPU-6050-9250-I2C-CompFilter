/*
 * MPUXX50.h
 *
 *  Created on: Apr 26, 2022
 *      Author: MarkSherstan
 */

#ifndef SRC_MPUXX50_H_
#define SRC_MPUXX50_H_

// Libraries
#include <stdint.h>
#include <math.h>
#include "I2C.h"

// Constants
#define RAD2DEG 57.2957795131

// Defines
#define WHO_AM_I_6050_ANS 0x68
#define WHO_AM_I_9250_ANS 0x71
#define WHO_AM_I          0x75
#define AD0_LOW           0x68
#define AD0_HIGH          0x69
#define GYRO_CONFIG       0x1B
#define ACCEL_CONFIG      0x1C
#define PWR_MGMT_1        0x6B
#define ACCEL_XOUT_H      0x3B
#define I2C_TIMOUT_MS     1000

// Structs
struct RawData
{
    int16_t ax, ay, az, gx, gy, gz;
};

struct ProcessedData
{
    float ax, ay, az, gx, gy, gz;
};

struct GyroCal
{
    float x, y, z;
};

struct Attitude
{
    float r, p, y;
};

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

class MPUXX50
{
private:
    // Functions
    void writeGyroFullScaleRange(uint8_t gFSR);
    void writeAccFullScaleRange(uint8_t aFSR);

    // Variables
    float aScaleFactor, gScaleFactor;
    I2C_HandleTypeDef *_pI2Cx;
    uint8_t _addr;

    // Default values
    uint8_t _gFSR = GFSR_500DPS;
    uint8_t _aFSR = AFSR_4G;
    float _tau = 0.98;
    float _dt = 0.004;

    // Structs
    GyroCal gyroCal;
    Attitude attitude;

public:
    // Init
    MPUXX50(I2C_HandleTypeDef *pI2Cx, uint8_t addr);

    // Functions
    void calibrateGyro(uint16_t numCalPoints);
    ProcessedData processData();
    Attitude calcAttitude();
    RawData readRawData();
    uint8_t begin();

    void setGyroFullScaleRange(uint8_t gFSR);
    void setAccFullScaleRange(uint8_t aFSR);
    void setDeltaTime(float dt);
    void setTau(float tau);
};

#endif /* SRC_MPUXX50_H_ */
