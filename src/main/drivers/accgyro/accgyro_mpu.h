#pragma once

#include "drivers/bus.h"
#include "drivers/exti.h"

typedef void (*mpuResetFnPtr)(void);

extern mpuResetFnPtr mpuResetFn;

typedef struct mpuConfiguration_s {
    mpuResetFnPtr resetFn;
} mpuConfiguration_t;

typedef enum {
    GYRO_OVERFLOW_NONE = 0x00,
    GYRO_OVERFLOW_X = 0x01,
    GYRO_OVERFLOW_Y = 0x02,
    GYRO_OVERFLOW_Z = 0x04
} gyroOverflow_e;

typedef enum {
    MPU_NONE,
    MPU_3050,
    MPU_60x0,
    MPU_60x0_SPI,
    MPU_65xx_I2C,
    MPU_65xx_SPI,
    MPU_9250_SPI,
    ICM_20601_SPI,
    ICM_20602_SPI,
    ICM_20608_SPI,
    ICM_20649_SPI,
    ICM_20689_SPI,
    BMI_160_SPI,
} mpuSensor_e;

typedef enum {
    MPU_HALF_RESOLUTION,
    MPU_FULL_RESOLUTION
} mpu6050Resolution_e;

typedef struct mpuDetectionResult_s {
    mpuSensor_e sensor;
    mpu6050Resolution_e resolution;
} mpuDetectionResult_t;

struct gyroDev_s;
struct accDev_s;


