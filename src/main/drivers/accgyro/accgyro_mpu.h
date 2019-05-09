#pragma once

#include "drivers/bus.h"
#include "drivers/exti.h"

#define MPU_RA_WHO_AM_I         0x75
#define MPU_RA_CONFIG           0x1A
#define MPU_RA_GYRO_CONFIG      0x1B

#define GYRO_USES_SPI

typedef void (*mpuResetFnPtr)(void);

extern mpuResetFnPtr mpuResetFn;

typedef struct mpuConfiguration_s {
    mpuResetFnPtr resetFn;
} mpuConfiguration_t;

enum gyro_fsr_e {
    INV_FSR_250DPS = 0,
    INV_FSR_500DPS,
    INV_FSR_1000DPS,
    INV_FSR_2000DPS,
    NUM_GYRO_FSR
};

enum fchoice_b {
    FCB_DISABLED = 0x00,
    FCB_8800_32 = 0x01,
    FCB_3600_32 = 0x02
};

enum clock_sel_e {
    INV_CLK_INTERNAL = 0,
    INV_CLK_PLL,
    NUM_CLK
};

enum accel_fsr_e {
    INV_FSR_2G = 0,
    INV_FSR_4G,
    INV_FSR_8G,
    INV_FSR_16G,
    NUM_ACCEL_FSR
};

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

void    mpuGyroInit(struct gyroDev_s *gyro);
bool    mpuGyroRead(struct gyroDev_s *gyro);
bool    mpuGyroReadSPI(struct gyroDev_s *gyro);
void    mpuDetect(struct gyroDev_s *gyro);
uint8_t mpuGyroDLPF(struct gyroDev_s *gyro);
uint8_t mpuGyroFCHOICE(struct gyroDev_s *gyro);
uint8_t mpuGyroReadRegister(const busDevice_t *bus, uint8_t reg);
bool mpuAccRead(struct accDev_s *acc);

