/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "platform.h"

#include "build/atomic.h"
#include "build/build_config.h"
#include "build/debug.h"

#include "common/maths.h"
#include "common/utils.h"

#include "drivers/bus.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_spi.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/nvic.h"
#include "drivers/sensor.h"
#include "drivers/system.h"
#include "drivers/time.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_mpu3050.h"
#include "drivers/accgyro/accgyro_mpu6050.h"
#include "drivers/accgyro/accgyro_mpu6500.h"
#include "drivers/accgyro/accgyro_spi_bmi160.h"
#include "drivers/accgyro/accgyro_spi_icm20649.h"
#include "drivers/accgyro/accgyro_spi_icm20689.h"
#include "drivers/accgyro/accgyro_spi_mpu6000.h"
#include "drivers/accgyro/accgyro_spi_mpu6500.h"
#include "drivers/accgyro/accgyro_spi_mpu9250.h"
#include "drivers/accgyro/accgyro_mpu.h"

// =========================================================================

busType_e accBusType = 0;
busType_e gyroBusType = 0;

mpuResetFnPtr mpuResetFn;

#define MPU_ADDRESS             0x68

#define MPU_INQUIRY_MASK   0x7E

static void mpu6050FindRevision(gyroDev_t *gyro)
{
    // determine product ID and revision
    uint8_t readBuffer[6];
    bool ack = busReadRegisterBuffer(&gyro->bus, MPU_RA_XA_OFFS_H, readBuffer, 6);
    uint8_t revision = ((readBuffer[5] & 0x01) << 2) | ((readBuffer[3] & 0x01) << 1) | (readBuffer[1] & 0x01);
    if (ack && revision) {
        // Congrats, these parts are better
        if (revision == 1) {
            gyro->mpuDetectionResult.resolution = MPU_HALF_RESOLUTION;
        } else if (revision == 2) {
            gyro->mpuDetectionResult.resolution = MPU_FULL_RESOLUTION;
        } else if ((revision == 3) || (revision == 7)) {
            gyro->mpuDetectionResult.resolution = MPU_FULL_RESOLUTION;
        } else {
            failureMode(FAILURE_ACC_INCOMPATIBLE);
        }
    } else {
        uint8_t productId;
        ack = busReadRegisterBuffer(&gyro->bus, MPU_RA_PRODUCT_ID, &productId, 1);
        revision = productId & 0x0F;
        if (!ack || revision == 0) {
            failureMode(FAILURE_ACC_INCOMPATIBLE);
        } else if (revision == 4) {
            gyro->mpuDetectionResult.resolution = MPU_HALF_RESOLUTION;
        } else {
            gyro->mpuDetectionResult.resolution = MPU_FULL_RESOLUTION;
        }
    }
}

/*
 * Gyro interrupt service routine
 */
static void mpuIntExtiHandler(extiCallbackRec_t *cb)
{
    gyroDev_t *gyro = container_of(cb, gyroDev_t, exti);
    gyro->dataReady = true;
}

static void mpuIntExtiInit(gyroDev_t *gyro)
{
    if (gyro->mpuIntExtiTag == IO_TAG_NONE) {
        return;
    }

    const IO_t mpuIntIO = IOGetByTag(gyro->mpuIntExtiTag);

    IOInit(mpuIntIO, OWNER_MPU_EXTI, 0);
    IOConfigGPIO(mpuIntIO, IOCFG_IN_FLOATING);   // TODO - maybe pullup / pulldown ?

    EXTIHandlerInit(&gyro->exti, mpuIntExtiHandler);
    EXTIConfig(mpuIntIO, &gyro->exti, NVIC_PRIO_MPU_INT_EXTI, EXTI_Trigger_Rising);
    EXTIEnable(mpuIntIO, true);
}

bool mpuAccRead(accDev_t *acc)
{
    accBusType = acc->bus.bustype;

    uint8_t data[6];

    const bool ack = busReadRegisterBuffer(&acc->bus, MPU_RA_ACCEL_XOUT_H, data, 6);
    if (!ack) {
        return false;
    }

    acc->ADCRaw[X] = (int16_t)((data[0] << 8) | data[1]);
    acc->ADCRaw[Y] = (int16_t)((data[2] << 8) | data[3]);
    acc->ADCRaw[Z] = (int16_t)((data[4] << 8) | data[5]);

    return true;
}

bool mpuGyroRead(gyroDev_t *gyro)
{
    gyroBusType = gyro->bus.bustype;

    uint8_t data[6];

    const bool ack = busReadRegisterBuffer(&gyro->bus, MPU_RA_GYRO_XOUT_H, data, 6);
    if (!ack) {
        return false;
    }

    gyro->gyroADCRaw[X] = (int16_t)((data[0] << 8) | data[1]);
    gyro->gyroADCRaw[Y] = (int16_t)((data[2] << 8) | data[3]);
    gyro->gyroADCRaw[Z] = (int16_t)((data[4] << 8) | data[5]);

    return true;
}

bool mpuGyroReadSPI(gyroDev_t *gyro)
{
    static const uint8_t dataToSend[7] = {MPU_RA_GYRO_XOUT_H | 0x80, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    uint8_t data[7];

    const bool ack = spiBusTransfer(&gyro->bus, dataToSend, data, 7);
    if (!ack) {
        return false;
    }

    gyro->gyroADCRaw[X] = (int16_t)((data[1] << 8) | data[2]);
    gyro->gyroADCRaw[Y] = (int16_t)((data[3] << 8) | data[4]);
    gyro->gyroADCRaw[Z] = (int16_t)((data[5] << 8) | data[6]);

    return true;
}

static bool detectSPISensorsAndUpdateDetectionResult(gyroDev_t *gyro)
{
    UNUSED(gyro); // since there are FCs which have gyro on I2C but other devices on SPI

    uint8_t sensor = MPU_NONE;
    UNUSED(sensor);

    // note, when USE_DUAL_GYRO is enabled the gyro->bus must already be initialised.

    spiBusSetInstance(&gyro->bus, MPU6000_SPI_INSTANCE);
    gyro->bus.busdev_u.spi.csnPin = gyro->bus.busdev_u.spi.csnPin == IO_NONE ? IOGetByTag(IO_TAG(MPU6000_CS_PIN)) : gyro->bus.busdev_u.spi.csnPin;
    sensor = mpu6000SpiDetect(&gyro->bus);
    if (sensor != MPU_NONE) {
        gyro->mpuDetectionResult.sensor = sensor;
        return true;
    }

    spiBusSetInstance(&gyro->bus, MPU6500_SPI_INSTANCE);
    gyro->bus.busdev_u.spi.csnPin = gyro->bus.busdev_u.spi.csnPin == IO_NONE ? IOGetByTag(IO_TAG(MPU6500_CS_PIN)) : gyro->bus.busdev_u.spi.csnPin;
    sensor = mpu6500SpiDetect(&gyro->bus);
    // some targets using MPU_9250_SPI, ICM_20608_SPI or ICM_20602_SPI state sensor is MPU_65xx_SPI
    if (sensor != MPU_NONE) {
        gyro->mpuDetectionResult.sensor = sensor;
        return true;
    }

    spiBusSetInstance(&gyro->bus, ICM20689_SPI_INSTANCE);
    gyro->bus.busdev_u.spi.csnPin = gyro->bus.busdev_u.spi.csnPin == IO_NONE ? IOGetByTag(IO_TAG(ICM20689_CS_PIN)) : gyro->bus.busdev_u.spi.csnPin;
    sensor = icm20689SpiDetect(&gyro->bus);
    // icm20689SpiDetect detects ICM20602 and ICM20689
    if (sensor != MPU_NONE) {
        gyro->mpuDetectionResult.sensor = sensor;
        return true;
    }

    return false;
}

void mpuDetect(gyroDev_t *gyro)
{
    // MPU datasheet specifies 30ms.
    delay(35);

    if (gyro->bus.bustype == BUSTYPE_NONE) {
        // if no bustype is selected try I2C first.
        gyro->bus.bustype = BUSTYPE_I2C;
    }

    if (gyro->bus.bustype == BUSTYPE_I2C) {
        gyro->bus.busdev_u.i2c.device = MPU_I2C_INSTANCE;
        gyro->bus.busdev_u.i2c.address = MPU_ADDRESS;

        uint8_t sig = 0;
        bool ack = busReadRegisterBuffer(&gyro->bus, MPU_RA_WHO_AM_I, &sig, 1);

        if (ack) {
            // If an MPU3050 is connected sig will contain 0.
            uint8_t inquiryResult;
            ack = busReadRegisterBuffer(&gyro->bus, MPU_RA_WHO_AM_I_LEGACY, &inquiryResult, 1);
            inquiryResult &= MPU_INQUIRY_MASK;
            if (ack && inquiryResult == MPUx0x0_WHO_AM_I_CONST) {
                gyro->mpuDetectionResult.sensor = MPU_3050;
                return;
            }

            sig &= MPU_INQUIRY_MASK;
            if (sig == MPUx0x0_WHO_AM_I_CONST) {
                gyro->mpuDetectionResult.sensor = MPU_60x0;
                mpu6050FindRevision(gyro);
            } else if (sig == MPU6500_WHO_AM_I_CONST) {
                gyro->mpuDetectionResult.sensor = MPU_65xx_I2C;
            }
            return;
        }
    }

    gyro->bus.bustype = BUSTYPE_SPI;
    detectSPISensorsAndUpdateDetectionResult(gyro);
}

void mpuGyroInit(gyroDev_t *gyro)
{
    mpuIntExtiInit(gyro);
}

uint8_t mpuGyroDLPF(gyroDev_t *gyro)
{
    uint8_t ret;
    if (gyro->gyroRateKHz > GYRO_RATE_8_kHz) {
        ret = 0;  // If gyro is in 32KHz mode then the DLPF bits aren't used - set to 0
    } else {
        switch (gyro->hardware_lpf) {
            case GYRO_HARDWARE_LPF_NORMAL:
                ret = 0;
                break;
            case GYRO_HARDWARE_LPF_EXPERIMENTAL:
                ret = 7;
                break;
            case GYRO_HARDWARE_LPF_1KHZ_SAMPLE:
                ret = 1;
                break;
            default:
                ret = 0;
                break;
        }
    }
    return ret;
}

uint8_t mpuGyroFCHOICE(gyroDev_t *gyro)
{
    if (gyro->gyroRateKHz > GYRO_RATE_8_kHz) {
        if (gyro->hardware_32khz_lpf == GYRO_32KHZ_HARDWARE_LPF_EXPERIMENTAL) {
            return FCB_8800_32;
        } else {
            return FCB_3600_32;
        }
    } else {
        return FCB_DISABLED;  // Not in 32KHz mode, set FCHOICE to select 8KHz sampling
    }
}

uint8_t mpuGyroReadRegister(const busDevice_t *bus, uint8_t reg)
{
    uint8_t data;
    const bool ack = busReadRegisterBuffer(bus, reg, &data, 1);
    if (ack) {
        return data;
    } else {
        return 0;
    }

}
