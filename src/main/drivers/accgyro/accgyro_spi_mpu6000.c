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

/*
 * Authors:
 * Dominic Clifton - Cleanflight implementation
 * John Ihlein - Initial FF32 code
*/

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "platform.h"

#include "build/atomic.h"
#include "build/build_config.h"
#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/utils.h"

#include "drivers/bus.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_spi.h"
#include "drivers/nvic.h"
#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_mpu.h"
#include "drivers/accgyro/accgyro_spi_mpu6000.h"
#include "drivers/bus_spi.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/time.h"
#include "drivers/sensor.h"
#include "drivers/system.h"

static void mpu6000AccAndGyroInit(gyroDev_t *gyro);

static bool mpuSpi6000InitDone = false;

mpuResetFnPtr mpuResetFn;

#define MPU_ADDRESS        0x68
#define MPU_INQUIRY_MASK   0x7E

// Bits
#define BIT_SLEEP                   0x40
#define BIT_H_RESET                 0x80
#define BITS_CLKSEL                 0x07
#define MPU_CLK_SEL_PLLGYROX        0x01
#define MPU_CLK_SEL_PLLGYROZ        0x03
#define MPU_EXT_SYNC_GYROX          0x02
#define BITS_FS_250DPS              0x00
#define BITS_FS_500DPS              0x08
#define BITS_FS_1000DPS             0x10
#define BITS_FS_2000DPS             0x18
#define BITS_FS_2G                  0x00
#define BITS_FS_4G                  0x08
#define BITS_FS_8G                  0x10
#define BITS_FS_16G                 0x18
#define BITS_FS_MASK                0x18
#define BITS_DLPF_CFG_256HZ         0x00
#define BITS_DLPF_CFG_188HZ         0x01
#define BITS_DLPF_CFG_98HZ          0x02
#define BITS_DLPF_CFG_42HZ          0x03
#define BITS_DLPF_CFG_20HZ          0x04
#define BITS_DLPF_CFG_10HZ          0x05
#define BITS_DLPF_CFG_5HZ           0x06
#define BITS_DLPF_CFG_2100HZ_NOLPF  0x07
#define BITS_DLPF_CFG_MASK          0x07
#define BIT_INT_ANYRD_2CLEAR        0x10
#define BIT_RAW_RDY_EN              0x01
#define BIT_I2C_IF_DIS              0x10
#define BIT_INT_STATUS_DATA         0x01
#define BIT_GYRO                    3
#define BIT_ACC                     2
#define BIT_TEMP                    1

// Product ID Description for MPU6000
// high 4 bits low 4 bits
// Product Name Product Revision
#define MPU6000ES_REV_C4 0x14
#define MPU6000ES_REV_C5 0x15
#define MPU6000ES_REV_D6 0x16
#define MPU6000ES_REV_D7 0x17
#define MPU6000ES_REV_D8 0x18
#define MPU6000_REV_C4 0x54
#define MPU6000_REV_C5 0x55
#define MPU6000_REV_D6 0x56
#define MPU6000_REV_D7 0x57
#define MPU6000_REV_D8 0x58
#define MPU6000_REV_D9 0x59
#define MPU6000_REV_D10 0x5A

void mpu6000SpiGyroInit(gyroDev_t *gyro)
{
    mpuGyroInit(gyro);

    mpu6000AccAndGyroInit(gyro);

    spiSetDivisor(gyro->bus.busdev_u.spi.instance, SPI_CLOCK_INITIALIZATON);

    // Accel and Gyro DLPF Setting
    spiBusWriteRegister(&gyro->bus, MPU6000_CONFIG, mpuGyroDLPF(gyro));
    delayMicroseconds(1);

    spiSetDivisor(gyro->bus.busdev_u.spi.instance, SPI_CLOCK_FAST);  // 18 MHz SPI clock

    mpuGyroRead(gyro);

    if (((int8_t)gyro->gyroADCRaw[1]) == -1 && ((int8_t)gyro->gyroADCRaw[0]) == -1) {
        failureMode(FAILURE_GYRO_INIT_FAILED);
    }
}

void mpu6000SpiAccInit(accDev_t *acc)
{
    acc->acc_1G = 512 * 4;
}

uint8_t mpu6000SpiDetect(const busDevice_t *bus)
{
    IOInit(bus->busdev_u.spi.csnPin, OWNER_MPU_CS, 0);
    IOConfigGPIO(bus->busdev_u.spi.csnPin, SPI_IO_CS_CFG);
    IOHi(bus->busdev_u.spi.csnPin);

    spiSetDivisor(bus->busdev_u.spi.instance, SPI_CLOCK_INITIALIZATON);

    spiBusWriteRegister(bus, MPU_RA_PWR_MGMT_1, BIT_H_RESET);

    uint8_t attemptsRemaining = 5;
    do {
        delay(150);

        const uint8_t whoAmI = spiBusReadRegister(bus, MPU_RA_WHO_AM_I);
        if (whoAmI == MPU6000_WHO_AM_I_CONST) {
            break;
        }
        if (!attemptsRemaining) {
            return MPU_NONE;
        }
    } while (attemptsRemaining--);

    const uint8_t productID = spiBusReadRegister(bus, MPU_RA_PRODUCT_ID);

    /* look for a product ID we recognise */

    // verify product revision
    switch (productID) {
    case MPU6000ES_REV_C4:
    case MPU6000ES_REV_C5:
    case MPU6000_REV_C4:
    case MPU6000_REV_C5:
    case MPU6000ES_REV_D6:
    case MPU6000ES_REV_D7:
    case MPU6000ES_REV_D8:
    case MPU6000_REV_D6:
    case MPU6000_REV_D7:
    case MPU6000_REV_D8:
    case MPU6000_REV_D9:
    case MPU6000_REV_D10:
        return MPU_60x0_SPI;
    }

    return MPU_NONE;
}

static void mpu6000AccAndGyroInit(gyroDev_t *gyro)
{
    if (mpuSpi6000InitDone) {
        return;
    }

    spiSetDivisor(gyro->bus.busdev_u.spi.instance, SPI_CLOCK_INITIALIZATON);

    // Device Reset
    spiBusWriteRegister(&gyro->bus, MPU_RA_PWR_MGMT_1, BIT_H_RESET);
    delay(150);

    spiBusWriteRegister(&gyro->bus, MPU_RA_SIGNAL_PATH_RESET, BIT_GYRO | BIT_ACC | BIT_TEMP);
    delay(150);

    // Clock Source PPL with Z axis gyro reference
    spiBusWriteRegister(&gyro->bus, MPU_RA_PWR_MGMT_1, MPU_CLK_SEL_PLLGYROZ);
    delayMicroseconds(15);

    // Disable Primary I2C Interface
    spiBusWriteRegister(&gyro->bus, MPU_RA_USER_CTRL, BIT_I2C_IF_DIS);
    delayMicroseconds(15);

    spiBusWriteRegister(&gyro->bus, MPU_RA_PWR_MGMT_2, 0x00);
    delayMicroseconds(15);

    // Accel Sample Rate 1kHz
    // Gyroscope Output Rate =  1kHz when the DLPF is enabled
    spiBusWriteRegister(&gyro->bus, MPU_RA_SMPLRT_DIV, gyro->mpuDividerDrops);
    delayMicroseconds(15);

    // Gyro +/- 1000 DPS Full Scale
    spiBusWriteRegister(&gyro->bus, MPU_RA_GYRO_CONFIG, INV_FSR_2000DPS << 3);
    delayMicroseconds(15);

    // Accel +/- 16 G Full Scale
    spiBusWriteRegister(&gyro->bus, MPU_RA_ACCEL_CONFIG, INV_FSR_16G << 3);
    delayMicroseconds(15);

    spiBusWriteRegister(&gyro->bus, MPU_RA_INT_PIN_CFG, 0 << 7 | 0 << 6 | 0 << 5 | 1 << 4 | 0 << 3 | 0 << 2 | 0 << 1 | 0 << 0);  // INT_ANYRD_2CLEAR
    delayMicroseconds(15);

#ifdef USE_MPU_DATA_READY_SIGNAL
    spiBusWriteRegister(&gyro->bus, MPU_RA_INT_ENABLE, MPU_RF_DATA_RDY_EN);
    delayMicroseconds(15);
#endif

    spiSetDivisor(gyro->bus.busdev_u.spi.instance, SPI_CLOCK_FAST);
    delayMicroseconds(1);

    mpuSpi6000InitDone = true;
}

bool mpu6000SpiAccDetect(accDev_t *acc)
{
    if (acc->mpuDetectionResult.sensor != MPU_60x0_SPI) {
        return false;
    }

    acc->initFn = mpu6000SpiAccInit;
    acc->readFn = mpuAccRead;

    return true;
}

bool mpu6000SpiGyroDetect(gyroDev_t *gyro)
{
    if (gyro->mpuDetectionResult.sensor != MPU_60x0_SPI) {
        return false;
    }

    gyro->initFn = mpu6000SpiGyroInit;
    gyro->readFn = mpuGyroReadSPI;
    // 16.4 dps/lsb scalefactor
    gyro->scale = 1.0f / 16.4f;

    return true;
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

    spiBusSetInstance(&gyro->bus, MPU6000_SPI_INSTANCE);
    gyro->bus.busdev_u.spi.csnPin = gyro->bus.busdev_u.spi.csnPin == IO_NONE ? IOGetByTag(IO_TAG(MPU6000_CS_PIN)) : gyro->bus.busdev_u.spi.csnPin;
    sensor = mpu6000SpiDetect(&gyro->bus);
    gyro->mpuDetectionResult.sensor = sensor;
    return true;
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
