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
#include <string.h>
#include <math.h>
#include <stdlib.h>

#include "platform.h"

#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/filter.h"

#include "config/feature.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_spi_mpu6000.h"

#include "drivers/bus_spi.h"
#include "drivers/io.h"

#include "fc/config.h"
#include "fc/runtime_config.h"

#include "io/beeper.h"
#include "io/statusindicator.h"

#include "scheduler/scheduler.h"

#include "sensors/boardalignment.h"
#include "sensors/gyro.h"
#include "sensors/gyroanalyse.h"
#include "sensors/sensors.h"

#ifdef USE_HARDWARE_REVISION_DETECTION
#include "hardware_revision.h"
#endif

FAST_RAM_ZERO_INIT gyro_t gyro;
static FAST_RAM_ZERO_INIT uint8_t gyroDebugMode;

static uint8_t gyroToUse = 0;

#ifdef USE_GYRO_OVERFLOW_CHECK
static FAST_RAM_ZERO_INIT uint8_t overflowAxisMask;
#endif
static FAST_RAM_ZERO_INIT float accumulatedMeasurements[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT float gyroPrevious[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT timeUs_t accumulatedMeasurementTimeUs;
static FAST_RAM_ZERO_INIT timeUs_t accumulationLastTimeSampledUs;

static bool gyroHasOverflowProtection = true;

typedef struct gyroCalibration_s {
    float sum[XYZ_AXIS_COUNT];
    stdev_t var[XYZ_AXIS_COUNT];
    int32_t cyclesRemaining;
} gyroCalibration_t;

bool firstArmingCalibrationWasStarted = false;

typedef union gyroLowpassFilter_u {
    pt1Filter_t pt1FilterState;
    biquadFilter_t biquadFilterState;
} gyroLowpassFilter_t;

typedef struct gyroSensor_s {
    gyroDev_t gyroDev;
    gyroCalibration_t calibration;

    // lowpass gyro soft filter
    filterApplyFnPtr lowpassFilterApplyFn;
    gyroLowpassFilter_t lowpassFilter[XYZ_AXIS_COUNT];

    // lowpass2 gyro soft filter
    filterApplyFnPtr lowpass2FilterApplyFn;
    gyroLowpassFilter_t lowpass2Filter[XYZ_AXIS_COUNT];

    // notch filters
    filterApplyFnPtr notchFilter1ApplyFn;
    biquadFilter_t notchFilter1[XYZ_AXIS_COUNT];

    filterApplyFnPtr notchFilter2ApplyFn;
    biquadFilter_t notchFilter2[XYZ_AXIS_COUNT];

    filterApplyFnPtr notchFilterDynApplyFn;
    biquadFilter_t notchFilterDyn[XYZ_AXIS_COUNT];

    // overflow and recovery
    timeUs_t overflowTimeUs;
    bool overflowDetected;
#ifdef USE_YAW_SPIN_RECOVERY
    timeUs_t yawSpinTimeUs;
    bool yawSpinDetected;
#endif // USE_YAW_SPIN_RECOVERY
} gyroSensor_t;

STATIC_UNIT_TESTED FAST_RAM_ZERO_INIT gyroSensor_t gyroSensor1;

#ifdef UNIT_TEST
STATIC_UNIT_TESTED gyroSensor_t * const gyroSensorPtr = &gyroSensor1;
STATIC_UNIT_TESTED gyroDev_t * const gyroDevPtr = &gyroSensor1.gyroDev;
#endif

static void gyroInitSensorFilters(gyroSensor_t *gyroSensor);
static void gyroInitLowpassFilterLpf(gyroSensor_t *gyroSensor, int slot, int type, uint16_t lpfHz);

#define DEBUG_GYRO_CALIBRATION 3

#define GYRO_SYNC_DENOM_DEFAULT 1

#define GYRO_OVERFLOW_TRIGGER_THRESHOLD 31980  // 97.5% full scale (1950dps for 2000dps gyro)
#define GYRO_OVERFLOW_RESET_THRESHOLD 30340    // 92.5% full scale (1850dps for 2000dps gyro)

PG_REGISTER_WITH_RESET_TEMPLATE(gyroConfig_t, gyroConfig, PG_GYRO_CONFIG, 4);

#ifndef GYRO_CONFIG_USE_GYRO_DEFAULT
#define GYRO_CONFIG_USE_GYRO_DEFAULT GYRO_CONFIG_USE_GYRO_1
#endif

PG_RESET_TEMPLATE(gyroConfig_t, gyroConfig,
    .gyro_align = ALIGN_DEFAULT,
    .gyroCalibrationDuration = 125,        // 1.25 seconds
    .gyroMovementCalibrationThreshold = 48,
    .gyro_sync_denom = GYRO_SYNC_DENOM_DEFAULT,
    .gyro_hardware_lpf = GYRO_HARDWARE_LPF_NORMAL,
    .gyro_32khz_hardware_lpf = GYRO_32KHZ_HARDWARE_LPF_NORMAL,
    .gyro_lowpass_type = FILTER_PT1,
    .gyro_lowpass_hz = 100,
    .gyro_lowpass2_type = FILTER_PT1,
    .gyro_lowpass2_hz = 300,
    .gyro_high_fsr = false,
    .gyro_use_32khz = false,
    .gyro_to_use = GYRO_CONFIG_USE_GYRO_DEFAULT,
    .gyro_soft_notch_hz_1 = 0,
    .gyro_soft_notch_cutoff_1 = 0,
    .gyro_soft_notch_hz_2 = 0,
    .gyro_soft_notch_cutoff_2 = 0,
    .checkOverflow = GYRO_OVERFLOW_CHECK_ALL_AXES,
    .gyro_offset_yaw = 0,
    .yaw_spin_recovery = true,
    .yaw_spin_threshold = 1950,
);


const busDevice_t *gyroSensorBus(void)
{
    return &gyroSensor1.gyroDev.bus;
}

#ifdef USE_GYRO_REGISTER_DUMP
const busDevice_t *gyroSensorBusByDevice(uint8_t whichSensor)
{
    UNUSED(whichSensor);
    return &gyroSensor1.gyroDev.bus;
}
#endif // USE_GYRO_REGISTER_DUMP

const mpuConfiguration_t *gyroMpuConfiguration(void)
{
    return &gyroSensor1.gyroDev.mpuConfiguration;
}

const mpuDetectionResult_t *gyroMpuDetectionResult(void)
{
    return &gyroSensor1.gyroDev.mpuDetectionResult;
}

STATIC_UNIT_TESTED gyroSensor_e gyroDetect(gyroDev_t *dev)
{
    gyroSensor_e gyroHardware = GYRO_DEFAULT;

    switch (gyroHardware) {
    case GYRO_DEFAULT:
        FALLTHROUGH;

    case GYRO_MPU6000:
        if (mpu6000SpiGyroDetect(dev)) {
            gyroHardware = GYRO_MPU6000;
#ifdef GYRO_MPU6000_ALIGN
            dev->gyroAlign = GYRO_MPU6000_ALIGN;
#endif
            break;
        }
        FALLTHROUGH;

    default:
        gyroHardware = GYRO_NONE;
    }

    if (gyroHardware != GYRO_NONE) {
        detectedSensors[SENSOR_INDEX_GYRO] = gyroHardware;
        sensorsSet(SENSOR_GYRO);
    }


    return gyroHardware;
}

static bool gyroInitSensor(gyroSensor_t *gyroSensor)
{
    gyroSensor->gyroDev.gyro_high_fsr = gyroConfig()->gyro_high_fsr;


    mpuDetect(&gyroSensor->gyroDev);
    mpuResetFn = gyroSensor->gyroDev.mpuConfiguration.resetFn; // must be set after mpuDetect

    const gyroSensor_e gyroHardware = gyroDetect(&gyroSensor->gyroDev);
    gyroSensor->gyroDev.gyroHardware = gyroHardware;
    if (gyroHardware == GYRO_NONE) {
        return false;
    }

    // gyro does not support 32kHz
    gyroConfigMutable()->gyro_use_32khz = false;

    // Must set gyro targetLooptime before gyroDev.init and initialisation of filters
    gyro.targetLooptime = gyroSetSampleRate(&gyroSensor->gyroDev, gyroConfig()->gyro_hardware_lpf, gyroConfig()->gyro_sync_denom, gyroConfig()->gyro_use_32khz);
    gyroSensor->gyroDev.hardware_lpf = gyroConfig()->gyro_hardware_lpf;
    gyroSensor->gyroDev.hardware_32khz_lpf = gyroConfig()->gyro_32khz_hardware_lpf;
    gyroSensor->gyroDev.initFn(&gyroSensor->gyroDev);
    if (gyroConfig()->gyro_align != ALIGN_DEFAULT) {
        gyroSensor->gyroDev.gyroAlign = gyroConfig()->gyro_align;
    }

    // As new gyros are supported, be sure to add them below based on whether they are subject to the overflow/inversion bug
    // Any gyro not explicitly defined will default to not having built-in overflow protection as a safe alternative.
    switch (gyroHardware) {
    case GYRO_NONE:    // Won't ever actually get here, but included to account for all gyro types
    case GYRO_DEFAULT:
    case GYRO_FAKE:
    case GYRO_MPU6050:
    case GYRO_L3G4200D:
    case GYRO_MPU3050:
    case GYRO_L3GD20:
    case GYRO_BMI160:
    case GYRO_MPU6000:
    case GYRO_MPU6500:
    case GYRO_MPU9250:
        gyroSensor->gyroDev.gyroHasOverflowProtection = true;
        break;

    case GYRO_ICM20601:
    case GYRO_ICM20602:
    case GYRO_ICM20608G:
    case GYRO_ICM20649:  // we don't actually know if this is affected, but as there are currently no flight controllers using it we err on the side of caution
    case GYRO_ICM20689:
        gyroSensor->gyroDev.gyroHasOverflowProtection = false;
        break;

    default:
        gyroSensor->gyroDev.gyroHasOverflowProtection = false;  // default catch for newly added gyros until proven to be unaffected
        break;
    }

    gyroInitSensorFilters(gyroSensor);

#ifdef USE_GYRO_DATA_ANALYSE
    gyroDataAnalyseInit(gyro.targetLooptime);
#endif
    return true;
}

bool gyroInit(void)
{
#ifdef USE_GYRO_OVERFLOW_CHECK
    if (gyroConfig()->checkOverflow == GYRO_OVERFLOW_CHECK_YAW) {
        overflowAxisMask = GYRO_OVERFLOW_Z;
    } else if (gyroConfig()->checkOverflow == GYRO_OVERFLOW_CHECK_ALL_AXES) {
        overflowAxisMask = GYRO_OVERFLOW_X | GYRO_OVERFLOW_Y | GYRO_OVERFLOW_Z;
    } else {
        overflowAxisMask = 0;
    }
#endif

    switch (debugMode) {
    case DEBUG_FFT:
    case DEBUG_GYRO_RAW:
    case DEBUG_GYRO_SCALED:
    case DEBUG_GYRO_FILTERED:
        gyroDebugMode = debugMode;
        break;
    default:
        // debugMode is not gyro-related
        gyroDebugMode = DEBUG_NONE;
        break;
    }
    firstArmingCalibrationWasStarted = false;

    bool ret = false;
    memset(&gyro, 0, sizeof(gyro));
    gyroToUse = gyroConfig()->gyro_to_use;

#if defined(USE_DUAL_GYRO) && defined(GYRO_1_CS_PIN)
    if (gyroToUse == GYRO_CONFIG_USE_GYRO_1 || gyroToUse == GYRO_CONFIG_USE_GYRO_BOTH) {
        gyroSensor1.gyroDev.bus.busdev_u.spi.csnPin = IOGetByTag(IO_TAG(GYRO_1_CS_PIN));
        IOInit(gyroSensor1.gyroDev.bus.busdev_u.spi.csnPin, OWNER_MPU_CS, RESOURCE_INDEX(0));
        IOHi(gyroSensor1.gyroDev.bus.busdev_u.spi.csnPin); // Ensure device is disabled, important when two devices are on the same bus.
        IOConfigGPIO(gyroSensor1.gyroDev.bus.busdev_u.spi.csnPin, SPI_IO_CS_CFG);
    }
#endif

#if defined(USE_DUAL_GYRO) && defined(GYRO_2_CS_PIN)
    if (gyroToUse == GYRO_CONFIG_USE_GYRO_2 || gyroToUse == GYRO_CONFIG_USE_GYRO_BOTH) {
        gyroSensor2.gyroDev.bus.busdev_u.spi.csnPin = IOGetByTag(IO_TAG(GYRO_2_CS_PIN));
        IOInit(gyroSensor2.gyroDev.bus.busdev_u.spi.csnPin, OWNER_MPU_CS, RESOURCE_INDEX(1));
        IOHi(gyroSensor2.gyroDev.bus.busdev_u.spi.csnPin); // Ensure device is disabled, important when two devices are on the same bus.
        IOConfigGPIO(gyroSensor2.gyroDev.bus.busdev_u.spi.csnPin, SPI_IO_CS_CFG);
    }
#endif

    gyroSensor1.gyroDev.gyroAlign = ALIGN_DEFAULT;

#if defined(GYRO_1_EXTI_PIN)
    gyroSensor1.gyroDev.mpuIntExtiTag =  IO_TAG(GYRO_1_EXTI_PIN);
#elif defined(MPU_INT_EXTI)
    gyroSensor1.gyroDev.mpuIntExtiTag =  IO_TAG(MPU_INT_EXTI);
#elif defined(USE_HARDWARE_REVISION_DETECTION)
    gyroSensor1.gyroDev.mpuIntExtiTag =  selectMPUIntExtiConfigByHardwareRevision();
#else
    gyroSensor1.gyroDev.mpuIntExtiTag =  IO_TAG_NONE;
#endif // GYRO_1_EXTI_PIN
#ifdef USE_DUAL_GYRO
#ifdef GYRO_1_ALIGN
    gyroSensor1.gyroDev.gyroAlign = GYRO_1_ALIGN;
#endif
    gyroSensor1.gyroDev.bus.bustype = BUSTYPE_SPI;
    spiBusSetInstance(&gyroSensor1.gyroDev.bus, GYRO_1_SPI_INSTANCE);
    if (gyroToUse == GYRO_CONFIG_USE_GYRO_1 || gyroToUse == GYRO_CONFIG_USE_GYRO_BOTH) {
        ret = gyroInitSensor(&gyroSensor1);
        if (!ret) {
            return false; // TODO handle failure of first gyro detection better. - Perhaps update the config to use second gyro then indicate a new failure mode and reboot.
        }
        gyroHasOverflowProtection =  gyroHasOverflowProtection && gyroSensor1.gyroDev.gyroHasOverflowProtection;
    }
#else // USE_DUAL_GYRO
    ret = gyroInitSensor(&gyroSensor1);
    gyroHasOverflowProtection =  gyroHasOverflowProtection && gyroSensor1.gyroDev.gyroHasOverflowProtection;
#endif // USE_DUAL_GYRO

#ifdef USE_DUAL_GYRO

    gyroSensor2.gyroDev.gyroAlign = ALIGN_DEFAULT;

#if defined(GYRO_2_EXTI_PIN)
    gyroSensor2.gyroDev.mpuIntExtiTag =  IO_TAG(GYRO_2_EXTI_PIN);
#elif defined(USE_HARDWARE_REVISION_DETECTION)
    gyroSensor2.gyroDev.mpuIntExtiTag =  selectMPUIntExtiConfigByHardwareRevision();
#else
    gyroSensor2.gyroDev.mpuIntExtiTag =  IO_TAG_NONE;
#endif // GYRO_2_EXTI_PIN
#ifdef GYRO_2_ALIGN
    gyroSensor2.gyroDev.gyroAlign = GYRO_2_ALIGN;
#endif
    gyroSensor2.gyroDev.bus.bustype = BUSTYPE_SPI;
    spiBusSetInstance(&gyroSensor2.gyroDev.bus, GYRO_2_SPI_INSTANCE);
    if (gyroToUse == GYRO_CONFIG_USE_GYRO_2 || gyroToUse == GYRO_CONFIG_USE_GYRO_BOTH) {
        ret = gyroInitSensor(&gyroSensor2);
        if (!ret) {
            return false; // TODO handle failure of second gyro detection better. - Perhaps update the config to use first gyro then indicate a new failure mode and reboot.
        }
        gyroHasOverflowProtection =  gyroHasOverflowProtection && gyroSensor2.gyroDev.gyroHasOverflowProtection;
    }
#endif // USE_DUAL_GYRO

#ifdef USE_DUAL_GYRO
    // Only allow using both gyros simultaneously if they are the same hardware type.
    // If the user selected "BOTH" and they are not the same type, then reset to using only the first gyro.
    if (gyroToUse == GYRO_CONFIG_USE_GYRO_BOTH) {
        if (gyroSensor1.gyroDev.gyroHardware != gyroSensor2.gyroDev.gyroHardware) {
            gyroToUse = GYRO_CONFIG_USE_GYRO_1;
            gyroConfigMutable()->gyro_to_use = GYRO_CONFIG_USE_GYRO_1;
            detectedSensors[SENSOR_INDEX_GYRO] = gyroSensor1.gyroDev.gyroHardware;
            sensorsSet(SENSOR_GYRO);

        }
    }
#endif // USE_DUAL_GYRO
    return ret;
}

void gyroInitLowpassFilterLpf(gyroSensor_t *gyroSensor, int slot, int type, uint16_t lpfHz)
{
    filterApplyFnPtr *lowpassFilterApplyFn;
    gyroLowpassFilter_t *lowpassFilter = NULL;

    switch (slot) {
    case FILTER_LOWPASS:
        lowpassFilterApplyFn = &gyroSensor->lowpassFilterApplyFn;
        lowpassFilter = gyroSensor->lowpassFilter;
        break;

    case FILTER_LOWPASS2:
        lowpassFilterApplyFn = &gyroSensor->lowpass2FilterApplyFn;
        lowpassFilter = gyroSensor->lowpass2Filter;
        break;

    default:
        return;
    }

    // Establish some common constants
    const uint32_t gyroFrequencyNyquist = 1000000 / 2 / gyro.targetLooptime;
    const float gyroDt = gyro.targetLooptime * 1e-6f;

    // Gain could be calculated a little later as it is specific to the pt1/bqrcf2/fkf branches
    const float gain = pt1FilterGain(lpfHz, gyroDt);

    // Dereference the pointer to null before checking valid cutoff and filter
    // type. It will be overridden for positive cases.
    *lowpassFilterApplyFn = &nullFilterApply;

    // If lowpass cutoff has been specified and is less than the Nyquist frequency
    if (lpfHz && lpfHz <= gyroFrequencyNyquist) {
        switch (type) {
        case FILTER_PT1:
            *lowpassFilterApplyFn = (filterApplyFnPtr) pt1FilterApply;
            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                pt1FilterInit(&lowpassFilter[axis].pt1FilterState, gain);
            }
            break;
        case FILTER_BIQUAD:
            *lowpassFilterApplyFn = (filterApplyFnPtr) biquadFilterApply;
            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                biquadFilterInitLPF(&lowpassFilter[axis].biquadFilterState, lpfHz, gyro.targetLooptime);
            }
            break;
        }
    }
}

static uint16_t calculateNyquistAdjustedNotchHz(uint16_t notchHz, uint16_t notchCutoffHz)
{
    const uint32_t gyroFrequencyNyquist = 1000000 / 2 / gyro.targetLooptime;
    if (notchHz > gyroFrequencyNyquist) {
        if (notchCutoffHz < gyroFrequencyNyquist) {
            notchHz = gyroFrequencyNyquist;
        } else {
            notchHz = 0;
        }
    }

    return notchHz;
}

#if defined(USE_GYRO_SLEW_LIMITER)
void gyroInitSlewLimiter(gyroSensor_t *gyroSensor) {

    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        gyroSensor->gyroDev.gyroADCRawPrevious[axis] = 0;
    }
}
#endif

static void gyroInitFilterNotch1(gyroSensor_t *gyroSensor, uint16_t notchHz, uint16_t notchCutoffHz)
{
    gyroSensor->notchFilter1ApplyFn = nullFilterApply;

    notchHz = calculateNyquistAdjustedNotchHz(notchHz, notchCutoffHz);

    if (notchHz != 0 && notchCutoffHz != 0) {
        gyroSensor->notchFilter1ApplyFn = (filterApplyFnPtr)biquadFilterApply;
        const float notchQ = filterGetNotchQ(notchHz, notchCutoffHz);
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            biquadFilterInit(&gyroSensor->notchFilter1[axis], notchHz, gyro.targetLooptime, notchQ, FILTER_NOTCH);
        }
    }
}

static void gyroInitFilterNotch2(gyroSensor_t *gyroSensor, uint16_t notchHz, uint16_t notchCutoffHz)
{
    gyroSensor->notchFilter2ApplyFn = nullFilterApply;

    notchHz = calculateNyquistAdjustedNotchHz(notchHz, notchCutoffHz);

    if (notchHz != 0 && notchCutoffHz != 0) {
        gyroSensor->notchFilter2ApplyFn = (filterApplyFnPtr)biquadFilterApply;
        const float notchQ = filterGetNotchQ(notchHz, notchCutoffHz);
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            biquadFilterInit(&gyroSensor->notchFilter2[axis], notchHz, gyro.targetLooptime, notchQ, FILTER_NOTCH);
        }
    }
}

#ifdef USE_GYRO_DATA_ANALYSE
static bool isDynamicFilterActive(void)
{
    return feature(FEATURE_DYNAMIC_FILTER);
}

static void gyroInitFilterDynamicNotch(gyroSensor_t *gyroSensor)
{
    gyroSensor->notchFilterDynApplyFn = nullFilterApply;

    if (isDynamicFilterActive()) {
        gyroSensor->notchFilterDynApplyFn = (filterApplyFnPtr)biquadFilterApplyDF1; // must be this function, not DF2
        const float notchQ = filterGetNotchQ(400, 390); //just any init value
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            biquadFilterInit(&gyroSensor->notchFilterDyn[axis], 400, gyro.targetLooptime, notchQ, FILTER_NOTCH);
        }
    }
}
#endif


static void gyroInitSensorFilters(gyroSensor_t *gyroSensor)
{
#if defined(USE_GYRO_SLEW_LIMITER)
    gyroInitSlewLimiter(gyroSensor);
#endif

    gyroInitLowpassFilterLpf(
      gyroSensor,
      FILTER_LOWPASS,
      gyroConfig()->gyro_lowpass_type,
      gyroConfig()->gyro_lowpass_hz
    );

    gyroInitLowpassFilterLpf(
      gyroSensor,
      FILTER_LOWPASS2,
      gyroConfig()->gyro_lowpass2_type,
      gyroConfig()->gyro_lowpass2_hz
    );

    gyroInitFilterNotch1(gyroSensor, gyroConfig()->gyro_soft_notch_hz_1, gyroConfig()->gyro_soft_notch_cutoff_1);
    gyroInitFilterNotch2(gyroSensor, gyroConfig()->gyro_soft_notch_hz_2, gyroConfig()->gyro_soft_notch_cutoff_2);
#ifdef USE_GYRO_DATA_ANALYSE
    gyroInitFilterDynamicNotch(gyroSensor);
#endif
}

void gyroInitFilters(void)
{
    gyroInitSensorFilters(&gyroSensor1);
#ifdef USE_DUAL_GYRO
    gyroInitSensorFilters(&gyroSensor2);
#endif
}

FAST_CODE bool isGyroSensorCalibrationComplete(const gyroSensor_t *gyroSensor)
{
    return gyroSensor->calibration.cyclesRemaining == 0;
}

FAST_CODE bool isGyroCalibrationComplete(void)
{
#ifdef USE_DUAL_GYRO
    switch (gyroToUse) {
        default:
        case GYRO_CONFIG_USE_GYRO_1: {
            return isGyroSensorCalibrationComplete(&gyroSensor1);
        }
        case GYRO_CONFIG_USE_GYRO_2: {
            return isGyroSensorCalibrationComplete(&gyroSensor2);
        }
        case GYRO_CONFIG_USE_GYRO_BOTH: {
            return isGyroSensorCalibrationComplete(&gyroSensor1) && isGyroSensorCalibrationComplete(&gyroSensor2);
        }
    }
#else
    return isGyroSensorCalibrationComplete(&gyroSensor1);
#endif
}

static bool isOnFinalGyroCalibrationCycle(const gyroCalibration_t *gyroCalibration)
{
    return gyroCalibration->cyclesRemaining == 1;
}

static int32_t gyroCalculateCalibratingCycles(void)
{
    return (gyroConfig()->gyroCalibrationDuration * 10000) / gyro.targetLooptime;
}

static bool isOnFirstGyroCalibrationCycle(const gyroCalibration_t *gyroCalibration)
{
    return gyroCalibration->cyclesRemaining == gyroCalculateCalibratingCycles();
}

static void gyroSetCalibrationCycles(gyroSensor_t *gyroSensor)
{
    gyroSensor->calibration.cyclesRemaining = gyroCalculateCalibratingCycles();
}

void gyroStartCalibration(bool isFirstArmingCalibration)
{
    if (!(isFirstArmingCalibration && firstArmingCalibrationWasStarted)) {
        gyroSetCalibrationCycles(&gyroSensor1);
#ifdef USE_DUAL_GYRO
        gyroSetCalibrationCycles(&gyroSensor2);
#endif

        if (isFirstArmingCalibration) {
            firstArmingCalibrationWasStarted = true;
        }
    }
}

bool isFirstArmingGyroCalibrationRunning(void)
{
    return firstArmingCalibrationWasStarted && !isGyroCalibrationComplete();
}

STATIC_UNIT_TESTED void performGyroCalibration(gyroSensor_t *gyroSensor, uint8_t gyroMovementCalibrationThreshold)
{
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        // Reset g[axis] at start of calibration
        if (isOnFirstGyroCalibrationCycle(&gyroSensor->calibration)) {
            gyroSensor->calibration.sum[axis] = 0.0f;
            devClear(&gyroSensor->calibration.var[axis]);
            // gyroZero is set to zero until calibration complete
            gyroSensor->gyroDev.gyroZero[axis] = 0.0f;
        }

        // Sum up CALIBRATING_GYRO_TIME_US readings
        gyroSensor->calibration.sum[axis] += gyroSensor->gyroDev.gyroADCRaw[axis];
        devPush(&gyroSensor->calibration.var[axis], gyroSensor->gyroDev.gyroADCRaw[axis]);

        if (isOnFinalGyroCalibrationCycle(&gyroSensor->calibration)) {
            const float stddev = devStandardDeviation(&gyroSensor->calibration.var[axis]);
            // DEBUG_GYRO_CALIBRATION records the standard deviation of roll
            // into the spare field - debug[3], in DEBUG_GYRO_RAW
            if (axis == X) {
                DEBUG_SET(DEBUG_GYRO_RAW, DEBUG_GYRO_CALIBRATION, lrintf(stddev));
            }

            // check deviation and startover in case the model was moved
            if (gyroMovementCalibrationThreshold && stddev > gyroMovementCalibrationThreshold) {
                gyroSetCalibrationCycles(gyroSensor);
                return;
            }

            // please take care with exotic boardalignment !!
            gyroSensor->gyroDev.gyroZero[axis] = gyroSensor->calibration.sum[axis] / gyroCalculateCalibratingCycles();
            if (axis == Z) {
              gyroSensor->gyroDev.gyroZero[axis] -= ((float)gyroConfig()->gyro_offset_yaw / 100);
            }
        }
    }

    if (isOnFinalGyroCalibrationCycle(&gyroSensor->calibration)) {
        schedulerResetTaskStatistics(TASK_SELF); // so calibration cycles do not pollute tasks statistics
        if (!firstArmingCalibrationWasStarted || (getArmingDisableFlags() & ~ARMING_DISABLED_CALIBRATING) == 0) {
            beeper(BEEPER_GYRO_CALIBRATED);
        }
    }
    --gyroSensor->calibration.cyclesRemaining;

}

#if defined(USE_GYRO_SLEW_LIMITER)
FAST_CODE int32_t gyroSlewLimiter(gyroSensor_t *gyroSensor, int axis)
{
    int32_t ret = (int32_t)gyroSensor->gyroDev.gyroADCRaw[axis];
    if (gyroConfig()->checkOverflow || gyroHasOverflowProtection) {
        // don't use the slew limiter if overflow checking is on or gyro is not subject to overflow bug
        return ret;
    }
    if (abs(ret - gyroSensor->gyroDev.gyroADCRawPrevious[axis]) > (1<<14)) {
        // there has been a large change in value, so assume overflow has occurred and return the previous value
        ret = gyroSensor->gyroDev.gyroADCRawPrevious[axis];
    } else {
        gyroSensor->gyroDev.gyroADCRawPrevious[axis] = ret;
    }
    return ret;
}
#endif

#ifdef USE_GYRO_OVERFLOW_CHECK
static FAST_CODE_NOINLINE void handleOverflow(gyroSensor_t *gyroSensor, timeUs_t currentTimeUs)
{
    const float gyroOverflowResetRate = GYRO_OVERFLOW_RESET_THRESHOLD * gyroSensor->gyroDev.scale;
    if ((abs(gyro.gyroADCf[X]) < gyroOverflowResetRate)
          && (abs(gyro.gyroADCf[Y]) < gyroOverflowResetRate)
          && (abs(gyro.gyroADCf[Z]) < gyroOverflowResetRate)) {
        // if we have 50ms of consecutive OK gyro vales, then assume yaw readings are OK again and reset overflowDetected
        // reset requires good OK values on all axes
        if (cmpTimeUs(currentTimeUs, gyroSensor->overflowTimeUs) > 50000) {
            gyroSensor->overflowDetected = false;
        }
    } else {
        // not a consecutive OK value, so reset the overflow time
        gyroSensor->overflowTimeUs = currentTimeUs;
    }
}

static FAST_CODE void checkForOverflow(gyroSensor_t *gyroSensor, timeUs_t currentTimeUs)
{
    // check for overflow to handle Yaw Spin To The Moon (YSTTM)
    // ICM gyros are specified to +/- 2000 deg/sec, in a crash they can go out of spec.
    // This can cause an overflow and sign reversal in the output.
    // Overflow and sign reversal seems to result in a gyro value of +1996 or -1996.
    if (gyroSensor->overflowDetected) {
        handleOverflow(gyroSensor, currentTimeUs);
    } else {
#ifndef SIMULATOR_BUILD
        // check for overflow in the axes set in overflowAxisMask
        gyroOverflow_e overflowCheck = GYRO_OVERFLOW_NONE;
        const float gyroOverflowTriggerRate = GYRO_OVERFLOW_TRIGGER_THRESHOLD * gyroSensor->gyroDev.scale;
        if (abs(gyro.gyroADCf[X]) > gyroOverflowTriggerRate) {
            overflowCheck |= GYRO_OVERFLOW_X;
        }
        if (abs(gyro.gyroADCf[Y]) > gyroOverflowTriggerRate) {
            overflowCheck |= GYRO_OVERFLOW_Y;
        }
        if (abs(gyro.gyroADCf[Z]) > gyroOverflowTriggerRate) {
            overflowCheck |= GYRO_OVERFLOW_Z;
        }
        if (overflowCheck & overflowAxisMask) {
            gyroSensor->overflowDetected = true;
            gyroSensor->overflowTimeUs = currentTimeUs;
#ifdef USE_YAW_SPIN_RECOVERY
            gyroSensor->yawSpinDetected = false;
#endif // USE_YAW_SPIN_RECOVERY
        }
#endif // SIMULATOR_BUILD
    }
}
#endif // USE_GYRO_OVERFLOW_CHECK

#ifdef USE_YAW_SPIN_RECOVERY
static FAST_CODE_NOINLINE void handleYawSpin(gyroSensor_t *gyroSensor, timeUs_t currentTimeUs)
{
    const float yawSpinResetRate = gyroConfig()->yaw_spin_threshold - 100.0f;
    if (abs(gyro.gyroADCf[Z]) < yawSpinResetRate) {
        // testing whether 20ms of consecutive OK gyro yaw values is enough
        if (cmpTimeUs(currentTimeUs, gyroSensor->yawSpinTimeUs) > 20000) {
            gyroSensor->yawSpinDetected = false;
        }
    } else {
        // reset the yaw spin time
        gyroSensor->yawSpinTimeUs = currentTimeUs;
    }
}

static FAST_CODE void checkForYawSpin(gyroSensor_t *gyroSensor, timeUs_t currentTimeUs)
{
    // if not in overflow mode, handle yaw spins above threshold
#ifdef USE_GYRO_OVERFLOW_CHECK
    if (gyroSensor->overflowDetected) {
        gyroSensor->yawSpinDetected = false;
        return;
    }
#endif // USE_GYRO_OVERFLOW_CHECK

    if (gyroSensor->yawSpinDetected) {
        handleYawSpin(gyroSensor, currentTimeUs);
    } else {
#ifndef SIMULATOR_BUILD
        // check for spin on yaw axis only
         if (abs(gyro.gyroADCf[Z]) > gyroConfig()->yaw_spin_threshold) {
            gyroSensor->yawSpinDetected = true;
            gyroSensor->yawSpinTimeUs = currentTimeUs;
        }
#endif // SIMULATOR_BUILD
    }
}
#endif // USE_YAW_SPIN_RECOVERY

static FAST_CODE FAST_CODE_NOINLINE void gyroUpdateSensor(gyroSensor_t *gyroSensor, timeUs_t currentTimeUs)
{
    if (!gyroSensor->gyroDev.readFn(&gyroSensor->gyroDev)) {
        return;
    }
    gyroSensor->gyroDev.dataReady = false;

    if (isGyroSensorCalibrationComplete(gyroSensor)) {
        // move 16-bit gyro data into 32-bit variables to avoid overflows in calculations

#if defined(USE_GYRO_SLEW_LIMITER)
        gyroSensor->gyroDev.gyroADC[X] = gyroSlewLimiter(gyroSensor, X) - gyroSensor->gyroDev.gyroZero[X];
        gyroSensor->gyroDev.gyroADC[Y] = gyroSlewLimiter(gyroSensor, Y) - gyroSensor->gyroDev.gyroZero[Y];
        gyroSensor->gyroDev.gyroADC[Z] = gyroSlewLimiter(gyroSensor, Z) - gyroSensor->gyroDev.gyroZero[Z];
#else
        gyroSensor->gyroDev.gyroADC[X] = gyroSensor->gyroDev.gyroADCRaw[X] - gyroSensor->gyroDev.gyroZero[X];
        gyroSensor->gyroDev.gyroADC[Y] = gyroSensor->gyroDev.gyroADCRaw[Y] - gyroSensor->gyroDev.gyroZero[Y];
        gyroSensor->gyroDev.gyroADC[Z] = gyroSensor->gyroDev.gyroADCRaw[Z] - gyroSensor->gyroDev.gyroZero[Z];
#endif

        alignSensors(gyroSensor->gyroDev.gyroADC, gyroSensor->gyroDev.gyroAlign);
    } else {
        performGyroCalibration(gyroSensor, gyroConfig()->gyroMovementCalibrationThreshold);
        // still calibrating, so no need to further process gyro data
        return;
    }

#ifdef USE_GYRO_DATA_ANALYSE
    if (isDynamicFilterActive()) {
        gyroDataAnalyse(&gyroSensor->gyroDev, gyroSensor->notchFilterDyn);
    }
#endif

    const timeDelta_t sampleDeltaUs = currentTimeUs - accumulationLastTimeSampledUs;
    accumulationLastTimeSampledUs = currentTimeUs;
    accumulatedMeasurementTimeUs += sampleDeltaUs;

#ifdef USE_GYRO_OVERFLOW_CHECK
    if (gyroConfig()->checkOverflow && !gyroHasOverflowProtection) {
        checkForOverflow(gyroSensor, currentTimeUs);
    }
#endif

#ifdef USE_YAW_SPIN_RECOVERY
    if (gyroConfig()->yaw_spin_recovery) {
        checkForYawSpin(gyroSensor, currentTimeUs);
    }
#endif

    if (gyroDebugMode == DEBUG_NONE) {
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            // NOTE: this branch optimized for when there is no gyro debugging, ensure it is kept in step with non-optimized branch
            float gyroADCf = gyroSensor->gyroDev.gyroADC[axis] * gyroSensor->gyroDev.scale;

#ifdef USE_GYRO_DATA_ANALYSE
            gyroADCf = gyroSensor->notchFilterDynApplyFn((filter_t *)&gyroSensor->notchFilterDyn[axis], gyroADCf);
#endif
            gyroADCf = gyroSensor->notchFilter1ApplyFn((filter_t *)&gyroSensor->notchFilter1[axis], gyroADCf);
            gyroADCf = gyroSensor->notchFilter2ApplyFn((filter_t *)&gyroSensor->notchFilter2[axis], gyroADCf);
            gyroADCf = gyroSensor->lowpassFilterApplyFn((filter_t *)&gyroSensor->lowpassFilter[axis], gyroADCf);
            gyroADCf = gyroSensor->lowpass2FilterApplyFn((filter_t *)&gyroSensor->lowpass2Filter[axis], gyroADCf);
            gyroSensor->gyroDev.gyroADCf[axis] = gyroADCf;

            if (!gyroSensor->overflowDetected) {
                // integrate using trapezium rule to avoid bias
                accumulatedMeasurements[axis] += 0.5f * (gyroPrevious[axis] + gyroADCf) * sampleDeltaUs;
                gyroPrevious[axis] = gyroADCf;
            }
        }
    } else {
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            DEBUG_SET(DEBUG_GYRO_RAW, axis, gyroSensor->gyroDev.gyroADCRaw[axis]);
            // scale gyro output to degrees per second
            float gyroADCf = gyroSensor->gyroDev.gyroADC[axis] * gyroSensor->gyroDev.scale;
            // DEBUG_GYRO_SCALED records the unfiltered, scaled gyro output
            DEBUG_SET(DEBUG_GYRO_SCALED, axis, lrintf(gyroADCf));

#ifdef USE_GYRO_DATA_ANALYSE
            // apply dynamic notch filter
            if (isDynamicFilterActive()) {
                if (axis == X) {
                    DEBUG_SET(DEBUG_FFT, 0, lrintf(gyroADCf)); // store raw data
                }
                gyroADCf = gyroSensor->notchFilterDynApplyFn((filter_t *)&gyroSensor->notchFilterDyn[axis], gyroADCf);
                if (axis == X) {
                    DEBUG_SET(DEBUG_FFT, 1, lrintf(gyroADCf)); // store data after dynamic notch
                }
            }
#endif
            // apply static notch filters and software lowpass filters
            gyroADCf = gyroSensor->notchFilter1ApplyFn((filter_t *)&gyroSensor->notchFilter1[axis], gyroADCf);
            gyroADCf = gyroSensor->notchFilter2ApplyFn((filter_t *)&gyroSensor->notchFilter2[axis], gyroADCf);
            gyroADCf = gyroSensor->lowpassFilterApplyFn((filter_t *)&gyroSensor->lowpassFilter[axis], gyroADCf);
            gyroADCf = gyroSensor->lowpass2FilterApplyFn((filter_t *)&gyroSensor->lowpass2Filter[axis], gyroADCf);
            // DEBUG_GYRO_FILTERED records the scaled, filtered, after all software filtering has been applied.
            DEBUG_SET(DEBUG_GYRO_FILTERED, axis, lrintf(gyroADCf));

            gyroSensor->gyroDev.gyroADCf[axis] = gyroADCf;
            if (!gyroSensor->overflowDetected) {
                // integrate using trapezium rule to avoid bias
                accumulatedMeasurements[axis] += 0.5f * (gyroPrevious[axis] + gyroADCf) * sampleDeltaUs;
                gyroPrevious[axis] = gyroADCf;
            }
        }
    }
}

FAST_CODE void gyroUpdate(timeUs_t currentTimeUs)
{
#ifdef USE_DUAL_GYRO
    switch (gyroToUse) {
    case GYRO_CONFIG_USE_GYRO_1:
        gyroUpdateSensor(&gyroSensor1, currentTimeUs);
        if (isGyroSensorCalibrationComplete(&gyroSensor1)) {
            gyro.gyroADCf[X] = gyroSensor1.gyroDev.gyroADCf[X];
            gyro.gyroADCf[Y] = gyroSensor1.gyroDev.gyroADCf[Y];
            gyro.gyroADCf[Z] = gyroSensor1.gyroDev.gyroADCf[Z];
        }
        DEBUG_SET(DEBUG_DUAL_GYRO_RAW, 0, gyroSensor1.gyroDev.gyroADCRaw[X]);
        DEBUG_SET(DEBUG_DUAL_GYRO_RAW, 1, gyroSensor1.gyroDev.gyroADCRaw[Y]);
        DEBUG_SET(DEBUG_DUAL_GYRO, 0, lrintf(gyroSensor1.gyroDev.gyroADCf[X]));
        DEBUG_SET(DEBUG_DUAL_GYRO, 1, lrintf(gyroSensor1.gyroDev.gyroADCf[Y]));
        DEBUG_SET(DEBUG_DUAL_GYRO_COMBINE, 0, lrintf(gyro.gyroADCf[X]));
        DEBUG_SET(DEBUG_DUAL_GYRO_COMBINE, 1, lrintf(gyro.gyroADCf[Y]));
        break;
    case GYRO_CONFIG_USE_GYRO_2:
        gyroUpdateSensor(&gyroSensor2, currentTimeUs);
        if (isGyroSensorCalibrationComplete(&gyroSensor2)) {
            gyro.gyroADCf[X] = gyroSensor2.gyroDev.gyroADCf[X];
            gyro.gyroADCf[Y] = gyroSensor2.gyroDev.gyroADCf[Y];
            gyro.gyroADCf[Z] = gyroSensor2.gyroDev.gyroADCf[Z];
        }
        DEBUG_SET(DEBUG_DUAL_GYRO_RAW, 2, gyroSensor2.gyroDev.gyroADCRaw[X]);
        DEBUG_SET(DEBUG_DUAL_GYRO_RAW, 3, gyroSensor2.gyroDev.gyroADCRaw[Y]);
        DEBUG_SET(DEBUG_DUAL_GYRO, 2, lrintf(gyroSensor2.gyroDev.gyroADCf[X]));
        DEBUG_SET(DEBUG_DUAL_GYRO, 3, lrintf(gyroSensor2.gyroDev.gyroADCf[Y]));
        DEBUG_SET(DEBUG_DUAL_GYRO_COMBINE, 2, lrintf(gyro.gyroADCf[X]));
        DEBUG_SET(DEBUG_DUAL_GYRO_COMBINE, 3, lrintf(gyro.gyroADCf[Y]));
        break;
    case GYRO_CONFIG_USE_GYRO_BOTH:
        gyroUpdateSensor(&gyroSensor1, currentTimeUs);
        gyroUpdateSensor(&gyroSensor2, currentTimeUs);
        if (isGyroSensorCalibrationComplete(&gyroSensor1) && isGyroSensorCalibrationComplete(&gyroSensor2)) {
            gyro.gyroADCf[X] = (gyroSensor1.gyroDev.gyroADCf[X] + gyroSensor2.gyroDev.gyroADCf[X]) / 2.0f;
            gyro.gyroADCf[Y] = (gyroSensor1.gyroDev.gyroADCf[Y] + gyroSensor2.gyroDev.gyroADCf[Y]) / 2.0f;
            gyro.gyroADCf[Z] = (gyroSensor1.gyroDev.gyroADCf[Z] + gyroSensor2.gyroDev.gyroADCf[Z]) / 2.0f;
        }
        DEBUG_SET(DEBUG_DUAL_GYRO_RAW, 0, gyroSensor1.gyroDev.gyroADCRaw[X]);
        DEBUG_SET(DEBUG_DUAL_GYRO_RAW, 1, gyroSensor1.gyroDev.gyroADCRaw[Y]);
        DEBUG_SET(DEBUG_DUAL_GYRO, 0, lrintf(gyroSensor1.gyroDev.gyroADCf[X]));
        DEBUG_SET(DEBUG_DUAL_GYRO, 1, lrintf(gyroSensor1.gyroDev.gyroADCf[Y]));
        DEBUG_SET(DEBUG_DUAL_GYRO_RAW, 2, gyroSensor2.gyroDev.gyroADCRaw[X]);
        DEBUG_SET(DEBUG_DUAL_GYRO_RAW, 3, gyroSensor2.gyroDev.gyroADCRaw[Y]);
        DEBUG_SET(DEBUG_DUAL_GYRO, 2, lrintf(gyroSensor2.gyroDev.gyroADCf[X]));
        DEBUG_SET(DEBUG_DUAL_GYRO, 3, lrintf(gyroSensor2.gyroDev.gyroADCf[Y]));
        DEBUG_SET(DEBUG_DUAL_GYRO_COMBINE, 1, lrintf(gyro.gyroADCf[X]));
        DEBUG_SET(DEBUG_DUAL_GYRO_COMBINE, 2, lrintf(gyro.gyroADCf[Y]));
        DEBUG_SET(DEBUG_DUAL_GYRO_DIFF, 0, lrintf(gyroSensor1.gyroDev.gyroADCf[X] - gyroSensor2.gyroDev.gyroADCf[X]));
        DEBUG_SET(DEBUG_DUAL_GYRO_DIFF, 1, lrintf(gyroSensor1.gyroDev.gyroADCf[Y] - gyroSensor2.gyroDev.gyroADCf[Y]));
        DEBUG_SET(DEBUG_DUAL_GYRO_DIFF, 2, lrintf(gyroSensor1.gyroDev.gyroADCf[Z] - gyroSensor2.gyroDev.gyroADCf[Z]));
        break;
    }
#else
    gyroUpdateSensor(&gyroSensor1, currentTimeUs);
    gyro.gyroADCf[X] = gyroSensor1.gyroDev.gyroADCf[X];
    gyro.gyroADCf[Y] = gyroSensor1.gyroDev.gyroADCf[Y];
    gyro.gyroADCf[Z] = gyroSensor1.gyroDev.gyroADCf[Z];
#endif
}

bool gyroGetAccumulationAverage(float *accumulationAverage)
{
    if (accumulatedMeasurementTimeUs > 0) {
        // If we have gyro data accumulated, calculate average rate that will yield the same rotation
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            accumulationAverage[axis] = accumulatedMeasurements[axis] / accumulatedMeasurementTimeUs;
            accumulatedMeasurements[axis] = 0.0f;
        }
        accumulatedMeasurementTimeUs = 0;
        return true;
    } else {
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            accumulationAverage[axis] = 0.0f;
        }
        return false;
    }
}

void gyroReadTemperature(void)
{
    if (gyroSensor1.gyroDev.temperatureFn) {
        gyroSensor1.gyroDev.temperatureFn(&gyroSensor1.gyroDev, &gyroSensor1.gyroDev.temperature);
    }
}

int16_t gyroGetTemperature(void)
{
    return gyroSensor1.gyroDev.temperature;
}

int16_t gyroRateDps(int axis)
{
#ifdef USE_DUAL_GYRO
    if (gyroToUse == GYRO_CONFIG_USE_GYRO_2) {
        return lrintf(gyro.gyroADCf[axis] / gyroSensor2.gyroDev.scale);
    } else {
        return lrintf(gyro.gyroADCf[axis] / gyroSensor1.gyroDev.scale);
    }
#else
    return lrintf(gyro.gyroADCf[axis] / gyroSensor1.gyroDev.scale);
#endif
}

bool gyroOverflowDetected(void)
{
    return gyroSensor1.overflowDetected;
}

#ifdef USE_YAW_SPIN_RECOVERY
bool gyroYawSpinDetected(void)
{
    return gyroSensor1.yawSpinDetected;
}
#endif // USE_YAW_SPIN_RECOVERY

uint16_t gyroAbsRateDps(int axis)
{
    return fabsf(gyro.gyroADCf[axis]);
}
