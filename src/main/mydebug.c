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

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "build/debug.h"

#include "common/streambuf.h"
#include "common/utils.h"
#include "common/crc.h"

#include "drivers/system.h"

#include "interface/msp.h"
#include "interface/cli.h"

#include "io/serial.h"

#include "drivers/bus.h"

#include <stdarg.h>

static void myputs(char * buf)
{
    extern serialPort_t * myDebugPort;

    for (char *p=buf; *p; p++) { 
        serialWrite(myDebugPort, *p);
    }
}


void mydebug(void)
{
    extern uint8_t lpfGlobal;
    extern uint8_t gyroSyncDenominatorGlobal;
    extern bool    gyro_use_32khz_global;

    extern void tfp_sprintf(char* s,char *fmt, ...);
    char buf[100];
    tfp_sprintf(buf, "lpf: %d    sync: %d    32khz: %d\n", lpfGlobal, gyroSyncDenominatorGlobal, gyro_use_32khz_global);
    myputs(buf);
}
