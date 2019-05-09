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

#include "platform.h"

#include "fc/fc_init.h"

#include "scheduler/scheduler.h"

#include "io/serial.h"
#include "drivers/pwm_output.h"

void run(void);

int main(void)
{
    init();

    run();

    return 0;
}

void FAST_CODE FAST_CODE_NOINLINE run(void)
{
    while (true) {
        scheduler();
        processLoopback();
#ifdef SIMULATOR_BUILD
        delayMicroseconds_real(50); // max rate 20kHz
#endif
    }
}

#include <stdarg.h>

void myprintf(const char *fmt, ...)
{
    extern serialPort_t * myDebugPort;
    extern void tfp_sprintf(char* s,char *fmt, ...);

    va_list ap;
    va_start(ap, fmt);
    char buf[200];
    //tfp_sprintf(buf, 200, fmt, ap); 
    vsnprintf(buf, 200, fmt, ap); 

    for (char *p=buf; *p; p++) { 
        serialWrite(myDebugPort, *p);
    }

    va_end(ap);
}


