/*
     ____ __     ____   ___    ____ __         (((((()
    | |_  \ \  /   ) ) | |  ) | |_  \ \  /  \(@)- /
    |_|__  \_\/  __)_) |_|_/  |_|__  \_\/   /(@)- \
                                               ((())))
 *//**
 *  \file   main.c
 *  \brief  ARM-BBR scaffolding routines
 *  \author  See AUTHORS for a full list of the developers
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *  * Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */


#include <stdbool.h>
#include <stdint.h>

#include <am18xx/sys_gpio.h>
#include <am18xx/sys_timer.h>

#include "resource_table_empty.h"
#include "tacho-encoder.h"

// define to flash LEDs for debugging
#define ENABLE_LEDDEBUG

#ifdef ENABLE_LEDDEBUG

#include "leddebug.h"
#define LEDDEBUG(motor, dir) leddebug_update(motor, dir)
#define FLASHING_INTERVAL 1                     // Controls the LED toggling interval per leddebug() calls

#else

#define LEDDEBUG(side, dir)

#endif


/* private functions */

// Periodic Timer routines
// Timer will auto-reload for next timer expiry

#define TIMER_PERIOD 12000000

static timer_t last_timerexpiry;
static timer_t timer_period;

// returns start time
timer_t timer_init(timer_t period) {
	last_timerexpiry = TIMER64P0.TIM34;
	timer_period = period;
	return last_timerexpiry;
}

bool timer_hasexpired(timer_t *currtime) {
	*currtime = TIMER64P0.TIM34;
	if (*currtime - last_timerexpiry >= timer_period) {
		last_timerexpiry += timer_period;		// advance timer expiry timestamp
		return true;
	}
	else
		return false;
}

int main(void) {

#ifdef ENABLE_LEDDEBUG
    leddebug_init(FLASHING_INTERVAL);
    leddebug_assignmotors(MOTOR0, MOTOR1);
#endif

    uint32_t start;
    encodervec oldevent = 0;
    timer_t currtime;


    // Initialize tacho-encoder
    // Setup all ports for capture
    tachoencoder_init(outA, outB, outC, outD);

    LEDDEBUG(MOTOR1, FORWARD);               // Force Left and Right to alternate in the loop

    timer_init(TIMER_PERIOD);

    /* blink the left green LED on the EV3 */
    while (true) {

        LEDDEBUG(MOTOR0, REVERSE);
        LEDDEBUG(MOTOR1, FORWARD);

        /* TIMER64P0.TIM34 is configured by Linux as a free run counter so we
         * can use it here to keep track of time. This timer runs off of the
         * external oscillator, so it runs at 24MHz (each count is 41.67ns).
         * Since it counts up to the full unsigned 32-bit value, we can
         * subtract without worrying about if the value wrapped around.
         */
        while (!timer_hasexpired(&currtime));

        LEDDEBUG(MOTOR0, REVERSE);
        LEDDEBUG(MOTOR1, FORWARD);

        while (!timer_hasexpired(&currtime));
    }
}
