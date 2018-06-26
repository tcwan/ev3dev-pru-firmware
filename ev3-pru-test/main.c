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

timer_t timer_gettimestamp() {
    return TIMER64P0.TIM34;
}

// returns timer start time
timer_t timer_init(timer_t period) {
	last_timerexpiry = timer_gettimestamp();
	timer_period = period;
	return last_timerexpiry;
}

bool timer_hasexpired(timer_t *currtime) {
	*currtime = timer_gettimestamp();

	/* TIMER64P0.TIM34 is configured by Linux as a free run counter so we
	 * can use it here to keep track of time. This timer runs off of the
	 * external oscillator, so it runs at 24MHz (each count is 41.67ns).
	 * Since it counts up to the full unsigned 32-bit value, we can
	 * subtract without worrying about if the value wrapped around.
	 */
	if (*currtime - last_timerexpiry >= timer_period) {
		last_timerexpiry += timer_period;		// advance timer expiry timestamp
		return true;
	}
	else
		return false;
}

/* public functions */
int main(void) {

#ifdef ENABLE_LEDDEBUG

#define LEFTMOTOR MOTOR1
#define RIGHTMOTOR MOTOR2

    leddebug_init(FLASHING_INTERVAL);
    leddebug_assignmotors(LEFTMOTOR, RIGHTMOTOR);
#endif

    encodervec_t newevent = 0;

    timer_t currtime;

    // Debug info
    encoder_direction motor_direction;
    encoder_direction motor_olddirectionleft = UNKNOWN;
    encoder_direction motor_olddirectionright = UNKNOWN;
    encoder_count_t   motor_count;
    encoder_count_t   motor_oldcountleft = 0;
    encoder_count_t   motor_oldcountright = 0;

    // history buffer variables
    event_index_t index = 0;

    // Initialize tacho-encoder
    // Setup all ports for capture
    tachoencoder_init(RINGBUF_MAXITEMS);

    //LEDDEBUG(MOTOR1, FORWARD);               // Force Left and Right to alternate in the loop

    timer_init(TIMER_PERIOD);

    while (true) {

        if (tachoencoder_hasnewevent(&newevent)) {
            currtime = timer_gettimestamp();
            tachoencoder_updateencoderstate(newevent, currtime);        // Actual event timestamp
            motor_direction = tachoencoder_getdircount(LEFTMOTOR, &motor_count);

            if ((motor_direction != motor_olddirectionleft) || (motor_count != motor_oldcountleft)) {
                LEDDEBUG(LEFTMOTOR, motor_direction);               // Toggle LED state
                motor_oldcountleft = motor_count;
                motor_olddirectionleft = motor_direction;
            }
            motor_direction = tachoencoder_getdircount(RIGHTMOTOR, &motor_count);
            if ((motor_direction != motor_olddirectionright) || (motor_count != motor_oldcountright)) {
                LEDDEBUG(RIGHTMOTOR, motor_direction);               // Toggle LED state
                motor_oldcountright = motor_count;
                motor_olddirectionright = motor_direction;

            }
        }

        if (timer_hasexpired(&currtime)) {
            tachoencoder_updateteventbuffer(index, currtime);
            index += 1;
            if (index >= RINGBUF_MAXITEMS)
                index = 0;

        }
    }
}
