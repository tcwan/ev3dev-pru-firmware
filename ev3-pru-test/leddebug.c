/*
     ____ __     ____   ___    ____ __         (((((()
    | |_  \ \  /   ) ) | |  ) | |_  \ \  /  \(@)- /
    |_|__  \_\/  __)_) |_|_/  |_|__  \_\/   /(@)- \
                                               ((())))
 *//**
 *  \file   leddebug.c
 *  \brief  Debug support routine for tacho motor encoders using EV3 LEDs
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
#include "leddebug.h"

static leddebug_state ledstate[MAX_TACHO_MOTORS];
static motor_identifier display_motor[MAX_LEDS];

/* Internal Routines */
void leddebug_setleds() {

    motor_identifier    motor;
    bool                led_enabled = false;
    encoder_direction   dir = UNKNOWN;

    motor = display_motor[LEFT];

    if ((unsigned) motor < MAX_TACHO_MOTORS) {
        leddebug_state *ledstateptr = &(ledstate[motor]);

        led_enabled = ledstateptr->led_enabled;
        dir = ledstateptr->dir;

        // program the LED control GPIOs
        LEFT_GREEN = ((led_enabled && ((dir == FORWARD) || (dir == UNKNOWN))) ? 1 : 0);
        LEFT_RED = ((led_enabled && ((dir == REVERSE) || (dir == UNKNOWN))) ? 1 : 0);
    }

    motor = display_motor[RIGHT];

    if ((unsigned) motor < MAX_TACHO_MOTORS) {
        leddebug_state *ledstateptr = &(ledstate[motor]);

        led_enabled = ledstateptr->led_enabled;
        dir = ledstateptr->dir;

        // program the LED control GPIOs
        RIGHT_GREEN = ((led_enabled && ((dir == FORWARD) || (dir == UNKNOWN))) ? 1 : 0);
        RIGHT_RED = ((led_enabled && ((dir == REVERSE) || (dir == UNKNOWN))) ? 1 : 0);
    }

}

/* Public Routines */
void leddebug_init(debug_count_t flashing_interval) {

    int i;
    leddebug_state *ledstateptr;

    for (i = 0; i < MAX_TACHO_MOTORS; i++) {
        ledstateptr = &(ledstate[i]);

        ledstateptr->led_enabled = false;
        ledstateptr->dir = UNKNOWN;
        ledstateptr->debug_count = 0;
        ledstateptr->flashing_interval = flashing_interval;
    }
    display_motor[LEFT] = display_motor[RIGHT] = MOTOR_UNUSED;
}


void leddebug_assignmotors(motor_identifier left, motor_identifier right) {
	display_motor[LEFT] = ((unsigned) left <= MOTOR_UNUSED) ? left : MOTOR_UNUSED;
    display_motor[RIGHT] = ((unsigned) right <= MOTOR_UNUSED) ? right : MOTOR_UNUSED;
}


void leddebug_update(motor_identifier motor, encoder_direction dir) {

	if ((unsigned) motor < MAX_TACHO_MOTORS) {
	    leddebug_state *ledstateptr = &(ledstate[motor]);

        if (ledstateptr->dir != dir) {
            // Reset counters
            ledstateptr->debug_count = 0;
            ledstateptr->led_enabled = false;
        }

        // Update counts
        ledstateptr->dir = dir;
        ledstateptr->debug_count++;                           // Debug count will always increment regardless of direction

        // Update led state
        if ((ledstateptr->debug_count % ledstateptr->flashing_interval) == 0) {
            ledstateptr->led_enabled = (ledstateptr->led_enabled ? false : true);   // Toggle LED state
        }
        leddebug_setleds();
    }
}
