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

#include "leddebug.h"

static leddebug_state ledstate[MAX_TACHO_MOTORS];
static motor_identifier display_motor[MAX_LEDS];

/* Internal Routines */
void leddebug_setleds() {
    // program the LED control GPIOs
	if ((unsigned) display_motor[LEFT] < MAX_TACHO_MOTORS) {
		LEFT_GREEN  = (ledstate[display_motor[LEFT]].green_state ? 1 : 0);
		LEFT_RED    = (ledstate[display_motor[LEFT]].red_state ? 1 : 0);
	}
	if ((unsigned) display_motor[RIGHT] < MAX_TACHO_MOTORS) {
		RIGHT_GREEN = (ledstate[display_motor[RIGHT]].green_state ? 1 : 0);
		RIGHT_RED   = (ledstate[display_motor[RIGHT]].red_state ? 1 : 0);
	}
}

/* Public Routines */
void leddebug_init(debug_count_t flashing_interval) {

    int i;
    for (i = 0; i < MAX_TACHO_MOTORS; i++) {
        ledstate[i].dir = UNKNOWN;
        ledstate[i].debug_count = 0;
        ledstate[i].flashing_interval = flashing_interval;
        ledstate[i].green_state = false;
        ledstate[i].red_state = false;
    }
    display_motor[LEFT] = display_motor[RIGHT] = MOTOR_UNUSED;
}


void leddebug_assignmotors(motor_identifier left, motor_identifier right) {
	display_motor[LEFT] = ((unsigned) left <= MOTOR_UNUSED) ? left : MOTOR_UNUSED;
    display_motor[RIGHT] = ((unsigned) right <= MOTOR_UNUSED) ? right : MOTOR_UNUSED;
}


void leddebug_update(motor_identifier motor, encoder_direction dir) {

	if ((unsigned) motor < MAX_TACHO_MOTORS) {

        if (ledstate[motor].dir != dir) {
            // Reset counters
            ledstate[motor].debug_count = 0;
            ledstate[motor].green_state = false;
            ledstate[motor].red_state = false;
        }

        // Update counts
        ledstate[motor].dir = dir;
        ledstate[motor].debug_count++;                           // Debug count will always increment regardless of direction

        // Update led state
        switch (dir) {
        case FORWARD:
            ledstate[motor].red_state = false;
            if ((ledstate[motor].debug_count % ledstate[motor].flashing_interval) == 0)
                ledstate[motor].green_state = (ledstate[motor].green_state ? false : true);
            break;
        case REVERSE:
            ledstate[motor].green_state = false;
            if ((ledstate[motor].debug_count % ledstate[motor].flashing_interval) == 0)
                ledstate[motor].red_state = (ledstate[motor].red_state ? false : true);
            break;
        default:
            ledstate[motor].green_state = true;
            ledstate[motor].red_state = true;
            break;
        }

        // program the LED control GPIOs
        leddebug_setleds();
    }
}
