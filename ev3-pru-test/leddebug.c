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

void init_leddebug(debug_count_t flashing_rate) {

    int i;
    for (i = 0; i < MAX_TACHO_MOTORS; i++) {
        ledstate[i].dir = UNKNOWN;
        ledstate[i].debug_count = 0;
        ledstate[i].flashing_rate = flashing_rate;
        ledstate[i].green_state = false;
        ledstate[i].red_state = false;
    }
}


void leddebug(motor_identifier side, encoder_direction dir) {

    // FIXME: Supports only two motors for now
    if ((side == LEFT) || (side == RIGHT)) {

        if (ledstate[side].dir != dir) {
            // Reset counters
            ledstate[side].debug_count = 0;
            ledstate[side].green_state = false;
            ledstate[side].red_state = false;
        }

        // Update counts
        ledstate[side].dir = dir;
        ledstate[side].debug_count++;                           // Debug count will always increment regardless of direction

        // Update led state
        switch (dir) {
        case FORWARD:
            ledstate[side].red_state = false;
            if ((ledstate[side].debug_count % ledstate[side].flashing_rate) == 0)
                ledstate[side].green_state = (ledstate[side].green_state ? false : true);
            break;
        case REVERSE:
            ledstate[side].green_state = false;
            if ((ledstate[side].debug_count % ledstate[side].flashing_rate) == 0)
                ledstate[side].red_state = (ledstate[side].red_state ? false : true);
            break;
        default:
            ledstate[side].green_state = true;
            ledstate[side].red_state = true;
            break;
        }

        // program the LED control GPIOs
        LEFT_GREEN  = (ledstate[LEFT].green_state ? 1 : 0);
        LEFT_RED    = (ledstate[LEFT].red_state ? 1 : 0);
        RIGHT_GREEN = (ledstate[RIGHT].green_state ? 1 : 0);
        RIGHT_RED   = (ledstate[RIGHT].red_state ? 1 : 0);
    }
}
