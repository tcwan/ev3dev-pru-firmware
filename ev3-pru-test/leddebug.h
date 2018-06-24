/*
     ____ __     ____   ___    ____ __         (((((()
    | |_  \ \  /   ) ) | |  ) | |_  \ \  /  \(@)- /
    |_|__  \_\/  __)_) |_|_/  |_|__  \_\/   /(@)- \
                                               ((())))
 *//**
 *  \file   leddebug.h
 *  \brief  Debug support routine Header File for tacho motor encoders
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

#ifndef LEDDEBUG_H_
#define LEDDEBUG_H_

#include <stdbool.h>
#include <am18xx/sys_gpio.h>
#include "tacho-encoder.h"

/** @addtogroup pru */
/*@{*/

/** @defgroup leddebug PRU Debug Support Routines
 *
 * The PRU Debug Support Routines display encoder position changes via the LEDs.
 * Green is for forward motion, while Red is for reverse motion.
 * The LED toggling depends on the flashing_interval value.
 * e.g. set flashing_interval to 1 to toggle the LED for each debug_count value change,
 *      while flashing_interval of 10 toggles the LED when the debug_count is a multiple of 10.
 *
 */

/*@{*/


#define DIODE0 GPIO.OUT_DATA67_bit.GP6P13           // Left Red    GPIO 6[13]
#define DIODE1 GPIO.OUT_DATA67_bit.GP6P7            // Left Green  GPIO 6[7]
#define DIODE2 GPIO.OUT_DATA67_bit.GP6P14           // Right Green GPIO 6[14]
#define DIODE3 GPIO.OUT_DATA67_bit.GP6P12           // Right Red   GPIO 6[12]

#define LEFT_RED DIODE0
#define LEFT_GREEN DIODE1
#define RIGHT_RED DIODE3
#define RIGHT_GREEN DIODE2

typedef long debug_count_t;

typedef enum led_identifier_t {
    LEFT = 0, RIGHT, MAX_LEDS      // MAX_LEDS used to count number of display LEDs

} led_identifier;

typedef struct {
    encoder_direction   dir;
    debug_count_t       debug_count;
    debug_count_t       flashing_interval;
    bool                green_state;
    bool                red_state;
} leddebug_state;

/** leddebug_setleds
 *
 * Internal routine
 *
 * Actually program the LEDs to required settings
 *
 * @param None
 * @return None
 *
 */
void leddebug_setleds();


/** leddebug_init
 *
 * Initialize leddebug
 *
 * @param flashing_interval: interval to toggle LED
 * @return None
 *
 */
void leddebug_init(debug_count_t flashing_interval);

/** leddebug_assignmotors
 *
 * Assign motors to LEDs
 *
 * @param left: motor identifier assigned to left LED
 * @param right: motor identifier assigned to right LED
 *
 * If LED is not used, pass MOTOR_UNUSED as the parameter
 * @return None
 *
 */
void leddebug_assignmotors(motor_identifier left, motor_identifier right);

/** leddebug_update
 *
 * Update debug state
 *
 * @param side: motor identifier
 * @param dir: encoder motion direction
 * @return None
 *
 */
void leddebug_update(motor_identifier side, encoder_direction dir);

/*@}*/
/*@}*/

#endif /* LEDDEBUG_H_ */
