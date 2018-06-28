/*
     ____ __     ____   ___    ____ __         (((((()
    | |_  \ \  /   ) ) | |  ) | |_  \ \  /  \(@)- /
    |_|__  \_\/  __)_) |_|_/  |_|__  \_\/   /(@)- \
                                               ((())))
 *//**
 *  \file   tach_encoder.h
 *  \brief  Common Header File for tacho motor encoders
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

#ifndef TACHO_ENCODER_H_
#define TACHO_ENCODER_H_

#include <stdbool.h>
#include <stdint.h>

#include <am18xx/sys_gpio.h>


/** @addtogroup pru */
/*@{*/

/** @defgroup tacho-encoder PRU Tacho Encoder Routines
 *
 * The PRU Tacho Encoder Routines keeps track of the encoder counts for the given tacho motors.
 * FIXME: Add more details
 *
 */

/*@{*/

#define INTA0 GPIO.IN_DATA45_bit.GP5P11     // GPIO 5[11]
#define INTB0 GPIO.IN_DATA45_bit.GP5P8      // GPIO 5[8]
#define INTC0 GPIO.IN_DATA45_bit.GP5P13     // GPIO 5[13]
#define INTD0 GPIO.IN_DATA67_bit.GP6P9      // GPIO 6[9]
#define  DIRA GPIO.IN_DATA01_bit.GP0P4      // GPIO 0[4]
#define  DIRB GPIO.IN_DATA23_bit.GP2P9      // GPIO 2[9]
#define  DIRC GPIO.IN_DATA23_bit.GP3P14     // GPIO 3[14]
#define  DIRD GPIO.IN_DATA23_bit.GP2P8      // GPIO 2[8]

/* Used by tachoencoder_readallmotors() encoder_value */
#define ENCODER_INTX0_MASK 0x02				// Bitmask for INTx0 setting
#define ENCODER_DIRX_MASK  0x01				// Bitmask for DIRx setting

#define ENCODERVEC_EVENT_SHIFT 2
#define ENCODERVEC_EVENT_MASK  (ENCODER_INTX0_MASK | ENCODER_DIRX_MASK)

/* Used by tachoencoder_readallmotors() encodervec_t */
#define ENCODER_INTD0_MASK 0x80				// Bitmask for INTD0 setting
#define ENCODER_DIRD_MASK  0x40				// Bitmask for DIRD setting
#define ENCODER_INTC0_MASK 0x20				// Bitmask for INTC0 setting
#define ENCODER_DIRC_MASK  0x10				// Bitmask for DIRC setting
#define ENCODER_INTB0_MASK 0x08				// Bitmask for INTB0 setting
#define ENCODER_DIRB_MASK  0x04				// Bitmask for DIRB setting
#define ENCODER_INTA0_MASK 0x02				// Bitmask for INTA0 setting
#define ENCODER_DIRA_MASK  0x01				// Bitmask for DIRA setting


typedef uint32_t encoder_count_t;
typedef uint32_t timer_t;
typedef uint8_t  encodervec_t;

typedef enum encoder_value_t {
	ENC_00 = 0x00, 												// Encoder position and State
	ENC_01 = 0x01, 												// Encoder position and State
	ENC_11 = 0x03, 												// Encoder position and State
	ENC_10 = 0x02, 												// Encoder position and State
	ENC_IDLE = 0xFF												// Encoder State
} encoder_value;

#define EVENTVEC_RESETMASK 0x00

typedef enum motor_identifier_t {
    MOTOR0 = 0, MOTOR1, MOTOR2, MOTOR3, MAX_TACHO_MOTORS      	// MAX_TACHO_MOTORS used to count number of motors
} motor_identifier;

#define MOTOR_UNUSED MAX_TACHO_MOTORS							// Needed by leddebug

typedef enum encoder_direction_t {
    UNKNOWN = 0,
    FORWARD = 1,												// Clockwise
    REVERSE = -1												// Anti-clockwise
} encoder_direction;

typedef struct {
    encoder_direction   dir;
    encoder_value		state;
    encoder_count_t     count;

} encoder_struct;


/** tachoencoder_extractmotorevent
 *
 * Internal routine
 *
 * Retrieve the encoder value for the given motor from the encoder event vector
 *
 * @param motor: motor_identifier
 * @param  event: encodervec_t
 * @return encoder_value
 *
 */
encoder_value tachoencoder_extractmotorevent(motor_identifier motor, encodervec_t event);


/** tachoencoder_readallmotors
 *
 * Internal routine
 *
 * Retrieve the event vector (encoder values from all motors)
 *
 * @param  None
 * @return event vector: encodervec_t
 *
 */
encodervec_t tachoencoder_readallmotors();

/** tachoencoder_updatemotorstate
 *
 * Internal routine
 *
 * Update tacho encoder state machine for given motor
 *
 * @param motor: motor_identifier
 * @param encval: encoder_value
 *
 * @return dir: encoder_direction
 *
 */
encoder_direction tachoencoder_updatemotorstate(motor_identifier motor, encoder_value encval);

/** tachoencoder_reset
 *
 * Resets the encoder config for the given motor
 *
 * @param motor: motor_identifier
 * @param  reset_count: bool, Reset encoder counters if true
 * @return Pointer to encoder_struct
 *
 */
encoder_struct *tachoencoder_reset(motor_identifier motor, bool reset_count);

/** tachoencoder_getdircount
 *
 * Retrieves tacho encoder direction and count for given motor
 *
 * @param motor: motor_identifier
 * @param count: encoder_count_t [Output]
 *
 * @return dir: encoder_direction
 *
 */
encoder_direction tachoencoder_getdircount(motor_identifier motor, encoder_count_t *count);

/** tachoencoder_hasnewevent
 *
 * Check if the eventvec has changed state
 *
 * This routine is non-blocking
 *
 * @param *eventvec: encodervec_t
 * @return status: bool (true if new event detected)
 *
 */
bool tachoencoder_hasnewevent(encodervec_t *eventvec);

/** tachoencoder_updateencoderstate
 *
 * Update the encoder state for all active motors due to new event trigger
 *
 * @param neweventvec: encodervec_t (Vector of encoder values for all motors)
 * @param timestamp: timer_t (Capture Event Timer value)
 * @return None
 *
 */
void tachoencoder_updateencoderstate(encodervec_t neweventvec, timer_t timestamp);

/*@}*/
/*@}*/


#endif /* TACHO_ENCODER_H_ */
