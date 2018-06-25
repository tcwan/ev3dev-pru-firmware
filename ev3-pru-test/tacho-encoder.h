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

/* Used by tachoencoder_readport() encoder_value */
#define ENCODER_INTX0_MASK 0x02				// Bitmask for INTx0 setting
#define ENCODER_DIRX_MASK  0x01				// Bitmask for DIRx setting

/* Used by tachoencoder_readallports() encodervec_t */
#define ENCODER_INTD0_MASK 0x80				// Bitmask for INTD0 setting
#define ENCODER_DIRD_MASK  0x40				// Bitmask for DIRD setting
#define ENCODER_INTC0_MASK 0x20				// Bitmask for INTC0 setting
#define ENCODER_DIRC_MASK  0x10				// Bitmask for DIRC setting
#define ENCODER_INTB0_MASK 0x08				// Bitmask for INTB0 setting
#define ENCODER_DIRB_MASK  0x04				// Bitmask for DIRB setting
#define ENCODER_INTA0_MASK 0x02				// Bitmask for INTA0 setting
#define ENCODER_DIRA_MASK  0x01				// Bitmask for DIRA setting

#define ENCODER_PORTAVEC_MASK (encodervec_t) (ENCODER_INTA0_MASK | ENCODER_DIRA_MASK)
#define ENCODER_PORTBVEC_MASK (encodervec_t) (ENCODER_INTB0_MASK | ENCODER_DIRB_MASK)
#define ENCODER_PORTCVEC_MASK (encodervec_t) (ENCODER_INTC0_MASK | ENCODER_DIRC_MASK)
#define ENCODER_PORTDVEC_MASK (encodervec_t) (ENCODER_INTD0_MASK | ENCODER_DIRD_MASK)

#define ENCODER_PORTAVEC_SHIFT (encodervec_t) 0
#define ENCODER_PORTBVEC_SHIFT (encodervec_t) 2
#define ENCODER_PORTCVEC_SHIFT (encodervec_t) 4
#define ENCODER_PORTDVEC_SHIFT (encodervec_t) 6


typedef long encoder_count_t;
typedef uint32_t timer_t;
typedef uint8_t  encodervec_t;

typedef enum encoder_value_t {
	ENC_00 = 0x00, 												// Encoder position and State
	ENC_01 = 0x01, 												// Encoder position and State
	ENC_11 = 0x11, 												// Encoder position and State
	ENC_10 = 0x10, 												// Encoder position and State
	ENC_IDLE = 0xFF												// Encoder State
} encoder_value;

#define EVENTVEC_RESETMASK 0x00

typedef enum output_port_t {
	PORT_UNUSED = 0, outA, outB, outC, outD, MAX_PORTS			// MAX_PORTS used for sanity checks
} output_port;

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
    output_port         port;
    encoder_direction   dir;
    encoder_value		state;
    encoder_count_t     count;

} encoder_struct;

/* Encoder History Data Storage
 *
 * This uses the AM1808 On Chip RAM (128K) as a ring buffer to store the encoder event history
 *
 */

typedef long event_index_t;

typedef struct {
	bool			updating;							// Semaphore: TRUE if PRU is updating event history buffer (modified only by PRU)
	event_index_t	ringbuf_maxitems;					// Maximum number of items in ringbuf
	event_index_t	lastevent_index;					// Index of most recently captured event
	timer_t			lastevent_time;						// Timestamp of raw speed calculation (and last event update)
	encoder_count_t raw_speed[MAX_TACHO_MOTORS];		// Counter Difference (per window) between current entry and oldest entry in history buffer

} encoder_history_struct;

typedef struct {
	encoder_count_t count[MAX_TACHO_MOTORS];
} encoder_event_struct;


// FIXME: The On Chip RAM needs to  be shared with the PRU SUART buffers
#define ON_CHIP_RAM_START ((volatile encoder_history_struct *)(0x80000000))
#define ON_CHIP_RAM_SIZE  0x20000
#define EVENT_RINGBUF_START  (volatile encoder_event_struct *) ((ON_CHIP_RAM_START + sizeof(encoder_event_struct)))
#define RINGBUF_MAXITEMS ((ON_CHIP_RAM_SIZE - sizeof(encoder_history_struct)) / (MAX_TACHO_MOTORS * sizeof(encoder_event_struct)))

/** tachoencoder_extractportevent
 *
 * Internal routine
 *
 * Retrieve the encoder value for the given port from the encoder event vector
 *
 * @param  port: output_port
 * @param  event: encodervec_t
 * @return encoder_value
 *
 */
encoder_value tachoencoder_extractportevent(output_port port, encodervec_t event);

/** tachoencoder_readport
 *
 * Internal routine
 *
 * Retrieve the encoder value from the given port
 *
 * @param  port: output_port
 * @return encoder_value
 *
 */
encoder_value tachoencoder_readport(output_port port);

/** tachoencoder_readallports
 *
 * Internal routine
 *
 * Retrieve the event vector (encoder values from all output port), regardless of whether they are active or not
 *
 * @param  None
 * @return event vector: encodervec_t
 *
 */
encodervec_t tachoencoder_readallports();

/** tachoencoder_updatemotorestate
 *
 * Internal routine
 *
 * Update tacho encoder state machine for given motor
 *
 * @param motor: motor_identifier
 * @param encval: encoder_value
 *
 * @return None
 *
 */
void tachoencoder_updatemotorestate(motor_identifier motor, encoder_value encval);

/** tachoencoder_getdircount
 *
 * Internal routine (for debugging)
 *
 * Retrieves tacho encoder direction and count for given motor
 *
 * @param motor: motor_identifier
 * @param dir: encoder_direction
 * @param count: encoder_count_t
 *
 * @return Output Port (0: unused, non-zero: port number)
 *
 */
output_port tachoencoder_getdircount(motor_identifier motor, encoder_direction *dir, encoder_count_t *count);

/** tachoencoder_init
 *
 * Initialize tacho encoder
 *
 * All motor encoder counts are zeroed
 *
 * @param motor0_port, motor1_port, motor2_port, motor3_port: output_port
 *
 * Unused motors should pass PORT_UNUSED as parameter
 * Note: The routine doesn't prevent duplicate port ids
 *
 * @return None
 *
 */
void tachoencoder_init(output_port motor0_port, output_port motor1_port, output_port motor2_port, output_port motor3_port);

/** tachoencoder_activatemotor
 *
 * Activate motor encoder tracking for the given motor attached to the given port
 *
 * @param motor: motor_identifier
 * @param  port: output_port
 * @return None
 *
 * Note: The routine doesn't prevent duplicate port ids
 *
 */
void tachoencoder_activatemotor(motor_identifier motor, output_port port);

/** tachoencoder_deactivatemotor
 *
 * Deactivate motor encoder tracking for the given port
 *
 * @param motor: motor_identifier
 * @return None
 *
 */
void tachoencoder_deactivatemotor(motor_identifier motor);

/** tachoencoder_reset
 *
 * Reset tacho encoder history buffers for all motors
 *
 * Motor activation status remains unchanged.
 *
 * This routine does not clear the per-motor encoder count
 * (All entries in history buffer is set to current per-motor encoder count)
 *
 * @param None
 * @return None
 *
 */
void tachoencoder_reset();


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

/** tachoencoder_updateteventbuffer
 *
 * Update the event history buffer for given index with the tachometer counts for all motors
 * and the current raw speed (delta count between current and oldest readings) for each motor
 *
 * @param index: event_index_t (Capture Event index value)
 * @param timestamp: timer_t (Capture Event Timer value)
 * @return None
 *
 */
void tachoencoder_updateteventbuffer(event_index_t index, timer_t timestamp);


/*@}*/
/*@}*/


#endif /* TACHO_ENCODER_H_ */
