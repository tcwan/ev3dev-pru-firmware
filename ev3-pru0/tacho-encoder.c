/*
     ____ __     ____   ___    ____ __         (((((()
    | |_  \ \  /   ) ) | |  ) | |_  \ \  /  \(@)- /
    |_|__  \_\/  __)_) |_|_/  |_|__  \_\/   /(@)- \
                                               ((())))
 *//**
 *  \file   tacho-encoder.c
 *  \brief  Encoder capture routines for EV3 tacho motors
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

#include <string.h>
#include <stdbool.h>
#include "tacho-encoder.h"

static encodervec_t lasteventvec = EVENTVEC_RESETMASK;
static encoder_struct encoder_config[MAX_TACHO_MOTORS];

/* Internal Routines */

encoder_value tachoencoder_extractmotorevent(motor_identifier motor, encodervec_t event) {

	// This routine is used to convert the event vector obtained by tachoencoder_readallmotors()
	// to an event for a specific motor
	//

    encoder_value motorevent = ENC_00;                           // Arbitrary return value

    motorevent = (encoder_value) ((event >> (ENCODERVEC_EVENT_SHIFT * motor)) & ENCODERVEC_EVENT_MASK);

	return motorevent;
}

encodervec_t tachoencoder_readallmotors() {
	// This routine is independent of tachoencoder_readmotor() for performance reasons
	// It will get the inputs from all motors and return it as a single vector for encoder
	// change detection in the main polling loop

	volatile encodervec_t allinputs;

	allinputs = (INTA0 ? ENCODER_INTA0_MASK : 0) | (DIRA ? ENCODER_DIRA_MASK : 0) | \
				(INTB0 ? ENCODER_INTB0_MASK : 0) | (DIRB ? ENCODER_DIRB_MASK : 0) | \
				(INTC0 ? ENCODER_INTC0_MASK : 0) | (DIRC ? ENCODER_DIRC_MASK : 0) | \
				(INTD0 ? ENCODER_INTD0_MASK : 0) | (DIRD ? ENCODER_DIRD_MASK : 0);

	return allinputs;
}

encoder_direction tachoencoder_updatemotorstate(motor_identifier motor, encoder_value encval) {
	// State machine for encoder processing
	//
	// [State Machine Diagram](State-Machine/Quadrature-Encoder-States-Simplified.eps)
	//
	// FIXME: Provide state machine description

	encoder_value currstate;
	encoder_value newstate = ENC_IDLE;				// Default transition (error condition)
	encoder_direction newdir = UNKNOWN;

	// encval must be one of (ENC_00, ENC_01, ENC_10, ENC_11) to be a valid input
	// Otherwise we just ignore and return
	if ((unsigned) encval > ENC_11)
		return UNKNOWN;

	if ((unsigned) motor < MAX_TACHO_MOTORS) {
	    encoder_struct *encoder_configptr = &(encoder_config[motor]);

	    currstate = encoder_configptr->state;

		if (encval != currstate) {

		    switch (currstate) {
				case ENC_00:
					if (encval == ENC_01) {
						newstate = ENC_01;
						newdir = REVERSE;
					} else if (encval == ENC_10) {
						newstate = ENC_10;
						newdir = FORWARD;
					}
					break;

				case ENC_01:
					if (encval == ENC_11) {
						newstate = ENC_11;
						newdir = REVERSE;
					} else if (encval == ENC_00) {
						newstate = ENC_00;
						newdir = FORWARD;
					}
					break;

				case ENC_11:
					if (encval == ENC_10) {
						newstate = ENC_10;
						newdir = REVERSE;
					} else if (encval == ENC_01) {
						newstate = ENC_01;
						newdir = FORWARD;
					}
					break;

				case ENC_10:
					if (encval == ENC_00) {
						newstate = ENC_00;
						newdir = REVERSE;
					} else if (encval == ENC_11) {
						newstate = ENC_11;
						newdir = FORWARD;
					}
					break;

				case ENC_IDLE:
				default:
					newstate = encval;
					newdir = UNKNOWN;
					break;
			}
		} else
			newstate = currstate;									// No state change

		// Assume that newstate has valid values here
		if (newstate == ENC_IDLE) {
			// Somehow we didn't get an expected transition
		    encoder_configptr->state = ENC_IDLE;					// Goto Idle
		    newdir = encoder_configptr->dir = UNKNOWN;				// Reset direction indicator,  but don't update event count
		} else if (newstate != currstate) {
				// Update state and encoder count only if we have changed states
		        encoder_configptr->state = newstate;
		        encoder_configptr->dir = newdir;
		        encoder_configptr->count += newdir;
		} else {
			// We are still in the same state
			// Do nothing
		}
	}

	return newdir;
}

/* Public Routines */

encoder_struct *tachoencoder_reset(motor_identifier motor, bool reset_count) {
    encoder_struct *encoder_configptr = &(encoder_config[motor]);

    encoder_configptr->dir = UNKNOWN;
    encoder_configptr->state = ENC_IDLE;
    if (reset_count)
        encoder_configptr->count = 0;

    return encoder_configptr;               // Reduce array address conversion overhead
}

encoder_direction tachoencoder_getdircount(motor_identifier motor, encoder_count_t *count) {

    encoder_direction retval = UNKNOWN;

    if ((unsigned) motor < MAX_TACHO_MOTORS) {
        encoder_struct *encoder_configptr = &(encoder_config[motor]);
        retval = encoder_configptr->dir;
        *count = encoder_configptr->count;
    }
    return retval;

}

bool tachoencoder_hasnewevent(encodervec_t *eventvec) {
	*eventvec = tachoencoder_readallmotors();			// Get all input port encoder values (active or otherwise)
	if (*eventvec != lasteventvec) {
		// state change detected
		lasteventvec = *eventvec;
		return true;
	} else {
		return false;									// no change in encoder input values
	}

}

void tachoencoder_updateencoderstate(encodervec_t neweventvec, timer_t timestamp) {

    // The timestamp is ignored for now since we're recording encoder counts at a fixed interval
	int i;
	encoder_value motorevent;					// encoder value for port event

	for (i = 0; i < MAX_TACHO_MOTORS; i++) {

	    motorevent = tachoencoder_extractmotorevent((motor_identifier) i, neweventvec);
        tachoencoder_updatemotorstate((motor_identifier) i, motorevent);
	}

}
