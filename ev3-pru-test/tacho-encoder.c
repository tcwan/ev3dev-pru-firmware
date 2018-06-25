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

static encoder_struct encoder_config[MAX_TACHO_MOTORS];

static volatile encoder_history_struct *encoder_history_config = ON_CHIP_RAM_START;
static volatile encoder_event_struct *encoder_event_buffer = EVENT_RINGBUF_START;
static encodervec_t active_encoders;
static encodervec_t lasteventvec;

/* Private utility functions */
void _set_semaphore() {
    // Set semaphore
    encoder_history_config->updating = true;
}

void _release_semaphore() {
    // Release semaphore
    encoder_history_config->updating = false;
}

encoder_struct *_reset_encoder_config(motor_identifier motor, bool reset_count) {
    encoder_struct *encoder_configptr = &(encoder_config[motor]);

    encoder_configptr->dir = UNKNOWN;
    encoder_configptr->state = ENC_IDLE;
    if (reset_count)
        encoder_configptr->count = 0;

    return encoder_configptr;               // Reduce array address conversion overhead
}

encodervec_t _port_to_encodermask(output_port port) {
	switch (port) {
	case outA:
		return ENCODER_PORTAVEC_MASK;
		// break;
	case outB:
		return ENCODER_PORTBVEC_MASK;
		// break;
	case outC:
		return ENCODER_PORTCVEC_MASK;
		// break;
	case outD:
		return ENCODER_PORTDVEC_MASK;
		// break;
	case PORT_UNUSED:
	default:
		return 0;
		// break;
	}
}

/* Internal Routines */
encoder_value tachoencoder_extractportevent(output_port port, encodervec_t event) {

	// This routine is used to convert the event vector obtained by tachoencoder_readallports()
	// to an event for a specific port
	//

# if 0
	switch (port) {
	case outA:
		portevent = (encoder_value) ((event & ENCODER_PORTAVEC_MASK) >> ENCODER_PORTAVEC_SHIFT);
		break;
	case outB:
		portevent = (encoder_value) ((event & ENCODER_PORTBVEC_MASK) >> ENCODER_PORTBVEC_SHIFT);
		break;
	case outC:
		portevent = (encoder_value) ((event & ENCODER_PORTCVEC_MASK) >> ENCODER_PORTCVEC_SHIFT);
		break;
	case outD:
		portevent = (encoder_value) ((event & ENCODER_PORTDVEC_MASK) >> ENCODER_PORTDVEC_SHIFT);
		break;
	case PORT_UNUSED:
	default:
		portevent = ENC_00;					// Arbitrary return value
		break;
	}
#endif

    encoder_value portevent = ENC_00;                           // Arbitrary return value

	if ((port != PORT_UNUSED) && (port < MAX_PORTS)) {
        output_port i = outA;
        while (i < port) {
            event = event >> ENCODERVEC_EVENT_SHIFT;
            i++;
        }
        portevent = (encoder_value) (event & ENCODERVEC_EVENT_MASK);
	}
	return portevent;
}

#if 0
encoder_value tachoencoder_readport(output_port port) {

	// This routine will read the encoder values from a single port.
	// It is not expected to be used much since we are interested in motor synchronization
	// It is more efficient to just use tachoencoder_readallports() to check multiple ports

	encoder_value input;

	switch (port) {
	case outA:
		input = (encoder_value) ((INTA0 ? ENCODER_INTX0_MASK : 0) | (DIRA ? ENCODER_DIRX_MASK : 0));
		break;
	case outB:
		input = (encoder_value) ((INTB0 ? ENCODER_INTX0_MASK : 0) | (DIRB ? ENCODER_DIRX_MASK : 0));
		break;
	case outC:
		input = (encoder_value) ((INTC0 ? ENCODER_INTX0_MASK : 0) | (DIRC ? ENCODER_DIRX_MASK : 0));
		break;
	case outD:
		input = (encoder_value) ((INTD0 ? ENCODER_INTX0_MASK : 0) | (DIRD ? ENCODER_DIRX_MASK : 0));
		break;
	case PORT_UNUSED:
	default:
		input = ENC_00;					// Arbitrary return value
		break;
	}

	return input;
}
#endif


encodervec_t tachoencoder_readallports() {
	// This routine is independent of tachoencoder_readport() for performance reasons
	// It will get the inputs from all ports and return it as a single vector for encoder
	// change detection in the main polling loop

	encodervec_t allinputs;

	allinputs = (INTA0 ? ENCODER_INTA0_MASK : 0) | (DIRA ? ENCODER_DIRA_MASK : 0) | \
				(INTB0 ? ENCODER_INTB0_MASK : 0) | (DIRB ? ENCODER_DIRB_MASK : 0) | \
				(INTC0 ? ENCODER_INTC0_MASK : 0) | (DIRC ? ENCODER_DIRC_MASK : 0) | \
				(INTD0 ? ENCODER_INTD0_MASK : 0) | (DIRD ? ENCODER_DIRD_MASK : 0);

	return allinputs;
}

void tachoencoder_updatmotorstate(motor_identifier motor, encoder_value encval) {
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
		return;



	if ((unsigned) motor < MAX_TACHO_MOTORS) {
	    encoder_struct *encoder_configptr = &(encoder_config[motor]);

	    currstate = encoder_configptr->state;

		if (encval != currstate) {

			switch (currstate) {
				case ENC_00:
					if (encval == ENC_01) {
						newstate = ENC_01;
						newdir = FORWARD;
					} else if (encval == ENC_10) {
						newstate = ENC_10;
						newdir = REVERSE;
					}
					break;

				case ENC_01:
					if (encval == ENC_11) {
						newstate = ENC_11;
						newdir = FORWARD;
					} else if (encval == ENC_00) {
						newstate = ENC_00;
						newdir = REVERSE;
					}
					break;

				case ENC_11:
					if (encval == ENC_10) {
						newstate = ENC_10;
						newdir = FORWARD;
					} else if (encval == ENC_01) {
						newstate = ENC_01;
						newdir = REVERSE;
					}
					break;

				case ENC_10:
					if (encval == ENC_00) {
						newstate = ENC_00;
						newdir = FORWARD;
					} else if (encval == ENC_11) {
						newstate = ENC_11;
						newdir = REVERSE;
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
		    encoder_configptr->dir = UNKNOWN;					// Reset direction indicator,  but don't update event count
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
}

output_port tachoencoder_getdircount(motor_identifier motor, encoder_direction *dir, encoder_count_t *count) {

	output_port retval = PORT_UNUSED;

	if ((unsigned) motor < MAX_TACHO_MOTORS) {
	    encoder_struct *encoder_configptr = &(encoder_config[motor]);
		retval = encoder_configptr->port;
		*dir = encoder_configptr->dir;
		*count = encoder_configptr->count;
	}
	return retval;

}


/* Public Routines */
void tachoencoder_init(event_index_t maxitems, output_port motor0_port, output_port motor1_port, output_port motor2_port, output_port motor3_port) {


	// Initialize Encoder History struct and history buffer (assumed contiguous)
    memset((void *) encoder_history_config, 0, sizeof(encoder_history_struct) + sizeof(encoder_event_struct) * maxitems);
    _set_semaphore();                           // Semaphore was zeroed out by memset()
    encoder_history_config->ringbuf_maxitems = maxitems;
#if 0
    // Merge buffer clearing code into one memset() call
	// Zero history buffer
	memset((void *) encoder_event_buffer, 0, sizeof(encoder_event_struct) * maxitems);
#endif

	// Initialize last event vector and active encoder bitmask
	lasteventvec = EVENTVEC_RESETMASK;
	active_encoders = 0;

	// Initialize per-motor Encoder Settings
	int i;
	for (i = 0; i < MAX_TACHO_MOTORS; i++) {
		// Initialize local state
		// encoder_config[i].port = PORT_UNUSED;			// Assumes that the port initialization step below configures all motors
		_reset_encoder_config((motor_identifier) i, true);

#if 0
		// already cleared by memset()
		encoder_history_config->raw_speed[i] = 0;			// Clear raw_speed variable for history buffer
#endif
	}

	// Activate Encoders and update bitmask
	// Assumes 4 motors only
	output_port valid_port;

	valid_port = ((unsigned) motor0_port < MAX_PORTS) ? motor0_port : PORT_UNUSED;
	encoder_config[MOTOR0].port = valid_port;
	active_encoders |= _port_to_encodermask(valid_port);

	valid_port = ((unsigned) motor1_port < MAX_PORTS) ? motor1_port : PORT_UNUSED;
	encoder_config[MOTOR1].port = valid_port;
	active_encoders |= _port_to_encodermask(valid_port);

	valid_port = ((unsigned) motor2_port < MAX_PORTS) ? motor2_port : PORT_UNUSED;
	encoder_config[MOTOR2].port = valid_port;
	active_encoders |= _port_to_encodermask(valid_port);

	valid_port = ((unsigned) motor3_port < MAX_PORTS) ? motor3_port : PORT_UNUSED;
	encoder_config[MOTOR3].port = valid_port;
	active_encoders |= _port_to_encodermask(valid_port);

    _release_semaphore();
}

void tachoencoder_activatemotor(motor_identifier motor, output_port port) {
	if (((unsigned) motor < MAX_TACHO_MOTORS) && ((unsigned) port < MAX_PORTS)){

        encoder_struct *encoder_configptr;
		active_encoders |= _port_to_encodermask(port);				// Enable active motor in Bitmask

		encoder_configptr = _reset_encoder_config(motor, true);
        encoder_configptr->port = port;
	}
}

void tachoencoder_deactivatemotor(motor_identifier motor) {
	if ((unsigned) motor < MAX_TACHO_MOTORS) {

	    encoder_struct *encoder_configptr;
	    output_port oldport;

        encoder_configptr = _reset_encoder_config(motor, true);
		oldport = encoder_configptr->port;
		active_encoders &= ~(_port_to_encodermask(oldport));			// Disable inactive motor in Bitmask
		encoder_configptr->port = PORT_UNUSED;
	}
}

void tachoencoder_reset() {
    _set_semaphore();

	// Reset last event vector
	lasteventvec = EVENTVEC_RESETMASK;

	// Reset history using current tachometer readings
	encoder_event_struct latestevent;

	int i;
	for (i = 0; i < MAX_TACHO_MOTORS; i++) {

        encoder_struct *encoder_configptr;

        // Reset encoder state
	    encoder_configptr = _reset_encoder_config((motor_identifier) i, false);

		// initialize event history buffer with last count for each motor
		latestevent.count[i] = encoder_configptr->count;
	}

	for (i = 0; i < encoder_history_config->ringbuf_maxitems; i++) {
		memcpy((void *)&(encoder_event_buffer[i]), &latestevent, sizeof(encoder_event_struct));
	}
    _release_semaphore();

}


bool tachoencoder_hasnewevent(encodervec_t *eventvec) {
	*eventvec = tachoencoder_readallports();			// Get all input port encoder values (active or otherwise)
	*eventvec &= active_encoders;						// Keep only active encoder values
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
	encoder_value portevent;					// encoder value for port event

	for (i = 0; i < MAX_TACHO_MOTORS; i++) {
		if ((encoder_config[i].port != PORT_UNUSED) && (encoder_config[i].port < MAX_PORTS)) {

			portevent = tachoencoder_extractportevent(encoder_config[i].port, neweventvec);
			tachoencoder_updatemotorstate((motor_identifier) i, portevent);
		}
	}

}


void tachoencoder_updateteventbuffer(event_index_t index, timer_t timestamp) {

	volatile encoder_event_struct *itemptr;		// pointer to event item
	int i;

	_set_semaphore();

	// Point to old event data at index
	itemptr = &(encoder_event_buffer[index]);

	for (i = 0; i < MAX_TACHO_MOTORS; i++) {

        encoder_struct *encoder_configptr = &(encoder_config[i]);
        output_port port = encoder_configptr->port;
        encoder_count_t new_count = encoder_configptr->count;
        volatile encoder_count_t *buffer_index_countptr = &(itemptr->count[i]);
        volatile encoder_count_t *raw_speedptr = &(encoder_history_config->raw_speed[i]);

		if ((port != PORT_UNUSED) && (port < MAX_PORTS)) {
			// Calculate raw speed = (new count - old count) [per event buffer window interval]
			*raw_speedptr = new_count - *buffer_index_countptr;
			// Store new event into buffer
			*buffer_index_countptr = new_count;
		} else {
		    *raw_speedptr = 0;
			*buffer_index_countptr = 0;

		}
	}

	// Update timestamps, etc. in encoder_history_config
	encoder_history_config->lastevent_index = index;
	encoder_history_config->lastevent_time = timestamp;

	_release_semaphore();
}

