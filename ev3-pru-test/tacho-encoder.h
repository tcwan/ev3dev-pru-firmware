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

/** @addtogroup pru */
/*@{*/

/** @defgroup tacho-encoder PRU Tacho Encoder Routines
 *
 * The PRU Tacho Encoder Routines keeps track of the encoder counts for the given tacho motors.
 * FIXME: Add more details
 *
 */

/*@{*/

#define INTA0 GPIO.IN_DATA45_bit.GP5P11      // GPIO 5[11]
#define INTB0 GPIO.IN_DATA45_bit.GP5P8       // GPIO 5[8]
#define INTC0 GPIO.IN_DATA45_bit.GP5P13      // GPIO 5[13]
#define INTD0 GPIO.IN_DATA67_bit.GP6P9       // GPIO 6[9]
#define  DIRA GPIO.IN_DATA01_bit.GP0P4       // GPIO 0[4]
#define  DIRB GPIO.IN_DATA23_bit.GP2P9       // GPIO 2[9]
#define  DIRC GPIO.IN_DATA23_bit.GP3P14      // GPIO 3[14]
#define  DIRD GPIO.IN_DATA23_bit.GP2P8       // GPIO 2[8]

typedef long encoder_count_t;

typedef enum output_port_t {
    outA, outB, outC, outD
} output_port;

typedef enum motor_identifier_t {
    LEFT = 0, RIGHT, MAX_TACHO_MOTORS      // MAX_TACHO_MOTORS used to count number of motors
} motor_identifier;


typedef enum encoder_direction_t {
    UNKNOWN = 0,
    FORWARD = 1,
    REVERSE = -1
} encoder_direction;

typedef struct {
    output_port         port;
    motor_identifier    side;
    encoder_direction   dir;
    encoder_count_t     count;

} encoder_struct;

/** Initialize tacho encoder
 *
 * FIXME: Assumes two ports only
 *
 * @param leftmotor: output_port, rightmotor: output_port
 * @return None
 *
 */

void init_tachoencoder(output_port leftmotor, output_port rightmotor);


/*@}*/
/*@}*/


#endif /* TACHO_ENCODER_H_ */
