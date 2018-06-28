/*
 * main.c
 */

#include <stdbool.h>
#include <stdint.h>

#include <pru_rpmsg.h>
#include <pru_virtio_config.h>

#include <am18xx/pru_ctrl.h>
#include <am18xx/pru_intc.h>
#include <am18xx/sys_gpio.h>
#include <am18xx/sys_timer.h>

#include "resource_table.h"
#include "tacho-encoder.h"

#define TRIGGER_PERIOD_MS 10

/* from Linux */

enum ev3_pru_tacho_msg_type {
	EV3_PRU_TACHO_MSG_START,
	EV3_PRU_TACHO_MSG_STOP,
	EV3_PRU_TACHO_MSG_REQ_ONE,
	EV3_PRU_TACHO_MSG_UPDATE,
};

enum ev3_pru_tacho_iio_channel {
	EV3_PRU_IIO_CH_TIMESTAMP,
	EV3_PRU_IIO_CH_TACHO_A,
	EV3_PRU_IIO_CH_TACHO_B,
	EV3_PRU_IIO_CH_TACHO_C,
	EV3_PRU_IIO_CH_TACHO_D,
	NUM_EV3_PRU_IIO_CH
};

struct ev3_pru_tacho_msg {
	uint32_t type;
	uint32_t value[NUM_EV3_PRU_IIO_CH];
};

/* end Linux */


// system events to/from ARM

// PRU0
#define HOST_INT	(uint32_t)(1 << 30)	// host interrupt 0
#define EVENT_FROM_ARM	32
#define EVENT_TO_ARM	33

// PRU1
//#define HOST_INT	(uint32_t)(1 << 31)	// host interrupt 1
//#define EVENT_FROM_ARM	34
//#define EVENT_TO_ARM	35

// clock is 24MHz, so 24000 ticks/msec
#define TRIGGER_PERIOD_TICKS (TRIGGER_PERIOD_MS * 24000)

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
        last_timerexpiry += timer_period;       // advance timer expiry timestamp
        return true;
    }
    else
        return false;
}

volatile uint32_t register __R31;

// Only have 512B local RAM for stack/data, so this won't fit there. Use shared RAM instead.
volatile __far uint8_t payload[RPMSG_BUF_SIZE] __attribute__((cregister("SHARED_RAM", far), peripheral));

#define LED (GPIO.OUT_DATA67_bit.GP6P7)

void _update_msgval(struct ev3_pru_tacho_msg *msg, timer_t timestamp) {
    msg->value[EV3_PRU_IIO_CH_TIMESTAMP] = timestamp;
    tachoencoder_getdircount(MOTOR0, &(msg->value[EV3_PRU_IIO_CH_TACHO_A]));
    tachoencoder_getdircount(MOTOR1, &(msg->value[EV3_PRU_IIO_CH_TACHO_B]));
    tachoencoder_getdircount(MOTOR2, &(msg->value[EV3_PRU_IIO_CH_TACHO_C]));
    tachoencoder_getdircount(MOTOR3, &(msg->value[EV3_PRU_IIO_CH_TACHO_D]));
}


int main(void) {

    // Tacho event detection
    encodervec_t newevent = 0;

    timer_t currtime;

    int i;

	volatile uint8_t *status;
	struct pru_rpmsg_transport transport;
	uint16_t src, dst, len;
	uint16_t trigger_src = 0, trigger_dst = 0;
	// uint32_t start_time;
	bool started = false;

	// Set cregister index to match AM18xx_PRU.cmd
	PRU0_CTRL.CONTABPROPTR1_bit.C30 = 0x1fc;

	// Clear the status of the PRU-system event that the ARM will use to 'kick' us
	PRU_INTC.STATIDXCLR_bit.INDEX = EVENT_FROM_ARM;

	// Wait until Linux gives us the OK that the driver is loaded
	status = &resource_table.rpmsg_vdev.status;
	while (!(*status & VIRTIO_CONFIG_S_DRIVER_OK));

	// TODO: should probably check return value here
	pru_rpmsg_init(&transport, &resource_table.vring0, &resource_table.vring1, EVENT_TO_ARM, EVENT_FROM_ARM);

	while (pru_rpmsg_channel(RPMSG_NS_CREATE, &transport, "ev3-tacho-rpmsg", trigger_src) != PRU_RPMSG_SUCCESS);

	// Setup quadrature encoders
	// Don't use tachometer_init() for now since we're not storing into the event history buffer
    // Initialize per-motor Encoder Settings
    for (i = 0; i < MAX_TACHO_MOTORS; i++) {
        tachoencoder_reset((motor_identifier) i, true);          // Initialize local state
    }

	//start_time = TIMER64P0.TIM34;
    timer_init(TRIGGER_PERIOD_TICKS);

	while (true) {

	    if (started) {
            if (tachoencoder_hasnewevent(&newevent)) {
                currtime = timer_gettimestamp();
                tachoencoder_updateencoderstate(newevent, currtime);        // Actual event timestamp
            }
	    }

		// wait for the ARM to kick us
		while (!(__R31 & HOST_INT)) {

	        if (timer_hasexpired(&currtime) && started) {
					struct ev3_pru_tacho_msg msg;

					msg.type = EV3_PRU_TACHO_MSG_UPDATE;
					_update_msgval(&msg, currtime);
					pru_rpmsg_send(&transport, trigger_src, trigger_dst, &msg, sizeof(msg));
				}
		}

		// clear the interrupt
		PRU_INTC.STATIDXCLR_bit.INDEX = EVENT_FROM_ARM;

		// Receive all available messages, multiple messages can be sent per kick
		while (pru_rpmsg_receive(&transport, &src, &dst, (void *)payload, &len) == PRU_RPMSG_SUCCESS) {
			struct ev3_pru_tacho_msg *msgptr = (void *)payload;

			switch (msgptr->type) {
			case EV3_PRU_TACHO_MSG_REQ_ONE:
                _update_msgval(msgptr, timer_gettimestamp());
				pru_rpmsg_send(&transport, dst, src, msgptr, sizeof(*msgptr));
				break;
			case EV3_PRU_TACHO_MSG_START:
				started = true;
				trigger_src = dst;
				trigger_dst = src;
			    for (i = 0; i < MAX_TACHO_MOTORS; i++) {
			        tachoencoder_reset((motor_identifier) i, false);          // reset local state (encoder count is not cleared)
			    }
				break;
			case EV3_PRU_TACHO_MSG_STOP:
				started = false;
				break;
			}
		}
	}
}
