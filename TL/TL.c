/*
 * TL.c
 *
 *  Created on: Jul 2, 2018
 *      Author: user
 */

#include "contiki.h"
#include "net/rime/rime.h"
#include "sys/etimer.h"
#include "stdio.h"
#include "dev/leds.h"


#define NO_VEHICLE "NO VEHICLE"

PROCESS(LedManagement, "Led Management");

AUTOSTART_PROCESSES(&LedManagement);


static void recv_status( struct broadcast_conn *c, const linkaddr_t *from ) {
	printf("broadcast message received from %d.%d: '%s' \n", from->u8[0], from->u8[1], (char*) packetbuf_dataptr());
	process_post(&LedManagement, PROCESS_EVENT_MSG, packetbuf_dataptr() );
}

static const struct broadcast_callbacks broadcast_call = {recv_status};
static struct broadcast_conn broadcast;

PROCESS_THREAD(LedManagement, ev, data) {
	static struct etimer toggle1Second;

	PROCESS_EXITHANDLER( broadcast_close(&broadcast));
	PROCESS_BEGIN();

	broadcast_open(&broadcast, 2018, &broadcast_call);

	while(1) {

		PROCESS_WAIT_EVENT();

		if ( ev == PROCESS_EVENT_MSG ) {
			printf("Msg recv\n");
			if( strcmp((char*) data, NO_VEHICLE) == 0 ) {
				etimer_set(&toggle1Second, CLOCK_SECOND);
			}
		} else {
			leds_toggle(LEDS_GREEN);
			leds_toggle(LEDS_RED);
			etimer_reset(&toggle1Second);
		}

	}
	PROCESS_END();

}
