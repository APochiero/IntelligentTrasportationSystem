/*
 * G1.c
 *
 *  Created on: Jul 2, 2018
 *      Author: user
 */


#include "contiki.h"
#include "net/rime/rime.h"
#include "sys/etimer.h"
#include "stdio.h"
#include "dev/button-sensor.h"

#define NO_VEHICLE "NO VEHICLE"

PROCESS(TrafficScheduler, "TrafficScheduler");

AUTOSTART_PROCESSES(&TrafficScheduler);

static void broadcast_recv( struct broadcast_conn *c, const linkaddr_t *from ) {
	printf("broadcast message received from %d.%d: '%s' \n", from->u8[0], from->u8[1], (char*) packetbuf_dataptr());
	process_post(&TrafficScheduler, PROCESS_EVENT_MSG, (char*) packetbuf_dataptr());
}

static void broadcast_sent(struct broadcast_conn *c, int status, int num_tx) {

  printf("broadcast message sent. Status %d. For this packet, this is transmission number %d\n", status, num_tx);
}

static const struct broadcast_callbacks broadcast_call = {broadcast_recv, broadcast_sent};
static struct broadcast_conn broadcast;

PROCESS_THREAD( TrafficScheduler, ev, data) {

	PROCESS_EXITHANDLER(broadcast_close(&broadcast));
	PROCESS_BEGIN();

	broadcast_open(&broadcast, 2018, &broadcast_call);

	while(1) {
		SENSORS_ACTIVATE(button_sensor);

		PROCESS_WAIT_EVENT_UNTIL(ev == sensors_event && data == &button_sensor );

		packetbuf_copyfrom(NO_VEHICLE,11);
		broadcast_send(&broadcast);

	}
	PROCESS_END();
}
