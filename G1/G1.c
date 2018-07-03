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
#include "Macros.h"
#include "stdint.h"


PROCESS(TrafficScheduler, "TrafficScheduler");

AUTOSTART_PROCESSES(&TrafficScheduler);

static void recv_status( struct broadcast_conn *c, const linkaddr_t *from ) {
	printf("broadcast message received from %d.%d: '%s' \n", from->u8[0], from->u8[1], (char*) packetbuf_dataptr());
	process_post(&TrafficScheduler, PROCESS_EVENT_MSG, (char*) packetbuf_dataptr());
}

static void send_status(struct broadcast_conn *c, int status, int num_tx) {

  printf("broadcast message sent. Status %d. For this packet, this is transmission number %d\n", status, num_tx);
}

static const struct broadcast_callbacks broadcast_call = {recv_status, send_status};
static struct broadcast_conn broadcast;

static void recv_runicast(struct runicast_conn *c, const linkaddr_t *from, uint8_t seqno){
	printf("New temperature of %s degrees received from %d.%d. The packet sequence number is %d\n", (char *)packetbuf_dataptr(), from->u8[0], from->u8[1], seqno);
	process_post(&LedManagement, packetbuf_dataptr());
}

static void sent_runicast(struct runicast_conn *c, const linkaddr_t *to, uint8_t retransmissions){
  printf("Runicast message successfully sent to %d.%d, retransmissions %d\n", to->u8[0], to->u8[1], retransmissions);
}

static void timedout_runicast(struct runicast_conn *c, const linkaddr_t *to, uint8_t retransmissions){
  printf("Runicast message timed out when sending to %d.%d, retransmissions %d\n", to->u8[0], to->u8[1], retransmissions);
}

static const struct runicast_callbacks runicast_calls = {recv_runicast, sent_runicast, timedout_runicast};
static struct runicast_conn runicast;

/*========================================================================================================================================================
  ========================================================================================================================================================
  ======================================================================================================================================================== */

PROCESS_THREAD( TrafficScheduler, ev, data) {

	static struct etimer isEmergencyTimer;

	PROCESS_EXITHANDLER(broadcast_close(&broadcast));
	PROCESS_EXITHANDLER(runicast_close(&runicast));
	PROCESS_BEGIN();

	broadcast_open(&broadcast, 2018, &broadcast_call);
	static uint8_t stillInTime = 0;
	static uint8_t firstPress  = 1;

	linkaddr_t semaphore;
	semaphore.u8[0] = 1;
	semaphore.u8[1] = 0;
	printf("%u.%u: Linking to %u.%u\n", linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1], semaphore.u8[0], semaphore.u8[1]);
	packetbuf_copyfrom('1',1);
	runicast_send(&runicast, &semaphore, MAX_RETRANSMISSIONS);

	while(1) {
		SENSORS_ACTIVATE(button_sensor);

		PROCESS_WAIT_EVENT();

		if ( ev == sensors_event && data == &button_sensor ) { // Button Press
			if ( firstPress ) { // first button press
				etimer_set(&isEmergencyTimer, CLOCK_SECOND*EMERGENCYINTERVAL);
				stillInTime = 1;
				firstPress    = 0;
			} else {
				etimer_stop(&isEmergencyTimer);
				stillInTime = 0;
				packetbuf_copyfrom(EMERGENCY1,2);
				broadcast_send(&broadcast);
			}
		} else if ( etimer_expired(&isEmergencyTimer ) ) { // Timer Expired
			stillInTime = 0;
			packetbuf_copyfrom(NORMAL1,2);
			broadcast_send(&broadcast);
		}


		packetbuf_copyfrom(NONE,11);
		broadcast_send(&broadcast);

	}
	PROCESS_END();
}
