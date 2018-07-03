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
#include "stdlib.h"
#include "dev/button-sensor.h"
#include "../Macros.h"
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

static const struct broadcast_callbacks broadcast_calls = {recv_status, send_status};
static struct broadcast_conn broadcast;

static void recv_runicast(struct runicast_conn *c, const linkaddr_t *from, uint8_t seqno){
//	printf("New temperature of %s degrees received from %d.%d. The packet sequence number is %d\n", (char *)packetbuf_dataptr(), from->u8[0], from->u8[1], seqno);
//	process_post(&LedManagement, packetbuf_dataptr());
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

static struct etimer crossingTimer;
static struct etimer waitConcurrency;

static uint8_t secondaryStreet = 0;
static uint8_t mainStreet      = 0;
static uint8_t firstPck    	   = 1;
static uint8_t crossing  	   = 0;

static void trafficScheduler() { // Everytime Main street has priority, reactivate button to catch another vehicle
	crossing = 0;
	if ( (mainStreet == 1 || mainStreet == 2) && secondaryStreet == 0 ) { // vehicle on main, no vehicle on secondary
		mainStreet = 0;
		firstPck = 1;
		crossing = 1;
		SENSORS_ACTIVATE(button_sensor);
	} else if ( mainStreet == 0 && (secondaryStreet == 1 || secondaryStreet == 2) ) { // No vehicle on main, vehicle on secondary
		secondaryStreet = 0;
		firstPck = 1;
	} else if ( (mainStreet == 1 && secondaryStreet == 1) || (mainStreet == 2 && secondaryStreet == 2) ) { // same vehicle on both street, main has priority
		mainStreet = 0;
		crossing = 1;
		SENSORS_ACTIVATE(button_sensor);
	} else if ( mainStreet == 2 && secondaryStreet == 1 ) {
		mainStreet = 0;
		crossing = 1;
		SENSORS_ACTIVATE(button_sensor);
	} else if ( mainStreet == 1 && secondaryStreet == 2 ) {
		secondaryStreet = 0;
	}
	etimer_set(&crossingTimer, CLOCK_SECOND*CROSSINGINTERVAL);
}

static void setConcurrencyTimer() {
	if ( firstPck ) { // G2 first, waiting for G1 pck
		firstPck = 0;
		if ( !crossing )
			etimer_set(&waitConcurrency, CLOCK_SECOND*CHECKINTERVAL);
	}
}

static void sendNewVehicle(uint8_t type) {
	char msg[3];
	sprintf(msg, "%d", type);
	packetbuf_copyfrom(msg,3);
	broadcast_send(&broadcast);
}

PROCESS_THREAD( TrafficScheduler, ev, data ) {

	static struct etimer isEmergencyTimer;
	char msg[3];

	PROCESS_EXITHANDLER(broadcast_close(&broadcast));
	PROCESS_EXITHANDLER(runicast_close(&runicast));
	PROCESS_BEGIN();

	broadcast_open(&broadcast, 2018, &broadcast_calls);
	runicast_open(&runicast, 144, &runicast_calls);
	static uint8_t stillInTime = 0;
	static uint8_t firstPress  = 1;

	linkaddr_t semaphore;
	semaphore.u8[0] = 2;
	semaphore.u8[1] = 0;
	sprintf(msg, "%d", 1);
	printf("%u.%u: Linking to %u.%u\n", linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1], semaphore.u8[0], semaphore.u8[1]);
	packetbuf_copyfrom(msg,2);
	runicast_send(&runicast, &semaphore, MAX_RETRANSMISSIONS);

	SENSORS_ACTIVATE(button_sensor);
	uint8_t code;

	while(1) {
		PROCESS_WAIT_EVENT();

		if ( ev == sensors_event && data == &button_sensor ) { // Button Press
			if ( firstPress ) { // first button press
				etimer_set(&isEmergencyTimer, CLOCK_SECOND*CHECKINTERVAL);
				stillInTime = 1;
				firstPress  = 0;
			} else if ( stillInTime ) { // Second Button Press within 0.5s
				etimer_stop(&isEmergencyTimer);
				stillInTime = 0;
				sendNewVehicle(EMERGENCY1);
				mainStreet = 2;
				SENSORS_DEACTIVATE(button_sensor);
				setConcurrencyTimer(); // Emergency on G1, wait for G2 pck
			}
		} else if ( etimer_expired(&isEmergencyTimer ) ) { // Timer Expired
			stillInTime = 0;
			SENSORS_DEACTIVATE(button_sensor);
			mainStreet = 1;
			sendNewVehicle(EMERGENCY1);
			setConcurrencyTimer(); // Normal on G1, wait for G2 pck
		} else if ( ev == PROCESS_EVENT_MSG ) {
			code = atoi((char*) data);
			switch(code) {
				case NORMAL2:    secondaryStreet = 1; break;
				case EMERGENCY2: secondaryStreet = 2; break;
				default: printf("Unknown code\n"); break;
			}
			setConcurrencyTimer(); // G2 first, wait for G1 pck
		} else if ( etimer_expired(&waitConcurrency) || etimer_expired(&crossingTimer) ) {
			trafficScheduler();
		}
	}
	PROCESS_END();
}
