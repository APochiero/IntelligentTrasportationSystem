/*
 * G1.c
 *
 *  Created on: Jul 2, 2018
 *      Author: Amedeo Pochiero
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "contiki.h"
#include "net/rime/rime.h"
#include "sys/etimer.h"
#include "lib/sensors.h"
#include "dev/button-sensor.h"

#include "../Macros.h"


PROCESS(TrafficScheduler, "TrafficScheduler");
PROCESS(SensingSink, "SensingSink");

AUTOSTART_PROCESSES(&TrafficScheduler, &SensingSink);

static void recv_status( struct broadcast_conn *c, const linkaddr_t *from ) {
	printf("broadcast message received from %d.%d: '%s' \n", from->u8[0], from->u8[1], (char*) packetbuf_dataptr());
	process_post(&TrafficScheduler, PROCESS_EVENT_MSG, (char*) packetbuf_dataptr());
}

static void send_status(struct broadcast_conn *c, int status, int num_tx) {

  printf("broadcast message sent. Status %d. For this packet, this is transmission number %d\n", status, num_tx);
}

static const struct broadcast_callbacks broadcast_calls = {recv_status, send_status};
static struct broadcast_conn broadcast;

static process_event_t sensing_ev;

static void recv_runicast(struct runicast_conn *c, const linkaddr_t *from, uint8_t seqno){
	printf("New data of %s degrees received from %d.%d. The packet sequence number is %d\n", (char *)packetbuf_dataptr(), from->u8[0], from->u8[1], seqno);
	process_post(&SensingSink, sensing_ev, packetbuf_dataptr());
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

static uint8_t secondaryStreet = 0;
static uint8_t mainStreet      = 0;
static uint8_t crossing  	   = 0;
static uint8_t debug           = 0;
static uint8_t active		   = 1;
static uint8_t firstPress  	   = 1;
static uint8_t firstPck  	   = 1;
static uint8_t waitConcurrencyEnable = 0;


static void activateButton() {
	if ( !active ) {
		SENSORS_ACTIVATE(button_sensor);
		active = 1;
		if (debug ) printf("G1: Button Activate\n");
		firstPress = 1;
	}
}

static void deactivateButton() {
	if ( active ) {
		SENSORS_DEACTIVATE(button_sensor);
		active = 0;
		if (debug ) printf("G1: Button Deactivate\n");
	}
}

static void trafficScheduler(struct etimer* crossingTimer) { // Everytime Main street has priority, reactivate button to catch another vehicle
	if ( mainStreet == 0 && secondaryStreet == 0 ) {
		crossing = 0;
		firstPck = 1;
		if (debug ) printf("Both no vehicle: MainStreet BLINK, Secondary Street BLINK\n");
		return;
	}
	if ( (mainStreet == 1 || mainStreet == 2) && secondaryStreet == 0 ) { // vehicle on main, no vehicle on secondary
		if (debug ) printf("MainStreet normal or emergency GREEN, Secondary Street no vehicle RED\n");
		mainStreet = 0;
		activateButton();
	} else if ( mainStreet == 0 && (secondaryStreet == 1 || secondaryStreet == 2) ) { // No vehicle on main, vehicle on secondary
		if (debug ) printf("MainStreet no vehicle RED, Secondary Street normal or emergency GREEN\n");
		secondaryStreet = 0;
	} else if ( (mainStreet == 1 && secondaryStreet == 1) || (mainStreet == 2 && secondaryStreet == 2) ) { // same vehicle on both street, main has priority
		if (debug ) printf("Same type of vehicle: MainStreet GREEN, Secondary Street RED\n");
		mainStreet = 0;
		activateButton();
	} else if ( mainStreet == 2 && secondaryStreet == 1 ) {
		if (debug ) printf("MainStreet emergency GREEN, Secondary Street normal RED\n");
		mainStreet = 0;
		activateButton();
	} else if ( mainStreet == 1 && secondaryStreet == 2 ) {
		if (debug ) printf("MainStreet normal RED, Secondary Street emergency GREEN\n");
		secondaryStreet = 0;
	}
	crossing = 1;
	etimer_set(crossingTimer, CLOCK_SECOND*CROSSINGINTERVAL);
}

static void setConcurrencyTimer(struct etimer* waitConcurrency) {
	if ( firstPck ) {
		firstPck = 0;
		if ( !crossing ) {
			etimer_set(waitConcurrency, CLOCK_SECOND*CHECKINTERVAL);
			waitConcurrencyEnable = 1;
			if (debug ) printf("G1: waiting for concurrency\n");
		}
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
	static struct etimer crossingTimer;
	static struct etimer waitConcurrency;
	char msg[3];
	PROCESS_EXITHANDLER(broadcast_close(&broadcast));
	PROCESS_EXITHANDLER(runicast_close(&runicast));
	PROCESS_BEGIN();

	broadcast_open(&broadcast, 2018, &broadcast_calls);
	runicast_open(&runicast, 144, &runicast_calls);
	static uint8_t stillInTime = 0;

	linkaddr_t semaphore;
	semaphore.u8[0] = 2;
	semaphore.u8[1] = 0;
	sprintf(msg, "%d", 1);
	printf("G1 [%u.%u]: Linking to TL1 [%u.%u]\n", linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1], semaphore.u8[0], semaphore.u8[1]);
	packetbuf_copyfrom(msg,2);
	runicast_send(&runicast, &semaphore, MAX_RETRANSMISSIONS);

	SENSORS_ACTIVATE(button_sensor);
	if (debug ) printf("G1: Button Activate\n");
	uint8_t code;

	while(1) {
		PROCESS_WAIT_EVENT();

		if ( etimer_expired(&isEmergencyTimer) && stillInTime) { // Timer Expired
			if (debug ) printf("G1: isEmergencyTimer expired\n");
			stillInTime = 0;
			mainStreet = 1;
			sendNewVehicle(NORMAL1);
			setConcurrencyTimer(&waitConcurrency); // Normal on G1, wait for G2 pck
			deactivateButton();
		}  else if ( etimer_expired(&waitConcurrency) && waitConcurrencyEnable && !crossing ) {
			if (debug ) printf("G1: Concurrency Timer expired\n");
			waitConcurrencyEnable = 0;
			trafficScheduler(&crossingTimer);
		} else if ( etimer_expired(&crossingTimer) && crossing ) {
			if (debug ) printf("G1: Crossing Timer expired\n");
			trafficScheduler(&crossingTimer);
		} else if ( ev == sensors_event && data == &button_sensor ) { // Button Press
			if ( firstPress ) { // first button press
				if (debug ) printf("G1: Button Pressed\n");
				etimer_set(&isEmergencyTimer, CLOCK_SECOND*CHECKINTERVAL);
				stillInTime = 1;
				firstPress  = 0;
			} else if ( stillInTime ) { // Second Button Press within 0.5s
				if (debug ) printf("G1: Second Button Pressed\n");
				etimer_stop(&isEmergencyTimer);
				stillInTime = 0;
				sendNewVehicle(EMERGENCY1);
				mainStreet = 2;
				deactivateButton();
				setConcurrencyTimer(&waitConcurrency); // Emergency on G1, wait for G2 pck
			}
		} else if ( ev == PROCESS_EVENT_MSG ) {
			code = atoi((char*) data);
			if (debug ) printf("G1: G2 pck received code %d\n", code);
			switch(code) {
				case NORMAL2:    secondaryStreet = 1; break;
				case EMERGENCY2: secondaryStreet = 2; break;
				default: printf("Unknown code\n"); break;
			}
			setConcurrencyTimer(&waitConcurrency); // G2 first, wait for G1 pck
		}
	}
	PROCESS_END();
}

PROCESS_THREAD(SensingSink, ev, data) {

//	static struct etimer sensingTimer;

	PROCESS_BEGIN();

	runicast_open(&runicast, 152, &runicast_calls);
	runicast_open(&runicast, 153, &runicast_calls);
	runicast_open(&runicast, 154, &runicast_calls);

	printf("Listening on channels 152-153-154\n");
	sensing_ev = process_alloc_event();

	while(1) {
		PROCESS_WAIT_EVENT_UNTIL(ev == sensing_ev);
		printf("Temperature and humidity received %s\n", (char*) data);
	}
	PROCESS_END();
}
