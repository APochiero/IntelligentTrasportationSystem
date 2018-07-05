/*
 * G2.c
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
#include "dev/sht11/sht11-sensor.h"


#include "../Macros.h"

PROCESS(Init, "Initialization");
PROCESS(TrafficScheduler, "TrafficScheduler");
PROCESS(Sensing, "Sensing");

AUTOSTART_PROCESSES(&Init);

static uint8_t debug 		   = 0;
static uint8_t debugSensing	   = 0;

static void recv_status( struct broadcast_conn *c, const linkaddr_t *from ) {
	printf("broadcast message received from %d.%d: '%s' \n", from->u8[0], from->u8[1], (char*) packetbuf_dataptr());
	process_post(&TrafficScheduler, PROCESS_EVENT_MSG, (char*) packetbuf_dataptr());
}

static void send_status(struct broadcast_conn *c, int status, int num_tx) {

  printf("broadcast message sent. Status %d. For this packet, this is transmission number %d\n", status, num_tx);
}

static const struct broadcast_callbacks broadcast_calls = {recv_status, send_status};
static struct broadcast_conn broadcast;

static void recv_runicast(struct runicast_conn *c, const linkaddr_t *from, uint8_t seqno){}

static void sent_runicast(struct runicast_conn *c, const linkaddr_t *to, uint8_t retransmissions){
  if( debug ) printf("Runicast message successfully sent to %d.%d, retransmissions %d\n", to->u8[0], to->u8[1], retransmissions);
  if ( process_is_running(&Init) )
  	  process_post(&Init, PROCESS_EVENT_CONTINUE, NULL);
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
static uint8_t active 	  	   = 1;
static uint8_t firstPress  	   = 1;
static uint8_t firstPck 	   = 1;
static uint8_t waitConcurrencyEnable = 0;



static void activateButton() {
	if ( !active ) {
		SENSORS_ACTIVATE(button_sensor);
		active = 1;
		if (debug ) printf("G2: Button Activate\n");
		firstPress = 1;
	}
}

static void deactivateButton() {
	if ( active ) {
		SENSORS_DEACTIVATE(button_sensor);
		active = 0;
		if (debug ) printf("G2: Button Deactivate\n");
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
		mainStreet = 0;
	} else if ( mainStreet == 0 && (secondaryStreet == 1 || secondaryStreet == 2) ) { // No vehicle on main, vehicle on secondary
		secondaryStreet = 0;
		activateButton();
	} else if ( (mainStreet == 1 && secondaryStreet == 1) || (mainStreet == 2 && secondaryStreet == 2) ) { // same vehicle on both street, main has priority
		mainStreet = 0;
	} else if ( mainStreet == 2 && secondaryStreet == 1 ) {
		mainStreet = 0;
	} else if ( mainStreet == 1 && secondaryStreet == 2 ) {
		secondaryStreet = 0;
		activateButton();
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
			if (debug ) printf("G2: waiting for concurrency\n");
		}
	}
}

static void sendNewVehicle(uint8_t type) {
	char msg[3];
	sprintf(msg, "%d", type);
	packetbuf_copyfrom(msg,3);
	broadcast_send(&broadcast);
}


PROCESS_THREAD( Init, ev, data) {
	PROCESS_EXITHANDLER(runicast_close(&runicast));
	PROCESS_BEGIN();
	runicast_open(&runicast, 144, &runicast_calls);
	char msg[3];
	linkaddr_t semaphore;
	semaphore.u8[0] = 4;
	semaphore.u8[1] = 0;
	sprintf(msg, "%d", 0);
	printf("G2 [%u.%u]: Linking to TL2 [%u.%u]\n", linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1], semaphore.u8[0], semaphore.u8[1]);
	packetbuf_copyfrom(msg,2);
	runicast_send(&runicast, &semaphore, MAX_RETRANSMISSIONS);
	PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_CONTINUE);
	runicast_close(&runicast);
	process_start(&TrafficScheduler, NULL);
	process_start(&Sensing, NULL);
	PROCESS_END();
}

PROCESS_THREAD( TrafficScheduler, ev, data ) {

	static struct etimer isEmergencyTimer;
	static struct etimer crossingTimer;
	static struct etimer waitConcurrency;

	PROCESS_EXITHANDLER(broadcast_close(&broadcast));
	PROCESS_BEGIN();

	SENSORS_ACTIVATE(button_sensor);
	if (debug ) printf("G2: Button Activate\n");

	broadcast_open(&broadcast, 129, &broadcast_calls);
	static uint8_t stillInTime = 0;

	uint8_t code;
	while(1) {
		PROCESS_WAIT_EVENT();

		if ( etimer_expired(&isEmergencyTimer ) && stillInTime ) { // Timer Expired
			if (debug ) printf("G2: isEmergency Timer expired\n");
			stillInTime = 0;
			deactivateButton();
			secondaryStreet = 1;
			sendNewVehicle(NORMAL2);
			setConcurrencyTimer(&waitConcurrency); // Normal on G2, wait for G1 pck
		}  else if ( etimer_expired(&waitConcurrency) && waitConcurrencyEnable && !crossing ) {
			if (debug ) printf("G2: Concurrency Timer expired\n");
			waitConcurrencyEnable = 0;
			trafficScheduler(&crossingTimer);
		} else if ( etimer_expired(&crossingTimer) && crossing ) {
			if (debug ) printf("G2: Crossing Timer expired\n");
			trafficScheduler(&crossingTimer);
		} else if ( ev == sensors_event && data == &button_sensor ) { // Button Press
			if ( firstPress ) { // first button press
				if (debug ) printf("G2: Button Pressed\n");
				etimer_set(&isEmergencyTimer, CLOCK_SECOND*CHECKINTERVAL);
				stillInTime = 1;
				firstPress  = 0;
			} else if ( stillInTime ) { // Second Button Press within 0.5s
				if (debug ) printf("G2: Second Button Pressed\n");
				etimer_stop(&isEmergencyTimer);
				stillInTime = 0;
				sendNewVehicle(EMERGENCY2);
				secondaryStreet = 2;
				deactivateButton();
				setConcurrencyTimer(&waitConcurrency);
			}
		} else if ( ev == PROCESS_EVENT_MSG ) {
			code = atoi((char*) data);
			if (debug ) printf("G2: G1 pck received code %d\n", code);
			switch(code) {
				case NORMAL1:    mainStreet = 1; break;
				case EMERGENCY1: mainStreet = 2; break;
				default: printf("Unknown code\n"); break;
			}
			setConcurrencyTimer(&waitConcurrency); // G1 first, wait for G2 pck
		}
	}
	PROCESS_END();
}

PROCESS_THREAD(Sensing, ev, data) {
	static struct etimer sensingTimer;

	PROCESS_EXITHANDLER(runicast_close(&runicast));
	PROCESS_BEGIN();

	runicast_open(&runicast, 144, &runicast_calls);
	etimer_set(&sensingTimer, CLOCK_SECOND*SENSINGINTERVAL);

	int16_t temperature;
	int16_t humidity;
	static char msg[SENSINGPACKETSIZE];
	static linkaddr_t G1Sink;
	G1Sink.u8[0] = 1;
	G1Sink.u8[1] = 0;

	while(1) {
		PROCESS_WAIT_EVENT_UNTIL( etimer_expired(&sensingTimer) );

		SENSORS_ACTIVATE(sht11_sensor);
		temperature = (sht11_sensor.value(SHT11_SENSOR_TEMP)/10-396)/10;
		humidity    = sht11_sensor.value(SHT11_SENSOR_HUMIDITY)/41;
		SENSORS_DEACTIVATE(sht11_sensor);

		sprintf(msg, "2"); // 0 TL2, 1 TL1, 2 G2
		sprintf(msg+1,"/");
		sprintf(msg+2, "%d", temperature);
		sprintf(msg+4, "/");
		sprintf(msg+5, "%d", humidity);

		if (debugSensing ) printf("TL: Sensing packet %s\n", msg );
		packetbuf_copyfrom(msg,SENSINGPACKETSIZE);
	    if(!runicast_is_transmitting(&runicast))
			runicast_send(&runicast, &G1Sink, MAX_RETRANSMISSIONS);
		etimer_reset(&sensingTimer);
	}
	PROCESS_END();

}
