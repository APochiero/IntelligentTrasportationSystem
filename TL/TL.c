/*
 * TL.c
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
#include "dev/leds.h"
#include "dev/sht11/sht11-sensor.h"
#include "dev/button-sensor.h"

#include "../Macros.h"

PROCESS(Init, "Initialization");
PROCESS(TrafficScheduler, "TrafficScheduler");
PROCESS(LedSwitcher, "LedSwitcher");
PROCESS(Sensing,"Sensing" );
PROCESS(LowBatteryPower, "LowBatteryPower");

AUTOSTART_PROCESSES(&Init, &LedSwitcher);


static process_event_t link_ev;
static uint8_t street; 			 // 1 Main Street, 0 Secondary Street
static uint8_t thisStreet  	= 0; // 0 NONE, 1 NORMAL, 2 EMERGENCY
static uint8_t otherStreet 	= 0;
static uint8_t debug  		= 0;
static uint8_t debugSensing = 0;
static uint8_t crossing     = 0;
static uint8_t firstPck	    = 1;
static uint8_t batteryPower = 100;

static void recv_broadcast( struct broadcast_conn *c, const linkaddr_t *from ) {
	printf("broadcast message received from %d.%d: '%s' \n", from->u8[0], from->u8[1], (char*) packetbuf_dataptr());
	process_post(&TrafficScheduler, PROCESS_EVENT_MSG, packetbuf_dataptr() );
}

static const struct broadcast_callbacks broadcast_calls = {recv_broadcast};
static struct broadcast_conn broadcast;


static void recv_runicast(struct runicast_conn *c, const linkaddr_t *from, uint8_t seqno){
	printf("%u.%u: Linking to %u.%u, retrasmission %d\n", linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],from->u8[0], from->u8[1], seqno);
	process_post(&Init, link_ev, packetbuf_dataptr());
}

static void sent_runicast(struct runicast_conn *c, const linkaddr_t *to, uint8_t retransmissions){
  if( debug ) printf("Runicast message successfully sent to %u.%u, retransmissions %d\n", to->u8[0], to->u8[1], retransmissions);
}

static void timedout_runicast(struct runicast_conn *c, const linkaddr_t *to, uint8_t retransmissions){
  printf("Runicast message timed out when sending to %u.%u, retransmissions %d\n", to->u8[0], to->u8[1], retransmissions);
}

static const struct runicast_callbacks runicast_calls = {recv_runicast, sent_runicast, timedout_runicast};
static struct runicast_conn runicast;

static void schedule( struct etimer* crossingTimer) {
	if ( thisStreet == 0 && otherStreet == 0 ) { // No vehicles
		process_start(&LedSwitcher, NULL);
		firstPck = 1;
		leds_off(LEDS_ALL);
		crossing = 0;
		if (debug ) printf("TL: reset, firstPck %d, crossing %d\n", firstPck, crossing);
		return;
	}

	if ( (thisStreet == 1 || thisStreet == 2) && otherStreet == 0 ) { // 1 Vehicle in this way
		leds_on(LEDS_GREEN);
		leds_off(LEDS_RED);
		thisStreet = 0;
	} else if ( thisStreet == 0 && (otherStreet == 1 || otherStreet == 2) ) { // 1 Vehicle other way
		leds_on(LEDS_RED);
		leds_off(LEDS_GREEN);
		otherStreet = 0;
	} else if ( (thisStreet == 1 && otherStreet == 1) || (thisStreet == 2 && otherStreet == 2) ) { // 2 Normal or 2 Emer, Main has prio
		if (street) { // TL1
			leds_on(LEDS_GREEN);
			leds_off(LEDS_RED);
			thisStreet = 0;
		} else {
			leds_on(LEDS_RED);
			leds_off(LEDS_GREEN);
			otherStreet = 0;
		}
	} else if ( thisStreet == 2 && otherStreet == 1 ) { // Emer in this street, Norm in the other street
		leds_on(LEDS_GREEN);
		leds_off(LEDS_RED);
		thisStreet = 0;
	} else if ( thisStreet == 1 && otherStreet == 2 ) { // Normal in this street, Emer in the other street
		leds_on(LEDS_RED);
		leds_off(LEDS_GREEN);
		otherStreet = 0;
	}
	crossing = 1;
	etimer_set(crossingTimer, CLOCK_SECOND*CROSSINGINTERVAL);
}

PROCESS_THREAD(Init, ev, data) {
	PROCESS_EXITHANDLER(runicast_close(&runicast));
	PROCESS_BEGIN();
	runicast_open(&runicast, 144, &runicast_calls);

	PROCESS_WAIT_EVENT_UNTIL(ev == link_ev);
	street = atoi((char*) data);
	if (debug ) printf("TL linked: %d\n", street);
	runicast_close(&runicast);
	process_start(&TrafficScheduler, NULL);
	process_start(&Sensing, NULL);
	PROCESS_END();
}

PROCESS_THREAD(TrafficScheduler, ev, data) {
	static struct etimer waitConcurrency;
	static struct etimer crossingTimer;
	static uint8_t waitConcurrencyEnable = 0;

	PROCESS_EXITHANDLER( broadcast_close(&broadcast));
	PROCESS_BEGIN();

	link_ev = process_alloc_event();
	broadcast_open(&broadcast, 129, &broadcast_calls);

	uint8_t code;

	while(1) {

		PROCESS_WAIT_EVENT();

		if ( etimer_expired(&waitConcurrency) && waitConcurrencyEnable && !crossing ) {
			schedule(&crossingTimer);
			waitConcurrencyEnable = 0;
		} else if ( etimer_expired(&crossingTimer) && crossing ) {
			schedule(&crossingTimer);
		} else if ( ev == PROCESS_EVENT_MSG ) {
			process_exit(&LedSwitcher);
			code = atoi((char*) data);
			if ( street ) { // TL1
				switch(code) {
					case NORMAL1:	 thisStreet  = 1; break;
					case EMERGENCY1: thisStreet  = 2; break;
					case NORMAL2:    otherStreet = 1; break;
					case EMERGENCY2: otherStreet = 2; break;
					default: printf("Unknown code\n"); break;
				}
				if (debug ) printf("TL1: Pck code %d  crossing %d\n", code, crossing);
			} else { // TL2
				switch(code) {
					case NORMAL1:	 otherStreet  = 1; break;
					case EMERGENCY1: otherStreet  = 2; break;
					case NORMAL2:    thisStreet   = 1; break;
					case EMERGENCY2: thisStreet   = 2; break;
					default: printf("Unknown code\n"); break;
				}
				if (debug ) printf("TL2: Pck code %d   crossing %d\n", code, crossing);
			}
			if (debug ) printf("TL: firstPck %d\n", firstPck);
			if ( firstPck ) {
				firstPck = 0;
				if ( !crossing  ) { // if there is no crossing vehicle, wait a possible packet for 0.5s then make a decision
					etimer_set(&waitConcurrency, CLOCK_SECOND*CHECKINTERVAL);
					waitConcurrencyEnable = 1;
					if (debug ) printf("TL: waiting for concurrency \n");
				}
			}
		}
	}
	PROCESS_END();
}

PROCESS_THREAD(LedSwitcher, ev, data) {
	static struct etimer toggleTimer;
	PROCESS_BEGIN();
	etimer_set(&toggleTimer, CLOCK_SECOND);
	while (1) {
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&toggleTimer));
		leds_toggle(LEDS_GREEN);
		leds_toggle(LEDS_RED);
		etimer_reset(&toggleTimer);
		if ( batteryPower >= 5 ) {
			//if (debugSensing ) printf("TL: battery decreased by 5 -- %d\n", batteryPower);
			batteryPower -= 5;
		} else {
			batteryPower = 0;
		}
		if (debug ) printf("TL[%d.%d]: blinking\n",linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1]);
	}
	PROCESS_END();
}

PROCESS_THREAD(Sensing, ev, data) {
	static struct etimer sensingTimer;
	static uint8_t sensingInterval = 5;

	PROCESS_EXITHANDLER(runicast_close(&runicast));
	PROCESS_BEGIN();

	runicast_open(&runicast, 144, &runicast_calls);
	etimer_set(&sensingTimer, CLOCK_SECOND*sensingInterval);

	int16_t temperature;
	int16_t humidity;
	static unsigned char msg[SENSINGPACKETSIZE];
	static linkaddr_t G1Sink;
	G1Sink.u8[0] = 1;
	G1Sink.u8[1] = 0;

	while(1) {
		PROCESS_WAIT_EVENT_UNTIL( etimer_expired(&sensingTimer) );

		SENSORS_ACTIVATE(sht11_sensor);
		temperature = (sht11_sensor.value(SHT11_SENSOR_TEMP)/10-396)/10;
		humidity    = sht11_sensor.value(SHT11_SENSOR_HUMIDITY)/41;
		SENSORS_DEACTIVATE(sht11_sensor);

		sprintf(msg, "%d", street); // 0 TL2, 1 TL1, 2 G2
		sprintf(msg+1,"/");
		sprintf(msg+2, "%d", temperature);
		sprintf(msg+4, "/");
		sprintf(msg+5, "%d", humidity);

		if (debugSensing ) printf("TL: Sensing packet %s\n", msg );
		packetbuf_copyfrom(msg,SENSINGPACKETSIZE);
	    if(!runicast_is_transmitting(&runicast)) {
	    	runicast_send(&runicast, &G1Sink, MAX_RETRANSMISSIONS);
	    }
		if ( batteryPower >= 10 ) {
			batteryPower -= 10;
			//if (debugSensing ) printf("TL: battery decreased by 10 -- %d\n", batteryPower);
		} else
			batteryPower = 0;

		if ( batteryPower <= 20 ) {
			if (debugSensing ) printf("TL: battery under 20 -- %d\n", batteryPower);
			sensingInterval = 20;
			etimer_set(&sensingTimer, CLOCK_SECOND*sensingInterval);
			process_start(&LowBatteryPower, NULL);
		} else if ( batteryPower <= 50 ) {
			if (debugSensing ) printf("TL: battery under 50 -- %d\n", batteryPower);
			sensingInterval = 10;
			etimer_set(&sensingTimer, CLOCK_SECOND*sensingInterval);
		} else {
			if (debugSensing ) printf("TL: battery remained %d \n", batteryPower);
			etimer_reset(&sensingTimer);
		}
	}
	PROCESS_END();
}

PROCESS_THREAD(LowBatteryPower, ev, data) {
	static struct etimer blinkTimer;
	PROCESS_BEGIN();
	etimer_set(&blinkTimer, CLOCK_SECOND);
	SENSORS_ACTIVATE(button_sensor);
	while(1) {
		PROCESS_WAIT_EVENT();
		if ( ev == sensors_event && data == &button_sensor ) {
			if (debugSensing ) printf("TL: Battery Recharged\n");
			batteryPower = 100;
			SENSORS_DEACTIVATE(button_sensor);
			leds_off(LEDS_BLUE);
			PROCESS_EXIT();
		} else if ( etimer_expired(&blinkTimer) ) {
			if (debugSensing ) printf("TL: Low Battery Alarm\n");
			leds_toggle(LEDS_BLUE);
			etimer_reset(&blinkTimer);
		}
	}
	PROCESS_END();
}


