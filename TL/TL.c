/*
 * TL.c
 *
 *  Created on: Jul 2, 2018
 *      Author: Amedeo Pochiero
 */

#include "contiki.h"
#include "net/rime/rime.h"
#include "sys/etimer.h"
#include "stdio.h"
#include "stdlib.h"
#include "dev/leds.h"
#include "../Macros.h"


PROCESS(TrafficScheduler, "TrafficScheduler");
PROCESS(ToggleLed, "ToggleLed");

AUTOSTART_PROCESSES(&TrafficScheduler, &ToggleLed);


static process_event_t link_ev;
static uint8_t street; 			 // 1 Main Street, 0 Secondary Street
static uint8_t thisStreet  	= 0; // 0 NONE, 1 NORMAL, 2 EMERGENCY
static uint8_t otherStreet 	= 0;
static uint8_t debug  		= 1;
static uint8_t crossing     = 0;
static uint8_t firstPck	    = 1;


static void recv_broadcast( struct broadcast_conn *c, const linkaddr_t *from ) {
	printf("broadcast message received from %d.%d: '%s' \n", from->u8[0], from->u8[1], (char*) packetbuf_dataptr());
	process_post(&TrafficScheduler, PROCESS_EVENT_MSG, packetbuf_dataptr() );
}

static const struct broadcast_callbacks broadcast_calls = {recv_broadcast};
static struct broadcast_conn broadcast;


static void recv_runicast(struct runicast_conn *c, const linkaddr_t *from, uint8_t seqno){
	printf("%u.%u: Linking to %u.%u, retrasmission %d\n", linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],from->u8[0], from->u8[1], seqno);
	process_post(&TrafficScheduler, link_ev, packetbuf_dataptr());
}

static void sent_runicast(struct runicast_conn *c, const linkaddr_t *to, uint8_t retransmissions){
  printf("Runicast message successfully sent to %d.%d, retransmissions %d\n", to->u8[0], to->u8[1], retransmissions);
}

static void timedout_runicast(struct runicast_conn *c, const linkaddr_t *to, uint8_t retransmissions){
  printf("Runicast message timed out when sending to %d.%d, retransmissions %d\n", to->u8[0], to->u8[1], retransmissions);
}

static const struct runicast_callbacks runicast_calls = {recv_runicast, sent_runicast, timedout_runicast};
static struct runicast_conn runicast;



static void schedule( struct etimer* crossingTimer) {
	if ( thisStreet == 0 && otherStreet == 0 ) { // No vehicles
		process_start(&ToggleLed, NULL);
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

PROCESS_THREAD(TrafficScheduler, ev, data) {
	static struct etimer waitConcurrency;
	static struct etimer crossingTimer;
	static uint8_t waitConcurrencyEnable = 0;

	PROCESS_EXITHANDLER( broadcast_close(&broadcast));
	PROCESS_BEGIN();

	link_ev = process_alloc_event();
	broadcast_open(&broadcast, 2018, &broadcast_calls);
	runicast_open(&runicast, 144, &runicast_calls);

	uint8_t code;

	while(1) {

		PROCESS_WAIT_EVENT();

		if ( etimer_expired(&waitConcurrency) && waitConcurrencyEnable && !crossing ) {
			schedule(&crossingTimer);
			waitConcurrencyEnable = 0;
		} else if ( etimer_expired(&crossingTimer) && crossing ) {
			schedule(&crossingTimer);
		} else if ( ev == link_ev ) {
			street = atoi((char*) data);
			if (debug ) printf("TL linked: %d\n", street);
		} else if ( ev == PROCESS_EVENT_MSG ) {
			process_exit(&ToggleLed);
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


PROCESS_THREAD(ToggleLed, ev, data) {
	static struct etimer toggleLed;
	PROCESS_BEGIN();
	etimer_set(&toggleLed, CLOCK_SECOND);
	while (1) {
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&toggleLed));
		leds_toggle(LEDS_GREEN);
		leds_toggle(LEDS_RED);
		etimer_reset(&toggleLed);
		if (debug ) printf("TL[%d.%d]: blinking\n",linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1]);
	}
	PROCESS_END();
}
