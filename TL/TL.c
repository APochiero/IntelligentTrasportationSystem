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
#include "stdlib.h"
#include "dev/leds.h"
#include "../Macros.h"


PROCESS(LedManagement, "Led Management");

AUTOSTART_PROCESSES(&LedManagement);


static process_event_t link_ev;
static uint8_t street; 			 // 1 Main Street, 0 Secondary Street
static uint8_t thisWay  	= 0; // 0 NONE, 1 NORMAL, 2 EMERGENCY
static uint8_t otherWay 	= 0;
static uint8_t crossing 	= 0;
static uint8_t firstPck     = 1;

static struct etimer crossingTimer;
static struct etimer toggleLed;


static void recv_broadcast( struct broadcast_conn *c, const linkaddr_t *from ) {
	printf("broadcast message received from %d.%d: '%s' \n", from->u8[0], from->u8[1], (char*) packetbuf_dataptr());
	process_post(&LedManagement, PROCESS_EVENT_MSG, packetbuf_dataptr() );
}

static const struct broadcast_callbacks broadcast_calls = {recv_broadcast};
static struct broadcast_conn broadcast;


static void recv_runicast(struct runicast_conn *c, const linkaddr_t *from, uint8_t seqno){
	printf("%u.%u: Linking to %u.%u, retrasmission %d\n", linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],from->u8[0], from->u8[1], seqno);
	process_post(&LedManagement, link_ev, packetbuf_dataptr());
}

static void sent_runicast(struct runicast_conn *c, const linkaddr_t *to, uint8_t retransmissions){
  printf("Runicast message successfully sent to %d.%d, retransmissions %d\n", to->u8[0], to->u8[1], retransmissions);
}

static void timedout_runicast(struct runicast_conn *c, const linkaddr_t *to, uint8_t retransmissions){
  printf("Runicast message timed out when sending to %d.%d, retransmissions %d\n", to->u8[0], to->u8[1], retransmissions);
}

static const struct runicast_callbacks runicast_calls = {recv_runicast, sent_runicast, timedout_runicast};
static struct runicast_conn runicast;



static void trafficScheduler() {
	crossing = 0;
	if ( thisWay == 0 && otherWay == 0 ) { // No vehicles
		etimer_set(&toggleLed, CLOCK_SECOND);
		return;
	}

	if ( (thisWay == 1 || thisWay == 2) && otherWay == 0 ) { // 1 Vehicle in this way
		leds_on(LEDS_GREEN);
		leds_off(LEDS_RED);
		crossing = 1;
		thisWay = 0;
		firstPck = 1;
	} else if ( thisWay == 0 && (otherWay == 1 || otherWay == 2) ) { // 1 Vehicle other way
		leds_on(LEDS_RED);
		leds_off(LEDS_GREEN);
		otherWay = 0;
		firstPck = 1;
	} else if ( (thisWay == 1 && otherWay == 1) || (thisWay == 2 && otherWay == 2) ) { // 2 Normal or 2 Emer, Main has prio
		if (street) { // TL1
			leds_on(LEDS_GREEN);
			leds_off(LEDS_RED);
			crossing = 1;
			thisWay = 0;
		} else {
			leds_on(LEDS_RED);
			leds_off(LEDS_GREEN);
			otherWay = 0;
		}
	} else if ( thisWay == 2 && otherWay == 1 ) { // Emer in this street, Norm in the other street
		leds_on(LEDS_GREEN);
		leds_off(LEDS_RED);
		crossing = 1;
		thisWay = 0;
	} else if ( thisWay == 1 && otherWay == 2 ) { // Normal in this street, Emer in the other street
		leds_on(LEDS_RED);
		leds_off(LEDS_GREEN);
		otherWay = 0;
	}
	etimer_set(&crossingTimer, CLOCK_SECOND*CROSSINGINTERVAL);
}

PROCESS_THREAD(LedManagement, ev, data) {
	static struct etimer checkOtherStreet;

	PROCESS_EXITHANDLER( broadcast_close(&broadcast));
	PROCESS_BEGIN();

	link_ev = process_alloc_event();
	broadcast_open(&broadcast, 2018, &broadcast_calls);
	runicast_open(&runicast, 144, &runicast_calls);

	uint8_t code;

	while(1) {
		etimer_set(&toggleLed, CLOCK_SECOND);
		PROCESS_WAIT_EVENT();

		if ( ev == link_ev ) {
			street = atoi((char*) data);
		} else if ( ev == PROCESS_EVENT_MSG ) {
			etimer_stop(&toggleLed);
			code = atoi(data);
			if ( street ) { // TL1
				switch(code) {
					case NONE1: 	 thisWay  = 0; break;
					case NORMAL1:	 thisWay  = 1; break;
					case EMERGENCY1: thisWay  = 2; break;
					case NONE2:	     otherWay = 0; break;
					case NORMAL2:    otherWay = 1; break;
					case EMERGENCY2: otherWay = 2; break;
				}
			} else { // TL2
				switch(code) {
					case NONE1: 	 otherWay  = 0; break;
					case NORMAL1:	 otherWay  = 1; break;
					case EMERGENCY1: otherWay  = 2; break;
					case NONE2:	     thisWay   = 0; break;
					case NORMAL2:    thisWay   = 1; break;
					case EMERGENCY2: thisWay   = 2; break;
				}
			}
			if ( firstPck ) { // if this is the first packet of a pair received
				firstPck = 0;
				if ( !crossing  ) // if there is no crossing vehicle, wait a possible packet for 0.5s then make a decision
					etimer_set(&checkOtherStreet, CLOCK_SECOND*CHECKINTERVAL);
			}
		} else if ( etimer_expired(&checkOtherStreet) || etimer_expired(&crossingTimer)) { // No pckts received or Vehicle has crossed the intersection
			trafficScheduler();
		} else if (etimer_expired(&toggleLed)){
			leds_toggle(LEDS_GREEN);
			leds_toggle(LEDS_RED);
			etimer_reset(&toggleLed);
		}
	}
	PROCESS_END();

}
