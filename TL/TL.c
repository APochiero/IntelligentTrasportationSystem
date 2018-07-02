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
#include "Macros.h"


PROCESS(LedManagement, "Led Management");

AUTOSTART_PROCESSES(&LedManagement);


static void recv_broadcast( struct broadcast_conn *c, const linkaddr_t *from ) {
	printf("broadcast message received from %d.%d: '%s' \n", from->u8[0], from->u8[1], (char*) packetbuf_dataptr());
	process_post(&LedManagement, PROCESS_EVENT_MSG, packetbuf_dataptr() );
}

static const struct broadcast_callbacks broadcast_call = {recv_broadcast};
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

static process_event_t link_ev;
static uint8_t street; // 1 Main Street, 0 Secondary Street
static uint8_t thisWay  	= 0; // 0 NONE, 1 NORMAL, 2 EMERGENCY
static uint8_t otherWay 	= 0;
static uint8_t crossing 	= 0;
static struct etimer crossingTimer;
static struct etimer toggleLed;


static void trafficScheduler() {
	if ( (thisWay == 1 || thisWay == 2) && otherWay == 0 ) { // 1 Vehicle in this way
		leds_on(LEDS_GREEN);
		leds_off(LEDS_RED);
		crossing = 1;
		thisWay = 0;
	} else if ( thisWay == 0 && (otherWay == 1 || otherWay == 2) ) { // 1 Vehicle other way
		leds_on(LEDS_RED);
		leds_off(LEDS_GREEN);
		otherWay = 0;
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
	} else if ( thisWay == 0 && otherWay == 0 ) { // No vehicles
		etimer_set(&toggleLed, CLOCK_SECOND);
	}
	etimer_set(&crossingTimer, CLOCK_SECOND*CROSSINGINTERVAL);
}

PROCESS_THREAD(LedManagement, ev, data) {
	static struct etimer checkOtherStreet;

	PROCESS_EXITHANDLER( broadcast_close(&broadcast));
	PROCESS_BEGIN();

	link_ev = process_alloc_event();
	broadcast_open(&broadcast, 2018, &broadcast_call);

	uint8_t firstPck	= 1;

	while(1) {
		etimer_set(&toggleLed, CLOCK_SECOND);
		PROCESS_WAIT_EVENT();

		if ( ev == link_ev ) {
			street = *data;
		} else if ( ev == PROCESS_EVENT_MSG ) {
			etimer_stop(&toggleLed);
			if ( street ) { // TL1
				if ( strcmp((char*) data, NORMAL1) == 0 ) { // NORMAL MAIN
					thisWay = 1;
				} else if ( strcmp((char*) data, EMERGENCY1 ) == 0 ) { // EMER MAIN
					thisWay = 2;
				} else if( strcmp((char*) data, NONE1) == 0 ) { // NONE MAIN
					thisWay = 0;
				} else if ( strcmp((char*) data, NORMAL2 ) == 0 ) { // NORMAL SEC
					otherWay = 1;
				} else if ( strcmp((char*) data, EMERGENCY2 ) == 0 ) { // EMER SEC
					otherWay = 2;
				} else if( strcmp((char*) data, NONE2) == 0 ) { // NONE SEC
					otherWay = 0;
				}
			} else { // TL2
				if ( strcmp((char*) data, NORMAL2) == 0 ) { // NORMAL SEC
					thisWay = 1;
				} else if ( strcmp((char*) data, EMERGENCY2 ) == 0 ) { // EMER SEC
					thisWay = 2;
				} else if( strcmp((char*) data, NONE2) == 0 ) { // NONE SEC
					thisWay = 0;
				}else if ( strcmp((char*) data, NORMAL1 ) == 0 ) { // NORMAL MAIN
					otherWay = 1;
				} else if ( strcmp((char*) data, EMERGENCY1 ) == 0 ) { // EMER MAIN
					otherWay = 2;
				} else if( strcmp((char*) data, NONE1) == 0 ) { // NONE MAIN
					otherWay = 0;
				}
			}
			if ( firstPck && !crossing ) {
				etimer_set(&checkOtherStreet, CLOCK_SECOND*CHECKINTERVAL);
				firstPck = 0;
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
