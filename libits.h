/*
 * libits.h
 *
 *  Created on: Jul 2, 2018
 *      Author: Amedeo Pochiero
 */

#include "net/rime/rime.h"
#include "sys/etimer.h"
#include "dev/button-sensor.h"
#include "lib/sensors.h"
#include "dev/sht11/sht11-sensor.h"

#include "Macros.h"

// VARIABLES

#ifdef GROUND
	static uint8_t secondaryStreet = 0; // secondaryStreet status 0 no vehicle, 1 normal, 2 emergency
	static uint8_t mainStreet      = 0; // mainStreet status 0 no vehicle, 1 normal, 2 emergency
	static uint8_t crossing  	   = 0; // 0 no vehicle is crossing, 1 otherwise
	static uint8_t active 	  	   = 1; // button status
	static uint8_t firstPress  	   = 1; // 0 button was already pressed, 1 button is not pressed yet
	static uint8_t firstPck 	   = 1; // 0 one packet already received, 1 no packet received yet
	static uint8_t waitConcurrencyEnable = 0; // 1 if waitConcurrencyTimer is enabled
#endif

#ifdef G1
	static char  emergencyWarning[MSGMAXSIZE] = ""; // RTD Emergency Message Buffer
#endif

#ifdef SENSE
	static linkaddr_t G1Sink = {{G1ADDRESS0,G1ADDRESS1}};
#endif

static void recv_broadcast( struct broadcast_conn *c, const linkaddr_t *from ) {
	if ( DEBUG ) printf("broadcast message received from %d.%d: '%s' \n", from->u8[0], from->u8[1], (char*) packetbuf_dataptr());
	process_post(&TrafficScheduler, PROCESS_EVENT_MSG, packetbuf_dataptr());
}

static void timedout_runicast(struct runicast_conn *c, const linkaddr_t *to, uint8_t retransmissions){
  if (DEBUG) printf("Runicast message timed out when sending to %d.%d, retransmissions %d\n", to->u8[0], to->u8[1], retransmissions);
}

static struct broadcast_conn broadcast;
static struct runicast_conn runicast;
static const struct broadcast_callbacks broadcast_calls = {recv_broadcast, send_broadcast};
static const struct runicast_callbacks runicast_calls = {recv_runicast, sent_runicast, timedout_runicast};

// FUNCTIONS
#ifdef GROUND
static void activateButton() {
	if ( !active ) {
		SENSORS_ACTIVATE(button_sensor);
		active = 1;
		firstPress = 1;
		if (DEBUG) printf("Button activated\n");
	}
}

static void deactivateButton() {
	if ( active ) {
		SENSORS_DEACTIVATE(button_sensor);
		active = 0;
		if (DEBUG) printf("Button deactivated\n");
	}
}


static void schedule(struct etimer* crossingTimer) { // Everytime Main street has priority, reactivate button to catch another vehicle
	if ( mainStreet == 0 && secondaryStreet == 0 ) { // No vehicles
		crossing = 0;
		firstPck = 1;
		if (DEBUG ) printf("Both no vehicle: MainStreet BLINK, Secondary Street BLINK\n");
		return;
	}
	if ( ((mainStreet == 1 || mainStreet == 2) && secondaryStreet == 0 ) || 						 // Vehicle on main, no vehicle on secondary
		 ((mainStreet == 1 && secondaryStreet == 1) || (mainStreet == 2 && secondaryStreet == 2)) || // Same vehicle on both street
		 ( mainStreet == 2 && secondaryStreet == 1 ) ) { 											 // Emergency on main, normal on secondary
		mainStreet = 0;
		#ifdef G1
			activateButton();
		#endif
	} else if (( mainStreet == 0 && (secondaryStreet == 1 || secondaryStreet == 2)) || 				 // No vehicle on main, vehicle on secondary
			   ( mainStreet == 1 && secondaryStreet == 2 )) { 										 // Normal on main, emergency on secondary
		secondaryStreet = 0;
		#ifdef G2
			activateButton();
		#endif
	}
	crossing = 1;
	etimer_set(crossingTimer, CLOCK_SECOND*CROSSINGINTERVAL);
}

static void setConcurrencyTimer(struct etimer* waitConcurrency) { // if 1 vehicle comes within 0.5s after the first one, they are scheduled together
	if ( firstPck ) {
		firstPck = 0;
		if ( !crossing ) {
			etimer_set(waitConcurrency, CLOCK_SECOND*CHECKINTERVAL);
			waitConcurrencyEnable = 1;
			if (DEBUG) printf("Waiting for concurrency\n");
		}
	}
}

static void sendNewVehicle(uint8_t type) { // Broadcast that new vehicle has arrived
	char msg[3];
	sprintf(msg, "%d", type);
	packetbuf_copyfrom(msg,3);
	broadcast_send(&broadcast);
}

#endif

#ifdef SENSE

static void sendSensedData(uint8_t id, int16_t temperature, int16_t humidity ) {
	unsigned char msg[SENSINGPACKETSIZE];

	// Packet format [ id/temperatureValue/humidityValue ]

	sprintf(msg, "%d", id); // 0 TL2, 1 TL1, 2 G2
	sprintf(msg+1,"/");
	sprintf(msg+2, "%d", temperature);
	sprintf(msg+4, "/");
	sprintf(msg+5, "%d", humidity);

	if (DEBUG ) printf("Sensing packet %s\n", msg );
	packetbuf_copyfrom(msg,SENSINGPACKETSIZE);
    if(!runicast_is_transmitting(&runicast)) {
    	runicast_send(&runicast, &G1Sink, MAX_RETRANSMISSIONS);
    }
}

#endif


