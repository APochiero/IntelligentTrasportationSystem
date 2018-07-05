/*
 * G1.c
 *
 *  Created on: Jul 2, 2018
 *      Author: Amedeo Pochiero
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include "contiki.h"
#include "net/rime/rime.h"
#include "sys/etimer.h"
#include "lib/sensors.h"
#include "dev/button-sensor.h"
#include "dev/sht11/sht11-sensor.h"
#include "dev/serial-line.h"

#include "../Macros.h"

PROCESS(Init, "Initialization");
PROCESS(TrafficScheduler, "TrafficScheduler");
PROCESS(SensingSink, "SensingSink");
PROCESS(WriteEmergencyWarning, "WriteEmergencyWarning");

AUTOSTART_PROCESSES( &Init );


static void recv_broadcast( struct broadcast_conn *c, const linkaddr_t *from ) {
	if ( DEBUG ) printf("broadcast message received from %d.%d: '%s' \n", from->u8[0], from->u8[1], (char*) packetbuf_dataptr());
	process_post(&TrafficScheduler, PROCESS_EVENT_MSG, (char*) packetbuf_dataptr());
}

static void send_broadcast(struct broadcast_conn *c, int status, int num_tx) {
  if ( DEBUG ) printf("broadcast message sent. Status %d. For this packet, this is transmission number %d\n", status, num_tx);
}

static process_event_t sensing_ev;
static void recv_runicast(struct runicast_conn *c, const linkaddr_t *from, uint8_t seqno){
	process_post(&SensingSink, sensing_ev, packetbuf_dataptr());
}
static void sent_runicast(struct runicast_conn *c, const linkaddr_t *to, uint8_t retransmissions){
  printf("G1 successfully linked to %d.%d, retransmissions %d\n", to->u8[0], to->u8[1], retransmissions);
  if ( process_is_running(&Init) ) {
	  process_post(&Init, PROCESS_EVENT_CONTINUE, NULL);
  }
}
static void timedout_runicast(struct runicast_conn *c, const linkaddr_t *to, uint8_t retransmissions){
  printf("Runicast message timed out when sending to %d.%d, retransmissions %d\n", to->u8[0], to->u8[1], retransmissions);
}

static struct broadcast_conn broadcast;
static struct runicast_conn runicast;
static const struct broadcast_callbacks broadcast_calls = {recv_broadcast, send_broadcast};
static const struct runicast_callbacks runicast_calls = {recv_runicast, sent_runicast, timedout_runicast};

static uint8_t secondaryStreet = 0; // secondaryStreet status 0 no vehicle, 1 normal, 2 emergency
static uint8_t mainStreet      = 0; // mainStreet status 0 no vehicle, 1 normal, 2 emergency
static uint8_t crossing  	   = 0; // 0 no vehicle is crossing, 1 otherwise
static uint8_t active 	  	   = 1; // button status
static uint8_t firstPress  	   = 1; // 0 button was already pressed, 1 button is not pressed yet
static uint8_t firstPck 	   = 1; // 0 one packet already received, 1 no packet received yet
static uint8_t waitConcurrencyEnable = 0; // 1 if waitConcurrencyTimer is enabled
static char  emergencyWarning[MSGMAXSIZE] = ""; // RTD Emergency Message Buffer

static void activateButton() {
	if ( !active ) {
		SENSORS_ACTIVATE(button_sensor);
		active = 1;
		if (DEBUG ) printf("G1: Button Activate\n");
		firstPress = 1;
	}
}

static void deactivateButton() {
	if ( active ) {
		SENSORS_DEACTIVATE(button_sensor);
		active = 0;
		if (DEBUG ) printf("G1: Button Deactivate\n");
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
		activateButton();
	} else if (( mainStreet == 0 && (secondaryStreet == 1 || secondaryStreet == 2)) || 				 // No vehicle on main, vehicle on secondary
			   ( mainStreet == 1 && secondaryStreet == 2 )) { 										 // Normal on main, emergency on secondary
		secondaryStreet = 0;
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
			if (DEBUG ) printf("G1: waiting for concurrency\n");
		}
	}
}

static void sendNewVehicle(uint8_t type) { // Broadcast that new vehicle has arrived
	char msg[3];
	sprintf(msg, "%d", type);
	packetbuf_copyfrom(msg,3);
	broadcast_send(&broadcast);
}

PROCESS_THREAD(Init, ev, data) { 				// Link G1 to TL1 then start other processes
	PROCESS_EXITHANDLER(runicast_close(&runicast));
	PROCESS_BEGIN();
	runicast_open(&runicast, 144, &runicast_calls);
	char msg[3];
	linkaddr_t semaphore;
	semaphore.u8[0] = TL1ADDRESS0;
	semaphore.u8[1] = TL1ADDRESS1;
	sprintf(msg, "%d", 1);
	if ( DEBUG ) printf("G1 [%u.%u]: Linking to TL1 [%u.%u]\n", linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1], semaphore.u8[0], semaphore.u8[1]);
	packetbuf_copyfrom(msg,2);
	runicast_send(&runicast, &semaphore, MAX_RETRANSMISSIONS);
	PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_CONTINUE);
	runicast_close(&runicast);
	process_start(&TrafficScheduler, NULL);
	process_start(&WriteEmergencyWarning, NULL);
	process_start(&SensingSink, NULL);
	PROCESS_END();
}

PROCESS_THREAD( TrafficScheduler, ev, data ) {

	static struct etimer isEmergencyTimer;
	static struct etimer crossingTimer;
	static struct etimer waitConcurrency;

	PROCESS_EXITHANDLER(broadcast_close(&broadcast));
	PROCESS_BEGIN();

	broadcast_open(&broadcast, 129, &broadcast_calls);
	static uint8_t stillInTime = 0;

	SENSORS_ACTIVATE(button_sensor);
	if (DEBUG ) printf("G1: Button Activate\n");
	uint8_t code;

	while(1) {
		PROCESS_WAIT_EVENT();

		if ( etimer_expired(&isEmergencyTimer) && stillInTime) { // EmergencyTimer expired --> send Normal vehicle code
			if (DEBUG ) printf("G1: isEmergencyTimer expired\n");
			stillInTime = 0;
			mainStreet = 1;
			sendNewVehicle(NORMAL1);
			setConcurrencyTimer(&waitConcurrency);
			deactivateButton();
		}  else if ( etimer_expired(&waitConcurrency) && waitConcurrencyEnable && !crossing ) { // Concurrency checked --> schedule
			if (DEBUG ) printf("G1: Concurrency Timer expired\n");
			waitConcurrencyEnable = 0;
			schedule(&crossingTimer);
		} else if ( etimer_expired(&crossingTimer) && crossing ) { // Crossing interval expired --> schedule again
			if (DEBUG ) printf("G1: Crossing Timer expired\n");
			schedule(&crossingTimer);
		} else if ( ev == sensors_event && data == &button_sensor ) { // Button Press --> if first press wait for second press, otherwise wait for concurrency
			if ( firstPress ) { // first button press
				if (DEBUG ) printf("G1: Button Pressed\n");
				etimer_set(&isEmergencyTimer, CLOCK_SECOND*CHECKINTERVAL);
				stillInTime = 1;
				firstPress  = 0;
			} else if ( stillInTime ) { // Second Button Press within 0.5s
				if (DEBUG ) printf("G1: Second Button Pressed\n");
				etimer_stop(&isEmergencyTimer);
				stillInTime = 0;
				sendNewVehicle(EMERGENCY1);
				mainStreet = 2;
				deactivateButton();
				setConcurrencyTimer(&waitConcurrency); // Emergency on G1, wait for G2 pck
			}
		} else if ( ev == PROCESS_EVENT_MSG ) {  // G1 notified a vehicle
			code = atoi((char*) data);
			if (DEBUG ) printf("G1: G2 pck received code %d\n", code);
			switch(code) {
				case NORMAL2:    secondaryStreet = 1; break;
				case EMERGENCY2: secondaryStreet = 2; break;
				default: printf("Unknown code\n"); break;
			}
			setConcurrencyTimer(&waitConcurrency); // wait for vehicle coming on G1
		}
	}
	PROCESS_END();
}

PROCESS_THREAD(SensingSink, ev, data) { // Receive data sensed by the other sensors and write the average on RTD

	static int16_t temperature[SENSORSCOUNT];
	static int16_t humidity[SENSORSCOUNT];
	static uint8_t isNew[SENSORSCOUNT-1]; // flags
	static uint8_t index;
	PROCESS_BEGIN();

	runicast_open(&runicast, 144, &runicast_calls);

	printf("Listening on channels 144\n");
	sensing_ev = process_alloc_event();

	uint8_t i;
	int16_t tempAvg, humAvg;
	for ( i = 0; i < SENSORSCOUNT-1; i++ ) {
		isNew[i] = 0;
	}
	while(1) {
		PROCESS_WAIT_EVENT_UNTIL(ev == sensing_ev);
		char* packet = (char*) data;				// Packet format [ id/temperatureValue/humidityValue ]
		index = atoi( strtok( packet,  '/') );
		temperature[index] = atoi( strtok( packet + 2, '/') );
		humidity[index] = atoi( strtok( packet + 5, '\0') );
		isNew[index] = 1;
		if (DEBUGSENSING ) printf("G1: Received from %d temp %d, hum %d \n", index, temperature[index], humidity[index]);

		if( isNew[0] == 1 && isNew[1] == 1 && isNew[2] == 1) { // if every value is new
			SENSORS_ACTIVATE(sht11_sensor);
			temperature[SENSORSCOUNT-1] = (sht11_sensor.value(SHT11_SENSOR_TEMP)/10-396)/10;
			humidity[SENSORSCOUNT-1]    = sht11_sensor.value(SHT11_SENSOR_HUMIDITY)/41;
			SENSORS_DEACTIVATE(sht11_sensor);

			tempAvg = 0;
			humAvg = 0;
			for ( i = 0; i < SENSORSCOUNT; i++ ) {
				tempAvg += temperature[i];
				humAvg += humidity[i];
				if (DEBUGSENSING) printf("G1: tempSum %d, humSum %d \n", tempAvg, humAvg);
			}
			tempAvg =  tempAvg / SENSORSCOUNT;
			humAvg  = humAvg / SENSORSCOUNT;

			for ( i = 0; i < SENSORSCOUNT-1; i++ ) {
				isNew[i] = 0;
			}
			if ( strcmp(emergencyWarning, "") ) {
				printf("%s \nTemp:%dC     Humidity:%d%%\n", emergencyWarning, tempAvg, humAvg );
			} else {
				printf("Temp:%dC     Humidity:%d%%\n", tempAvg, humAvg );
			}
		}
	}
	PROCESS_END();
}

PROCESS_THREAD(WriteEmergencyWarning, ev, data) { //Waiting for emergency message
	PROCESS_BEGIN();
	PROCESS_WAIT_EVENT_UNTIL( ev == serial_line_event_message );

	while(1) {
		while( strcmp ( (char*) data, PASSWORD ) != 0 ) { // Authentication
			printf("Wrong Password!\n");
			PROCESS_WAIT_EVENT_UNTIL( ev == serial_line_event_message );
		}
		printf("Authentication Completed!\nWrite Emergency Warning:\n");
		packetbuf_clear();
		strcpy(emergencyWarning, "");
		PROCESS_WAIT_EVENT_UNTIL( ev == serial_line_event_message );
		if ( strlen((char*)data) > MSGMAXSIZE )
			printf("Error! Message too long (%d char)\n", strlen((char*)data) );
		else {
			strcpy(emergencyWarning, (char*) data );
		}
		PROCESS_WAIT_EVENT_UNTIL( ev == serial_line_event_message );
	}
	PROCESS_END();
}
