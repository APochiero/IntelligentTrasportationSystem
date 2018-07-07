/*
 * G1.c
 *
 *  Created on: Jul 2, 2018
 *      Author: Amedeo Pochiero
 */

#define GROUND
#define G1

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "contiki.h"
#include "net/rime/rime.h"
#include "dev/serial-line.h"
#include "../Macros.h"

PROCESS(Init, "Initialization");
PROCESS(TrafficScheduler, "TrafficScheduler");
PROCESS(SensingSink, "SensingSink");
PROCESS(WriteEmergencyWarning, "WriteEmergencyWarning");

AUTOSTART_PROCESSES( &Init );

static void send_broadcast(struct broadcast_conn *c, int status, int num_tx) {
  if ( DEBUG ) printf("broadcast message sent. Status %d. For this packet, this is transmission number %d\n", status, num_tx);
}

static process_event_t sensing_ev;
static void recv_runicast(struct runicast_conn *c, const linkaddr_t *from, uint8_t seqno){
	process_post(&SensingSink, sensing_ev, packetbuf_dataptr());
}
static void sent_runicast(struct runicast_conn *c, const linkaddr_t *to, uint8_t retransmissions){
	if (DEBUG ) printf("G1 successfully linked to %d.%d, retransmissions %d\n", to->u8[0], to->u8[1], retransmissions);
  if ( process_is_running(&Init) ) {
	  process_post(&Init, PROCESS_EVENT_CONTINUE, NULL);
  }
}

#include "../libits.h"

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

	if ( DEBUG ) printf("Listening on channels 144\n");
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
		if (DEBUG ) printf("G1: Received from %d temp %d, hum %d \n", index, temperature[index], humidity[index]);

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
				if (DEBUG) printf("G1: tempSum %d, humSum %d \n", tempAvg, humAvg);
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
