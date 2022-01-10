/*

RobotRadioLora  --  runs on Arduino Nano and handles communication over LoRa
                    for RobotMainBrain

     Copyright (C) 2017  David C.

     This program is free software: you can redistribute it and/or modify
     it under the terms of the GNU General Public License as published by
     the Free Software Foundation, either version 3 of the License, or
     (at your option) any later version.

     This program is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of
     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
     GNU General Public License for more details.

     You should have received a copy of the GNU General Public License
     along with this program.  If not, see <http://www.gnu.org/licenses/>.

     */


#include "RobotRadioLora.h"

//#define MYDEBUG_OUT Serial

#ifdef MYDEBUG_OUT
#define MYDEBUG(x) MYDEBUG_OUT.println(x)
#else
#define MYDEBUG(x)
#endif

uint8_t heartPins[3] = {3,5,6};
uint16_t heartDelay[3] = {0,0,0};
uint32_t lastHeart[3] = {0,0,0};


boolean rmbActive = false;
boolean connectedToBase = false;

enum States {BOOTUP, WAITING_ON_RMB, WAITING_ON_BASE, RUNNING} bootState;

RH_RF95 radio(RFM95_CS, RFM95_INT);

StreamParser parser(&Serial, START_OF_PACKET, END_OF_PACKET, handleSerialCommand);


uint32_t lastCommandTime;
uint32_t commandTimeout = 20000;
boolean blackoutReported = false;



void setup() {
	initRadio();

	for (int i=0; i<3; i++){
		pinMode(heartPins[i], OUTPUT);
		digitalWrite(heartPins[i], LOW);
	}

	//  beat the light a few times to let us know the program came on.
	for (int i = 0; i < 3; i++) {
		digitalWrite(heartPins[i], HIGH);
		delay(50);
		digitalWrite(heartPins[i], LOW);
		delay(50);
	}

	Serial.begin(ROBOT_COM_BAUD);
	delay(100);

	digitalWrite(6, HIGH);

	resetRadio();

	digitalWrite(6, LOW);

	//  beat the light a few times to let us know the radio came on.
	for (int i = 0; i < 3; i++) {
		digitalWrite(heartPins[i], HIGH);
		delay(50);
		digitalWrite(heartPins[i], LOW);
		delay(50);
	}


	parser.setRawCallback(handleRawSerial);
	setHeartDelay(500,0,0);

}

void loop()
{
	switch(bootState){
	case BOOTUP:
	case WAITING_ON_RMB:
		if(rmbActive){
			Serial.print(F(COM_START_STRING));
			bootState = WAITING_ON_BASE;
			setHeartDelay(0,500,0);
		}
		break;
	case WAITING_ON_BASE:
		if(connectedToBase){
			Serial.print(F(COM_CONNECT_STRING));
			setHeartDelay(0,0,2000);
			lastCommandTime = millis();  // reset the command timer
			bootState = RUNNING;
		}
		break;
	case RUNNING:
		// if we lose contact, report to main brain and flashy light
		if (millis() - lastCommandTime >= commandTimeout) {
			if (!blackoutReported) {
				Serial.print(F("<LOST_COM>"));
				blackoutReported = true;
				setHeartDelay(200,0,0);
			}
		}
		break;
	default:
		//freak out , we shouldn't be here!
		setHeartDelay(50, 0, 0);
	}

	listenToRadio(); // handle the radio
	parser.run();   // listen to serial
	handleOutput();
	heartbeat();   // beat the light
}

void setHeartDelay(uint16_t aRed, uint16_t aGreen, uint16_t aBlue){
	heartDelay[0] = aRed;
	heartDelay[1] = aGreen;
	heartDelay[2] = aBlue;
}




void handleRadioCommand(char* aCommand){
	// Right now just ship everything to RMB
	connectedToBase = true;  //we received a formatted command we must be connected
	if(!strcmp(aCommand, "<LGO>")){
		rmbActive = true;
	} else if(aCommand[1] == 'l'){
		delay(250);
		handleConfigString(aCommand);
		delay(500);
	} else if(aCommand[1] == 'P'){
		int rvl = atoi((const char*)(aCommand + 2));
		char resp[10];
		snprintf(resp, 10, "<p%i>", rvl);
		sendToRadio(resp);
	} else if (rmbActive) {
		Serial.print(aCommand);
	}
	lastCommandTime = millis();
	//  if we had lost contact
	if(blackoutReported){
		blackoutReported = false;
		setHeartDelay(0,0,2000);
	}
}

void handleRawRadio(uint8_t *p) {
	uint8_t numBytes = p[2];
	//  If properly formatted message
	if ((numBytes < RADIO_BUFFER_SIZE) && (p[numBytes - 1] == '>')) {
		connectedToBase = true; //we received a formatted raw message we must be connected
		if (rmbActive) {
			for (uint8_t i = 0; i < numBytes; i++) {
				Serial.write(p[i]);
			}
		}
		lastCommandTime = millis();
		//  if we had lost contact
		if (blackoutReported) {
			blackoutReported = false;
			setHeartDelay(0,0,2000);
		}
	}
}

void handleRawSerial(char *p) {

	int numBytes = p[2];

	if (p[1] == 0x13 && numBytes == ROBOT_DATA_DUMP_SIZE) {
		uint8_t snr = (uint8_t) (radio.lastSNR());
		int rs = radio.lastRssi();
		uint8_t rssi = (uint8_t) (abs(rs));
		p[ROBOT_DATA_DUMP_SIZE - 5] = snr;
		p[ROBOT_DATA_DUMP_SIZE - 4] = rssi;
	}

	addToHolding((uint8_t*) p, numBytes);

	flush();
}

void handleSerialCommand(char *aCommand) {
	if (strcmp(aCommand, "<FFF>") == 0){
		flush();
		return;
	}
	else if (strcmp(aCommand, RMB_STARTUP_STRING) == 0) {
		rmbActive = true;
//	}
//	else if (strstr(aCommand, "<RRMMBB  HHBBooRR>") != NULL) {
//		digitalWrite(heartPins[0], HIGH);
//		digitalWrite(heartPins[1], HIGH);
//		digitalWrite(heartPins[2], HIGH);
//		// WHITE light and lock up, we found it...
//		while (1)
//			;
	} else if (aCommand[1] == 'l') {
		addToHolding(aCommand);
		flush();
		handleConfigString(aCommand);
	} else if (connectedToBase) {
		addToHolding(aCommand);
	}
}


void heartbeat(){
	uint32_t cur = millis();
	for(uint8_t i=0; i<3; i++){
		if(heartDelay[i] == 0){
			digitalWrite(heartPins[i], LOW);
		} else if(heartDelay[i] == 1){
			digitalWrite(heartPins[i], HIGH);
		} else if(cur - lastHeart[i] >= heartDelay[i]){
			digitalWrite(heartPins[i], !digitalRead(heartPins[i]));
			lastHeart[i] = cur;
		}
	}
}

