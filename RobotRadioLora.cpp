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

//#define DEBUG_OUT Serial

#ifdef DEBUG_OUT
#define DEBUG(x) DEBUG_OUT.println(x)
#else
#define DEBUG(x)
#endif


#define HOLDING_BUFFER_SIZE 248

#define RFM95_CS 10
#define RFM95_RST 9
#define RFM95_INT 2

#define RF95_FREQ 915.0

#define MAX_MESSAGE_SIZE_RH RH_RF95_MAX_MESSAGE_LEN

const uint8_t heartBeatPin = 6;
unsigned int heartBeatDelay = 100;

boolean rmbActive = false;
boolean connectedToBase = false;

enum States {BOOTUP, WAITING_ON_RMB, WAITING_ON_BASE, RUNNING} bootState;

RH_RF95 radio(RFM95_CS, RFM95_INT);

StreamParser parser(&Serial, START_OF_PACKET, END_OF_PACKET, handleSerialCommand);


uint32_t lastCommandTime;
uint32_t commandTimeout = 1000000;
boolean blackoutReported = false;

uint32_t lastFlushTime;
uint32_t maxFlushInterval = 10000;


uint8_t holdingBuffer[HOLDING_BUFFER_SIZE];
uint8_t holdingSize = 0;

//boolean flushOnNextRaw = false;


void setup() {
	pinMode(RFM95_RST, OUTPUT);
	digitalWrite(RFM95_RST, HIGH);

	pinMode(heartBeatPin, OUTPUT);

	//  beat the light a few times to let us know the program came on.
	for (int i = 0; i < 3; i++) {
		digitalWrite(heartBeatPin, HIGH);
		delay(50);
		digitalWrite(heartBeatPin, LOW);
		delay(50);
	}

	Serial.begin(ROBOT_COM_BAUD);
	delay(100);

	// manual reset
	digitalWrite(RFM95_RST, LOW);
	delay(10);
	digitalWrite(RFM95_RST, HIGH);
	delay(10);

	while (!radio.init()) {
		DEBUG("LoRa radio init failed");
		while (1)
			;
	}
	DEBUG("LoRa radio init OK!");
	digitalWrite(heartBeatPin, HIGH);

	// Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
	if (!radio.setFrequency(RF95_FREQ)) {
		DEBUG("setFrequency failed");
		while (1)
			;
	}
			digitalWrite(heartBeatPin, LOW);
	DEBUG("Set Freq to: ");
	DEBUG(RF95_FREQ);

	radio.setTxPower(23, false);

	//  beat the light a few times to let us know the radio came on.
	for (int i = 0; i < 3; i++) {
		digitalWrite(heartBeatPin, HIGH);
		delay(50);
		digitalWrite(heartBeatPin, LOW);
		delay(50);
	}


	parser.setRawCallback(handleRawSerial);

}

void loop()
{
	switch(bootState){
	case BOOTUP:
	case WAITING_ON_RMB:
		if(rmbActive){
			Serial.print(COM_START_STRING);
			bootState = WAITING_ON_BASE;
			heartBeatDelay = 500;
		}
		break;
	case WAITING_ON_BASE:
		if(connectedToBase){
			Serial.print(COM_CONNECT_STRING);
			heartBeatDelay = 2000;
			lastCommandTime = millis();  // reset the command timer
			bootState = RUNNING;
		}
		break;
	case RUNNING:
		// if we lose contact, report to main brain and flashy light
		if (millis() - lastCommandTime >= commandTimeout) {
			if (!blackoutReported) {
				Serial.print("<LOST_COM>");
				blackoutReported = true;
				heartBeatDelay = 200;
			}
		}
		if (holdingSize == 0){
			lastFlushTime = millis();  // don't start timer if we don't have anything to send.
		}
		if (millis() - lastFlushTime >= maxFlushInterval){
			flush();
		}
		break;
	default:
		//freak out , we shouldn't be here!
		heartBeatDelay = 50;
	}

	listenToRadio(); // handle the radio
	parser.run();   // listen to serial
	heartBeat();   // beat the light
}


void addToHolding(uint8_t* p, uint8_t aSize){
	if(HOLDING_BUFFER_SIZE - holdingSize < aSize){
		//  Not enough room so clear the buffer now
		flush();
	}
	memcpy(holdingBuffer + holdingSize, p, aSize);
	holdingSize += aSize;
}

void addToHolding(char* p){
	addToHolding((uint8_t*)p, strlen(p));
}


void sendToRadio(char* p){
	sendToRadio((uint8_t*)p, strlen(p));
}

void sendToRadio(uint8_t* p, uint8_t aSize){
	radio.send(p, aSize);
	radio.waitPacketSent();
}

void flush(){
	sendToRadio(holdingBuffer, holdingSize);
	holdingSize = 0;
	lastFlushTime = millis();
}

void listenToRadio() {
	if (radio.available()) {

		uint8_t buf[MAX_MESSAGE_SIZE_RH];
		uint8_t len = sizeof(buf);

		if (radio.recv(buf, &len)) {
			processRadioBuffer(buf, len);
		}
	}

}


void processRadioBuffer(uint8_t* aBuf, uint8_t aLen){

	static boolean receiving = false;
	static char commandBuffer[100];
	static int index;

//	flushOnNextRaw = true;
	uint8_t len = aLen;
	if (len > MAX_MESSAGE_SIZE_RH) {
		len = MAX_MESSAGE_SIZE_RH;
	}

	// radio.racv doesn't put any null terminator, so we can't use
	// string functions, have to scroll through and pick stuff out.
	for(int i=0; i<len; i++){
		char c = aBuf[i];

		if (c == START_OF_PACKET) {
			if ((aBuf[i + 1] >= 0x11) && (aBuf[i + 1] <= 0x14)) {
				handleRawRadio(&aBuf[i]);
				i += (aBuf[i + 2] - 1);
				continue;
			}

			receiving = true;
			index = 0;
			commandBuffer[0] = 0;
		}
		if (receiving) {
			commandBuffer[index] = c;
			commandBuffer[++index] = 0;
			if(index >= 100){
				index--;
			}
			if(c == END_OF_PACKET){
				receiving = false;
				handleRadioCommand(commandBuffer);
			}
		}
	}
}

void handleRadioCommand(char* aCommand){
	// Right now just ship everything to RMB
	connectedToBase = true;  //we received a formatted command we must be connected
	if (rmbActive) {
		Serial.print(aCommand);
	}
	lastCommandTime = millis();
	//  if we had lost contact
	if(blackoutReported){
		blackoutReported = false;
		heartBeatDelay = 2000;
	}
}

void handleRawRadio(uint8_t *p) {
	uint8_t numBytes = p[2];
	//  If properly formatted message
	if ((numBytes < 100) && (p[numBytes - 1] == '>')) {
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
			heartBeatDelay = 2000;
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
	if (strcmp(aCommand, RMB_STARTUP_STRING) == 0) {
		rmbActive = true;
	}
	if (connectedToBase) {
		addToHolding(aCommand);
	}
}

void heartBeat(){

	static unsigned long pm = millis();
	unsigned long cm = millis();

	if(cm - pm >= heartBeatDelay){
		digitalWrite(heartBeatPin, !digitalRead(heartBeatPin));
		pm = cm;
	}
}
