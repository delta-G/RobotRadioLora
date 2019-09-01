#include "RH_02.h"

//#define DEBUG_OUT Serial

#ifdef DEBUG_OUT
#define DEBUG(x) DEBUG_OUT.println(x)
#else
#define DEBUG(x)
#endif


#define RFM95_CS 10
#define RFM95_RST 9
#define RFM95_INT 2

#define RF95_FREQ 915.0

#define MAX_MESSAGE_SIZE_RH RH_RF95_MAX_MESSAGE_LEN

const uint8_t heartBeatPin = 5;
unsigned int heartBeatDelay = 100;

boolean rmbActive = false;
boolean connectedToBase = false;

enum States {BOOTUP, WAITING_ON_RMB, WAITING_ON_BASE, RUNNING} bootState;

RH_RF95 radio(RFM95_CS, RFM95_INT);

//StreamParser parser(&Serial, START_OF_PACKET, END_OF_PACKET, handleSerialCommand);

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
			bootState = RUNNING;
		}
		break;
	case RUNNING:
		// everything already needs to happen every loop
		// I'm sure there will be some stuff to put here eventually

		break;
	default:
		//freak out , we shouldn't be here!
		heartBeatDelay = 50;
	}

	listenToRadio(); // handle the radio
//	parser.run();   // listen to serial
	handleSerial();
	heartBeat();   // beat the light
}

void sendToRadio(char* p){
//	DEBUG("Sending ");
//	DEBUG(p);
	uint8_t len = strlen(p);
	radio.send((uint8_t*)p, len);
	radio.waitPacketSent();
}


void listenToRadio() {
	if (radio.available()) {

		uint8_t buf[MAX_MESSAGE_SIZE_RH];
		uint8_t len = sizeof(buf);

		if (radio.recv(buf, &len)) {
			processRadioBuffer(buf);
		}
	}

}


void processRadioBuffer(uint8_t* aBuf){

	static boolean receiving = false;
	static char commandBuffer[100];
	static int index;

	// radio.racv doesn't put any null terminator, so we can't use
	// string functions, have to scroll through and pick stuff out.
	for(int i=0; i<MAX_MESSAGE_SIZE_RH; i++){
		char c = aBuf[i];

		if(c == START_OF_PACKET){
			if(aBuf[i+1] == 0x14 || aBuf[i+1] == 0x0D){
				controllerDataToASCII(&aBuf[i+1]);
				i += 15;
				continue;
			}
			receiving = true;
			index = 0;
			commandBuffer[0] = 0;
		}
		if(receiving){
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

void controllerDataToASCII(uint8_t* aBuf){

	char retBuf[32] = "<X";

	for(int i = 0; i<14; i++){
		sprintf(&retBuf[2+(2*i)], "%0.2X", aBuf[i]);
	}
	retBuf[30] = '>';
	retBuf[31] = 0;
	handleRadioCommand(retBuf);
}

void handleRadioCommand(char* aCommand){
	// Right now just ship everything to RMB
	connectedToBase = true;  //we received a formatted command we must be connected
	if (rmbActive) {
		Serial.print(aCommand);
	}
}

void handleSerial(){

	static char buffer[100];
	static boolean receiving = false;
	static int index = 0;

	if(Serial.available()){
		char c = Serial.read();

		if(c == START_OF_PACKET){
			if(Serial.peek() == 0x13){
				handleRawDataDump();
				return;
			}
			receiving = true;
			index = 0;
			buffer[index] = 0;
		}
		if (receiving){
			buffer[index] = c;
			buffer[++index] = 0;
			if (index >= 100){
				index--;
			}
			if(c == END_OF_PACKET){
				receiving = false;
				handleSerialCommand(buffer);
			}
		}
	}
}

void handleRawDataDump(){
	uint8_t buffer[14];
	buffer[0] = '<';  // we read this already but just peeked the 0x13
	while(Serial.available() < 13);  // block and catch the whole thing.
	for(int i=1; i<14; i++){
		buffer[i] = Serial.read();
	}
	radio.send(buffer, 14);
	radio.waitPacketSent();
}

void handleSerialCommand(char *aCommand) {
	// Right now just ship it all over the radio
	if (strcmp(aCommand, RMB_STARTUP_STRING) == 0) {
		rmbActive = true;
	}
	if (connectedToBase) {
		sendToRadio(aCommand);
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
