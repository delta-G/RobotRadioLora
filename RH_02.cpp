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

StreamParser parser(&Serial, START_OF_PACKET, END_OF_PACKET, handleSerialCommand);

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

	// Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
	if (!radio.setFrequency(RF95_FREQ)) {
		DEBUG("setFrequency failed");
		while (1)
			;
	}
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
	parser.run();   // listen to serial
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
	// If anything is coming in on radio, just ship it out to serial
	// the serial parsers on either end can handle it however it comes.
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


void handleRadioCommand(char* aCommand){
	// Right now just ship everything to RMB
	connectedToBase = true;  //we received a formatted command we must be connected
	if (rmbActive) {
		Serial.print(aCommand);
	}
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
