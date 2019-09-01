#ifndef _RH_02_H_
#define _RH_02_H_
#include "Arduino.h"

#include <SPI.h>
#include <RH_RF95.h>

#include <RobotSharedDefines.h>

#include <StreamParser.h>


void setup();
void loop();

void sendToRadio(char*);
void listenToRadio();
void processRadioBuffer(uint8_t*);
void handleRadioCommand(char*);

void handleSerialCommand(char*);

void controllerDataToASCII(uint8_t*);

void heartBeat();

#endif /* _RH_02_H_ */
