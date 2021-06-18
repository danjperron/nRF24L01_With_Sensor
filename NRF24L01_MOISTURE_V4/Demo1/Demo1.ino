#include <RF24_config.h>
#include <RF24.h>



/*
  Copyright (C) 2020 Daniel Perron
  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  version 2 as published by the Free Software Foundation.

  Read Moisture sensor  via nRF24L01

*/




// version DEMO 1.0  June 2021
// strip down version demo


#include <SPI.h>
#include "printf.h"


// UNIT_ID     ID number of the device for nRF24L01
#define UNIT_ID 0xA5


// Radio pipe addresses for the 2 nodes to communicate.
const uint64_t serverPipe = 0xE0E0E0E001LL;
const uint64_t sensorPipe = 0xA5A5A5A500LL | UNIT_ID;



#define   SERIAL_DEBUG



#define RF24_CE_PIN  8
//#define RF24_CSN_PIN 10 
#define RF24_CSN_PIN 7

#define RF24_DATA_RATE RF24_250KBPS

#define RADIO_CHANNEL 80

int counter= 0;
char info_buffer[32];

///////////////   radio /////////////////////
// Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 9 & 10
RF24 radio(RF24_CE_PIN,RF24_CSN_PIN);


void StartRadio()
{
#ifdef SERIAL_DEBUG
  Serial.print("*** START RADIO ***\n");
#endif
  radio.begin();
  radio.setPALevel(RF24_PA_MIN);
  radio.setChannel(RADIO_CHANNEL);
  radio.setDataRate(RF24_DATA_RATE);
  radio.openReadingPipe(1, sensorPipe);
  radio.openWritingPipe(serverPipe);
  radio.enableDynamicPayloads();
  radio.powerUp();
  radio.startListening();
  
#ifdef SERIAL_DEBUG
  radio.stopListening();
  radio.printDetails();
#endif



}


void setup() {
  Serial.begin(57600);
  printf_begin();
#ifdef SERIAL_DEBUG
  printf("\n\rRF24 Demo 1\n\r");
#endif
  StartRadio();
}




void loop(void) {
  radio.stopListening();
  radio.openWritingPipe(serverPipe);
  counter++;
  sprintf(info_buffer,"Allo pour la %d fois!",counter);
  Serial.println(info_buffer);
  Serial.println(radio.write(info_buffer, strlen(info_buffer)));
  delay(1000);
}
