#include <RF24_config.h>
#include <RF24.h>
#include <SPI.h>
#include "printf.h"

/*
  Copyright (C) 2020 Daniel Perron
  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  version 2 as published by the Free Software Foundation.

  Read Moisture sensor  via nRF24L01
*/


// version DEMO 2.0  June 2021
// Read capactive sensore and output result packet to RF24

// UNIT_ID     ID number of the device for nRF24L01
#define UNIT_ID 0x01


// Radio pipe addresses for the 2 nodes to communicate.
const uint64_t serverPipe = 0xE0E0E0E001LL;
const uint64_t sensorPipe = 0xA5A5A5A500LL | UNIT_ID;


#define   SERIAL_DEBUG

// PIN DEFINITION

#define RF24_CE_PIN  8
//#define RF24_CSN_PIN 10 
#define RF24_CSN_PIN 7

#define RF24_DATA_RATE RF24_250KBPS

#define RADIO_CHANNEL 80


// sensor input variable
short analog0;  // Humidity sensor
short vcc;      // voltage at VCC of the arduino


int _second = millis()/1000;


///////////////   radio /////////////////////
// Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 9 & 10
RF24 radio(RF24_CE_PIN,RF24_CSN_PIN);


void StartRadio()
{
#ifdef SERIAL_DEBUG
  Serial.print("*** START RADIO ***\n");
#endif

  delay(50);
  radio.begin();                          // Start up the radio
  radio.setChannel(RADIO_CHANNEL);
  radio.setDataRate(RF24_DATA_RATE);
  radio.setPALevel(RF24_PA_MIN);
  radio.setPayloadSize(32);
  radio.enableDynamicPayloads();
  radio.openReadingPipe(1, sensorPipe);
  radio.openWritingPipe(serverPipe);
  Serial.println("print details");
#ifdef SERIAL_DEBUG
  radio.stopListening();
  radio.printDetails();
#endif
  Serial.println("end details");
  //radio.startistening();
  
}


void setup() {
  Serial.begin(57600);
  printf_begin();
#ifdef SERIAL_DEBUG
  Serial.println("==== RF24 Humidity Sensor DEMO 2 ====");
#endif
  StartRadio();
}

#define STRUCT_TYPE_MOISTURE_DATA 7
#define STATUS_DATA_VALID  1

typedef struct
{
  char header;
  unsigned char structSize;
  unsigned char structType;
  unsigned char txmUnitId;
  unsigned char Status;
  unsigned long  stampTime;
  unsigned short voltageA2D;
  unsigned short analog0;
  unsigned short spare;
  unsigned long  frequency;
} TxmMoisturePacketStruct;

TxmMoisturePacketStruct Txmdata;

void PrintHex(uint8_t *data, uint8_t length) // prints 16-bit data in hex with leading zeroes
{
#ifdef SERIAL_DEBUG
  char tmp[32];
  for (int i = 0; i < length; i++)
  {
    sprintf(tmp, "0x%.2X", data[i]);
    Serial.print(tmp); Serial.print(" ");
  }
#endif
}



bool readSensor()
{
  //ok read sensor and nRF24L01 batteries
  analog0 = analogRead(A0);
  vcc =   readVcc();

#ifdef SERIAL_DEBUG
  Serial.println("Get data");
  Serial.print("Moisture:");
  Serial.println(analog0);
  Serial.print("VCC:");
  Serial.println(vcc);
#endif
  return (1);
}

//From http://provideyourown.com/2012/secret-arduino-voltmeter-measure-battery-voltage/
unsigned short readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
  ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  ADMUX = _BV(MUX3) | _BV(MUX2);
#else
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif

  delay(5); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA, ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both

  long result = (high << 8) | low;

#define VOLTAGE_FACTOR 1125300L
//#define VOLTAGE_FACTOR   1186125L
  result = VOLTAGE_FACTOR / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return (unsigned short) result; // Vcc in millivolts
}


void loop(void) {

  //  read Sensor
  readSensor();


  // fill transmit buffer
  Txmdata.header = '*';
  Txmdata.structSize = sizeof(TxmMoisturePacketStruct);
  Txmdata.structType = STRUCT_TYPE_MOISTURE_DATA;
  Txmdata.txmUnitId = UNIT_ID;
  Txmdata.stampTime = 0;
  Txmdata.voltageA2D = vcc;
  Txmdata.analog0 = analog0;
  Txmdata.spare = 0;
  Txmdata.frequency = 0;
  Txmdata.Status = STATUS_DATA_VALID;
  radio.stopListening();
  radio.openWritingPipe(serverPipe);
  radio.write(&Txmdata,Txmdata.structSize);
  while(_second == (millis()/1000));
  _second=millis()/1000;
}
