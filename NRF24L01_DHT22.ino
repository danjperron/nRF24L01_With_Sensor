
/*
 Copyright (C) 2015 Daniel Perron
 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.
 
 Read DHT22 sensor using a nRF24L01 sensor base on J.Coliz code
 
*/



/*
 Copyright (C) 2011 J. Coliz <maniacbug@ymail.com>

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.
//2014 - TMRh20 - Updated along with Optimized RF24 Library fork
 */

/**
 * Example for Getting Started with nRF24L01+ radios. 
 *
 * This is an example of how to use the RF24 class to communicate on a basic level.  Write this sketch to two 
 * different nodes.  Put one of the nodes into 'transmit' mode by connecting with the serial monitor and
 * sending a 'T'.  The ping node sends the current time to the pong node, which responds by sending the value
 * back.  The ping node can then see how long the whole cycle took. 
 * Note: For a more efficient call-response scenario see the GettingStarted_CallResponse.ino example.
 * Note: When switching between sketches, the radio may need to be powered down to clear settings that are not "un-set" otherwise
 */


#include <SPI.h>
#include <Sleep_n0m1.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"
#include "dht.h"


#define DHT_PIN 2
#define DHT_POWER_PIN 4


//DHT22 class
dht DHT;
Sleep sleep;

unsigned long sleepTime=300000;

short  temperature= 32767;
unsigned short  humidity  = 32767;


// energy mode





// Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 9 & 10 
RF24 radio(8,7);

#define UNIT_ID 0xc3


const uint64_t pipes[2] = { 0xc2c2c2c2c3 , 0xe7e7e7e7e7 };              // Radio pipe addresses for the 2 nodes to communicate.


// Set up roles to simplify testing 
boolean role;                                    // The main role variable, holds the current role identifier
boolean role_ping_out = 1, role_pong_back = 0;   // The two different roles.
unsigned long Count=0;



void StartRadio()
{
  // Setup and configure rf radio
  //radio.setChannel(0x60);
  radio.begin();                          // Start up the radio
  radio.setPayloadSize(32);
  radio.setChannel(0x4e);
  radio.setAutoAck(1);                    // Ensure autoACK is enabled
  radio.setRetries(15,15);   // Max delay between retries & number of retries
  radio.enableDynamicPayloads();
  radio.enableAckPayload();
  radio.maskIRQ(true,true,false);
  //role = role_ping_out;                  // Become the primary transmitter (ping out)
  radio.openWritingPipe(pipes[1]);
  radio.openReadingPipe(1,pipes[0]);
  radio.startListening();                 // Start listening
}



void setup() {
  // Set pin for DHT22 power
  pinMode(DHT_POWER_PIN, OUTPUT);
  digitalWrite(DHT_POWER_PIN, LOW);
  
  Serial.begin(57600);
  printf_begin();
  printf("\n\rRF24/examples/GettingStarted/\n\r");
  printf("*** PRESS 'T' to begin transmitting to the other node\n\r");

  StartRadio();
  radio.stopListening();
  radio.printDetails();                   // Dump the configuration of the rf unit for debugging
  radio.startListening();
}





enum cycleMode {ModeInit,ModeListen,ModeWriteData,ModeWait};

cycleMode cycle= ModeInit;


#define STRUCT_TYPE_GETDATA    0
#define STRUCT_TYPE_INIT_DATA  1
#define STRUCT_TYPE_DHT22_DATA 2


typedef struct
{
  char header;
  unsigned char structSize;
  unsigned char structType;
  unsigned char txmUnitId;
  unsigned long currentTime;
  unsigned short nextTimeReading;
  char Spare[22];
}RcvPacketStruct;


typedef struct
{
   char header;
   unsigned char structSize;
   unsigned char structType;
   unsigned char txmUnitId;
   unsigned long  stampTime;
   unsigned char  valid;
   unsigned short voltageA2D;
   unsigned short temperature;
   unsigned short humidity;
}TxmDHT22PacketStruct;


unsigned long currentDelay;
unsigned long targetDelay;

RcvPacketStruct RcvData;
TxmDHT22PacketStruct Txmdata;

unsigned char * pt = (unsigned char *) &RcvData;


unsigned char rcvBuffer[32];


void PrintHex(uint8_t *data, uint8_t length) // prints 16-bit data in hex with leading zeroes
{
       char tmp[32];
       for (int i=0; i<length; i++)
       { 
         sprintf(tmp, "0x%.2X",data[i]); 
         Serial.print(tmp); Serial.print(" ");
       }
}

bool readSensor(void)
{

  // power Sensor UP
  digitalWrite(DHT_POWER_PIN, HIGH);
  // Wait 2 sec
//  delay(2000);
   sleep.pwrDownMode();
   sleep.sleepDelay(2000);

  // Now let's read the sensor twice
  // since the first one will be bad
  
  DHT.read(DHT_PIN);
//  delay(1000);
   sleep.pwrDownMode();
   sleep.sleepDelay(1000);

  int rcode = DHT.read(DHT_PIN);
  
  // power off DHT22
  
  digitalWrite(DHT_POWER_PIN,LOW);
  
  return (rcode == DHTLIB_OK);
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
 
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring
 
  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both
 
  long result = (high<<8) | low;
 
  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return (unsigned short) result; // Vcc in millivolts
}


void loop(void){
int loop;
unsigned long deltaTime;  
/****************** Ping Out Role ***************************/
  
  if(cycle==ModeInit)
   {
     Count++;
     Txmdata.header='*';
     Txmdata.structSize= sizeof(TxmDHT22PacketStruct);
     Txmdata.structType=STRUCT_TYPE_INIT_DATA;
     Txmdata.txmUnitId = UNIT_ID;
     Txmdata.stampTime=0;
     Txmdata.valid=0;
     Txmdata.temperature=0;
     Txmdata.humidity=0;
     Txmdata.voltageA2D=0;
     radio.writeAckPayload(1,&Txmdata,sizeof(TxmDHT22PacketStruct));
     cycle=ModeListen;
   }
   
  if(cycle==ModeListen)
   {
     // put arduino on sleep until we got 
     // an interrupt from RF24 trasnmitter
     sleep.pwrDownMode();
     sleep.sleepDelay(100);

     if(radio.available()) 
      {
        int rcv_size= radio.getDynamicPayloadSize();
        radio.read( &RcvData,rcv_size);
        Serial.print("T:" );
        Serial.print(RcvData.currentTime);
        Serial.print(" Next reading in (1/10 sec): ");
        Serial.print(RcvData.nextTimeReading);
        Serial.print("\n");
        PrintHex(pt,rcv_size);
        Serial.print("\n");        
        currentDelay = millis();
        if(RcvData.nextTimeReading > 50)
          { 
           sleepTime =  RcvData.nextTimeReading*100 - 5000UL;
           cycle = ModeWait;
          }
          else
           cycle = ModeWriteData;
      }
   }
   
   
   if(cycle== ModeWait)
     {
       radio.powerDown();       
       sleep.pwrDownMode();
       sleep.sleepDelay(sleepTime);
       cycle = ModeWriteData;
     }
   
   
   if(cycle==ModeWriteData)
    {
      
     Txmdata.valid=1;
     Txmdata.stampTime=RcvData.currentTime;
     Txmdata.header='*';
     Txmdata.structSize= sizeof(TxmDHT22PacketStruct);
     Txmdata.structType=STRUCT_TYPE_DHT22_DATA;
     Txmdata.txmUnitId = UNIT_ID;
     Txmdata.stampTime=RcvData.currentTime + (deltaTime / 1000);
     Txmdata.valid=0;
     Txmdata.temperature=32767;
     Txmdata.humidity=32767;
     Txmdata.voltageA2D=readVcc();
     

     if(readSensor())
       {
         Txmdata.temperature = (short) floor(DHT.temperature * 10.0);
         Txmdata.humidity = (short) floor(DHT.humidity);
         Txmdata.valid = 1;
       }
      
     StartRadio(); 
     radio.writeAckPayload(1,&Txmdata,sizeof(TxmDHT22PacketStruct));
     cycle=ModeListen;     
    }
      
}
