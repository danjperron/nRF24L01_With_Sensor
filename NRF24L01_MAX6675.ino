
/*
 Copyright (C) 2015 Daniel Perron
 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.
 
 Read max6675 sensor using a nRF24L01 sensor base on J.Coliz code
 
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



#define MAX_SO_PIN  2
#define MAX_CLK_PIN 6
#define MAX_CS_PIN  9

#define MAX_POWER_PIN 4

#define RF24_POWER_PIN 5



//assuming one sensor per device
// we will use the SKIP command


float celsius;

Sleep sleep;

unsigned long sleepTime=300000;

short  temperature= 32767;


// energy mode

//#define DISABLE_SLEEP

///////////////   radio /////////////////////
// Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 9 & 10 
RF24 radio(8,7);

#define UNIT_ID 0xc5

const uint8_t MasterPipe[6] = {0xe7,0xe7,0xe7,0xe7,0xe7,0};

const uint8_t SensorPipe[6]  = { UNIT_ID,0xc2,0xc2,0xc2,0xc2,0};


// Radio pipe addresses for the 2 nodes to communicate.


// Set up roles to simplify testing 
//boolean role;                                    // The main role variable, holds the current role identifier
//boolean role_ping_out = 1, role_pong_back = 0;   // The two different roles.
unsigned long Count=0;


void StopRadio()
{
  pinMode(RF24_POWER_PIN,OUTPUT);
  digitalWrite(RF24_POWER_PIN,LOW);
}


void StartRadio()
{
  pinMode(RF24_POWER_PIN,OUTPUT);
  digitalWrite(RF24_POWER_PIN,HIGH);
  delay(50);  
  radio.begin();                          // Start up the radio
  radio.setPayloadSize(32);
  radio.setChannel(0x4e);
  radio.setDataRate(RF24_1MBPS);
  radio.setAutoAck(1);                    // Ensure autoACK is enabled
  radio.setRetries(7,3);   // Max delay between retries & number of retries
  radio.enableDynamicPayloads();
  radio.enableAckPayload();
  radio.maskIRQ(true,true,false);
  //role = role_ping_out;                  // Become the primary transmitter (ping out)
  radio.openWritingPipe(MasterPipe);
  radio.openReadingPipe(1,SensorPipe);
  radio.startListening();                 // Start listening
}



void setup() {
  // Set pin for DHT22 power
  StopSensor();
  
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
#define STRUCT_TYPE_DS18B20_DATA 3
#define STRUCT_TYPE_MAX6675_DATA    4


#define STATUS_DATA_VALID 1
#define STATUS_TIME_OUT   2

typedef struct
{
  char header;
  unsigned char structSize;
  unsigned char structType;
  unsigned char txmUnitId;
  unsigned long currentTime;
  unsigned short nextTimeReading;
  unsigned short nextTimeOnTimeOut;
  char Spare[20];
}RcvPacketStruct;


typedef struct
{
   char header;
   unsigned char structSize;
   unsigned char structType;
   unsigned char txmUnitId;
   unsigned char Status;
   unsigned long  stampTime;
   unsigned short voltageA2D;
   unsigned short temperature;
}TxmMAX6675PacketStruct;


unsigned long currentDelay;
unsigned long targetDelay;

RcvPacketStruct RcvData;
TxmMAX6675PacketStruct Txmdata;

unsigned char * pt = (unsigned char *) &RcvData;


unsigned char rcvBuffer[32];
unsigned short nextTimeOnTimeOut = 60;
unsigned short waitTimeOnListen = 60;
unsigned long startTimeOnListen;
unsigned char gotTimeOut=0;


void PrintHex(uint8_t *data, uint8_t length) // prints 16-bit data in hex with leading zeroes
{
       char tmp[32];
       for (int i=0; i<length; i++)
       { 
         sprintf(tmp, "0x%.2X",data[i]); 
         Serial.print(tmp); Serial.print(" ");
       }
}



void StopSensor(void)
{
  pinMode(MAX_POWER_PIN, OUTPUT);
  digitalWrite(MAX_POWER_PIN, LOW);
  pinMode(MAX_CS_PIN,OUTPUT);
  digitalWrite(MAX_CS_PIN,LOW);
  pinMode(MAX_CLK_PIN,OUTPUT);
  digitalWrite(MAX_CLK_PIN,LOW);
  pinMode(MAX_SO_PIN,INPUT);
}

void PowerSensor(void)
{
  pinMode(MAX_POWER_PIN, OUTPUT);
  digitalWrite(MAX_POWER_PIN, HIGH);
  pinMode(MAX_CS_PIN,OUTPUT);
  digitalWrite(MAX_CS_PIN,HIGH);
  pinMode(MAX_CLK_PIN,OUTPUT);
  digitalWrite(MAX_CLK_PIN,LOW);
  pinMode(MAX_SO_PIN,INPUT_PULLUP);
}  
  
  
  


bool readSensor(void)
{
  word data=0;
  int loop;
  
  //CS LOW
  digitalWrite(MAX_CS_PIN,LOW);
  
  for(loop=0;loop<16;loop++)
  {
    // clock up
    digitalWrite(MAX_CLK_PIN,HIGH);
    data <<=1;
    if(digitalRead(MAX_SO_PIN))
        data |=1;
    digitalWrite(MAX_CLK_PIN,LOW);        
  }
  
  // CS HIGH
  digitalWrite(MAX_CS_PIN,HIGH);  
  
    if((data & 2) == 2)
       {
           Serial.println("Sensor Error");
           return 0;
       }
           
    if((data & 4) == 4)
       {
           Serial.println("Probe error");
           return 0;
       }
       
     data >>=3;
     temperature= data;
     celsius = data /4.0;
     Serial.print("temperature :");
     Serial.println(celsius);
 
     return 1;
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
     Txmdata.structSize= sizeof(TxmMAX6675PacketStruct);
     Txmdata.structType=STRUCT_TYPE_INIT_DATA;
     Txmdata.txmUnitId = UNIT_ID;
     Txmdata.stampTime=0;
     Txmdata.Status=0;
     Txmdata.temperature=0;
     Txmdata.voltageA2D=0;
     radio.writeAckPayload(1,&Txmdata,sizeof(TxmMAX6675PacketStruct));
     cycle=ModeListen;
     startTimeOnListen = millis();
   }
   
  if(cycle==ModeListen)
   {
     // put arduino on sleep until we got 
     // an interrupt from RF24 trasnmitter
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
        nextTimeOnTimeOut = RcvData.nextTimeOnTimeOut;
        waitTimeOnListen = 1;
        if(RcvData.nextTimeReading > 50)
          { 
           sleepTime =  RcvData.nextTimeReading*100 - 2000UL; // wake the unit 2 seconds before
           cycle = ModeWait;
          }
          else
           cycle = ModeWriteData;
      }
      else
      {
       deltaTime = (millis() - startTimeOnListen) / 1000;
       if(deltaTime > waitTimeOnListen)
        { // ok we got tim eout
        Serial.print("got time out!");
        Serial.print(deltaTime);
        Serial.print("sec.  Sleep for ");
        Serial.print(nextTimeOnTimeOut);
        Serial.print("sec.\n");
        waitTimeOnListen = 1;
        sleepTime = (unsigned long) nextTimeOnTimeOut * 1000;
        cycle = ModeWait;
        }
      }
   }
   
   
   if(cycle== ModeWait)
     {
       radio.powerDown();
       StopRadio();       

 #ifdef DISABLE_SLEEP
       delay(sleepTime);
#else       
       sleep.pwrDownMode();
       sleep.sleepDelay(sleepTime);
#endif
       cycle = ModeWriteData;
     }
   
   
   if(cycle==ModeWriteData)
    {
     Txmdata.stampTime=RcvData.currentTime;
     Txmdata.header='*';
     Txmdata.structSize= sizeof(TxmMAX6675PacketStruct);
     Txmdata.structType=STRUCT_TYPE_MAX6675_DATA;
     Txmdata.txmUnitId = UNIT_ID;
     Txmdata.stampTime=RcvData.currentTime + (deltaTime / 1000);
     Txmdata.Status=gotTimeOut ? STATUS_TIME_OUT : 0;
     Txmdata.temperature=32767;
     Txmdata.voltageA2D=readVcc();
     

     PowerSensor();

#ifdef DISABLE_SLEEP
       delay(250);
#else       
       sleep.pwrDownMode();
       sleep.sleepDelay(250);
#endif
     
     if(readSensor())
       {
         Txmdata.temperature = temperature;
         Txmdata.Status |= STATUS_DATA_VALID;
       }
       StopSensor();
  
       StartRadio(); 
       radio.writeAckPayload(1,&Txmdata,sizeof(TxmMAX6675PacketStruct));
       cycle=ModeListen;
       startTimeOnListen=millis();
       gotTimeOut=0;     
    }
      
}
