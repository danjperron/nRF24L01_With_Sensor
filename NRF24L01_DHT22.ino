
/*
 Copyright (C) 2015 Daniel Perron
 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.
 
 Read DHT22 sensor using a nRF24L01 sensor base on J.Coliz code
 
March 29,2015
Version 0.01
DTH22 doesn't work very well under 2.7V. Supply will be 4.5V 
- Put Back LE33 regulator but the power input  will be on one Arduino Pin.


Arduino Pin


0 - Serial RX
1 - Serial TX
2 - DHT22 Data
3 - nRF24L01 IRQ  (not used)DHT22 POWER 
4 - DHT22 Power
5 - nRF24L01 Power (to LE33cz regulator).
6 -
7 - nRF24L01 CS
8 - nRF24L01 CE
9 - 
10 -
11 - nRF24L01 MOSI
12 - nRF24L01 MISO
13 - nRF24L01 SCK




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

#define RF24_POWER_PIN 5
//DHT22 class
dht DHT;
Sleep sleep;

unsigned long sleepTime=300000;

short  temperature= 32767;
unsigned short  humidity  = 32767;


// energy mode


// Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 9 & 10 
RF24 radio(8,7);

#define UNIT_ID 0xc8

const uint8_t MasterPipe[5] = {0xe7,0xe7,0xe7,0xe7,0xe7};

const uint8_t SensorPipe[5]  = { UNIT_ID,0xc2,0xc2,0xc2,0xc2};


// Radio pipe addresses for the 2 nodes to communicate.


// Set up roles to simplify testing 
//boolean role;                                    // The main role variable, holds the current role identifier
//boolean role_ping_out = 1, role_pong_back = 0;   // The two different roles.
unsigned long Count=0;



#define        WD_CALIB_COUNT   720
unsigned short WDCalibrationCycle=0;


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
  // Setup and configure rf radio
  radio.begin();                          // Start up the radio
  radio.setPayloadSize(32);
  radio.setChannel(0x4e);
  radio.setAutoAck(1);                    // Ensure autoACK is enabled
  radio.setRetries(7,4);   // Max delay between retries & number of retries
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
  pinMode(DHT_POWER_PIN, OUTPUT);
  digitalWrite(DHT_POWER_PIN, LOW);
  
  Serial.begin(57600);
  printf_begin();
  printf("\n\rRF24 DHT22");

  StartRadio();
  radio.stopListening();
  radio.printDetails();                   // Dump the configuration of the rf unit for debugging
  radio.startListening();
   // disable calibration interval by setting modulus to
  sleep.setCalibrationInterval(1);
}



enum cycleMode {ModeInit, ModeListen, ModeReadSensor,ModeWriteData, ModeWait};


cycleMode cycle= ModeInit;


#define STRUCT_TYPE_GETDATA    0
#define STRUCT_TYPE_INIT_DATA  1
#define STRUCT_TYPE_DHT22_DATA 2
#define STRUCT_TYPE_DS18B20_DATA 3
#define STRUCT_TYPE_MAX6675_DATA 4

#define STATUS_DATA_VALID  1
#define STATUS_TIME_OUT    2

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
   unsigned char  Status;
   unsigned long  stampTime;
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
unsigned short nextTimeOnTimeOut = 60;
unsigned short waitTimeOnListen = 60;
unsigned long  startTimeOnListen;
unsigned char  gotTimeOut = 0;


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
  pinMode(DHT_PIN, OUTPUT);
  digitalWrite(DHT_PIN, HIGH);

  // Wait 2 sec
//  delay(2000);
  if(WDCalibrationCycle ==0)
    sleep.setCalibrationInterval(2);  // enable watch dog calibration one every 2 sleep
  WDCalibrationCycle++;
  WDCalibrationCycle %= WD_CALIB_COUNT;  //modulus 720 count (half day) on every minutes 
  sleep.pwrDownMode();
  sleep.sleepDelay(2000);


  // Now let's read the sensor twice
  // since the first one will be bad
  // ok let's check if we need to calibrate the watch dog timer
  DHT.read(DHT_PIN);
//  delay(1000);
   sleep.pwrDownMode();
   sleep.sleepDelay(1000);
   sleep.setCalibrationInterval(1);
   
  int rcode = DHT.read(DHT_PIN);
  
  // power off DHT22
  
  digitalWrite(DHT_POWER_PIN,LOW);
  pinMode(DHT_PIN, OUTPUT);
  digitalWrite(DHT_PIN, LOW);
  
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
     Txmdata.Status=0;
     Txmdata.temperature=0;
     Txmdata.humidity=0;
     Txmdata.voltageA2D=0;
     radio.writeAckPayload(1,&Txmdata,sizeof(TxmDHT22PacketStruct));
     cycle=ModeListen;
     startTimeOnListen = millis();
   }
   
  if(cycle==ModeListen)
   {
     // put arduino on sleep until we got 
     // an interrupt from RF24 trasnmitter
     //sleep.pwrDownMode();
     //sleep.sleepDelay(100);

     if(radio.available()) 
      {
       int rcv_size = radio.getDynamicPayloadSize();
       radio.read( &RcvData, rcv_size);

       if (RcvData.header != '*')
        {
          cycle = ModeWriteData;
          return;
        }
       Serial.print("Received after ");
       Serial.print((millis() - startTimeOnListen) / 1000.0);
       Serial.print(" sec\n");
       Serial.print("T:" );
       Serial.print(RcvData.currentTime);
       Serial.print(" Next reading in (1/10 sec): ");
       Serial.print(RcvData.nextTimeReading);
       Serial.print("\n");
       PrintHex(pt, rcv_size);
       Serial.print("\n");
       currentDelay = millis();
       #ifdef DISABLE_SLEEP
       delay(200);
       #endif
       waitTimeOnListen = 1;
       nextTimeOnTimeOut = RcvData.nextTimeOnTimeOut;
        if(RcvData.nextTimeReading > 30)
          { 
           sleepTime =  RcvData.nextTimeReading*100 - 3000UL;
           cycle = ModeWait;
          }
          else
           cycle = ModeReadSensor;
      }
     else
     {

      deltaTime = (millis() - startTimeOnListen) / 1000;
      // did we have time out
      if (deltaTime > waitTimeOnListen)
      {
        // ok we got time out
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
       sleep.pwrDownMode();
       sleep.sleepDelay(sleepTime);
       cycle = ModeReadSensor;
     }
   
   if(cycle== ModeReadSensor)
   {
     Txmdata.temperature = 32767;
     Txmdata.humidity=32767;
     Txmdata.Status  = gotTimeOut ? STATUS_TIME_OUT : 0 ;
     

     if(readSensor())
       {
         Txmdata.temperature = (short) floor(DHT.temperature * 10.0);
         Txmdata.humidity = (short) floor(DHT.humidity);
         Txmdata.Status |=  STATUS_DATA_VALID;
       }
       
      cycle = ModeWriteData; 
   }   
   
   if(cycle==ModeWriteData)
    {
     Txmdata.stampTime=RcvData.currentTime;
     Txmdata.header='*';
     Txmdata.structSize= sizeof(TxmDHT22PacketStruct);
     Txmdata.structType=STRUCT_TYPE_DHT22_DATA;
     Txmdata.txmUnitId = UNIT_ID;
     Txmdata.stampTime=RcvData.currentTime + (deltaTime / 1000);
     Txmdata.voltageA2D=readVcc();
      
     StartRadio(); 
     radio.writeAckPayload(1,&Txmdata,sizeof(TxmDHT22PacketStruct));
     startTimeOnListen = millis();
     cycle=ModeListen;
     gotTimeOut=0;     
    }
      
}
