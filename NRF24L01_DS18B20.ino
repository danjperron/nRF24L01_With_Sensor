
/*
 Copyright (C) 2015 Daniel Perron
 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.

 Read DS18B20 sensor using a nRF24L01 sensor base on J.Coliz code

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

#include <OneWire.h>

// OneWire DS18S20, DS18B20, DS1822 Temperature Example
//
// http://www.pjrc.com/teensy/td_libs_OneWire.html
//
// The DallasTemperature library can do all this work for you!
// http://milesburton.com/Dallas_Temperature_Control_Library


// assume DS18B20

#define DS18B20_PIN 2
#define DS18B20_POWER_PIN 4

#define RF24_POWER_PIN 5



//assuming one sensor per device
// we will use the SKIP command


OneWire  ds(DS18B20_PIN);  // on pin 10 (a 4.7K resistor is necessary)

float celsius;

Sleep sleep;

unsigned long sleepTime = 300000;

short  temperature = 32767;
unsigned short  humidity  = 32767;


// energy mode

//#define DISABLE_SLEEP

///////////////   radio /////////////////////
// Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 9 & 10
RF24 radio(8, 7);

#define UNIT_ID 0xc6

const uint8_t MasterPipe[6] = {0xe7, 0xe7, 0xe7, 0xe7, 0xe7, 0};

const uint8_t SensorPipe[6]  = { UNIT_ID, 0xc2, 0xc2, 0xc2, 0xc2, 0};


// Radio pipe addresses for the 2 nodes to communicate.


// Set up roles to simplify testing
//boolean role;                                    // The main role variable, holds the current role identifier
//boolean role_ping_out = 1, role_pong_back = 0;   // The two different roles.
unsigned long Count = 0;


// because of sleep mode calibration problem
// and I don't want to change the library
// we will mimic the calibration
// by disable it and enable only when we reach 0 at modulus 720
// we will calibrate the watch dog  when we wait  for the DS18B20 conversion

#define        WD_CALIB_COUNT   720
unsigned short WDCalibrationCycle=0;



void StopRadio()
{
  Serial.print("*** STOP RADIO ***\n");
  delay(100);
  pinMode(RF24_POWER_PIN, OUTPUT);
  digitalWrite(RF24_POWER_PIN, LOW);
}


void StartRadio()
{
  Serial.print("*** START RADIO ***\n");

  pinMode(RF24_POWER_PIN, OUTPUT);
  digitalWrite(RF24_POWER_PIN, HIGH);
  delay(50);
  radio.begin();                          // Start up the radio
  radio.setPayloadSize(32);
  radio.setChannel(0x4e);
  radio.setDataRate(RF24_1MBPS);
  radio.setAutoAck(1);                    // Ensure autoACK is enabled
  radio.setRetries(7, 3);  // Max delay between retries & number of retries
  radio.enableDynamicPayloads();
  radio.enableAckPayload();
  radio.maskIRQ(true, true, false);
  //role = role_ping_out;                  // Become the primary transmitter (ping out)
  radio.openWritingPipe(MasterPipe);
  radio.openReadingPipe(1, SensorPipe);
  radio.startListening();                 // Start listening
}



void setup() {
  // Set pin for DHT22 power
  pinMode(DS18B20_POWER_PIN, OUTPUT);
  digitalWrite(DS18B20_POWER_PIN, LOW);

  Serial.begin(57600);
  printf_begin();
  printf("\n\rRF24  DS18B20\n\r");
  StartRadio();
  radio.stopListening();
  radio.printDetails();                   // Dump the configuration of the rf unit for debugging
  radio.startListening();

  // disable calibration interval by setting modulus to
  sleep.setCalibrationInterval(1);

}





enum cycleMode {ModeInit, ModeListen, ModeWriteData, ModeWait};

cycleMode cycle = ModeInit;


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
} RcvPacketStruct;


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
} TxmDS18B20PacketStruct;


unsigned long currentDelay;
unsigned long targetDelay;

RcvPacketStruct RcvData;
TxmDS18B20PacketStruct Txmdata;

unsigned char * pt = (unsigned char *) &RcvData;


unsigned char rcvBuffer[32];
unsigned short nextTimeOnTimeOut = 60;
unsigned short waitTimeOnListen = 60;
unsigned long  startTimeOnListen;
unsigned char  gotTimeOut = 0;

void PrintHex(uint8_t *data, uint8_t length) // prints 16-bit data in hex with leading zeroes
{
  char tmp[32];
  for (int i = 0; i < length; i++)
  {
    sprintf(tmp, "0x%.2X", data[i]);
    Serial.print(tmp); Serial.print(" ");
  }
}





bool readSensor(void)
{
  byte data[12];
  byte present = 0;
  byte type_s;
  byte CRC;
  byte i;

  // power Sensor UP
  digitalWrite(DS18B20_POWER_PIN, HIGH);
  pinMode(DS18B20_PIN, INPUT);

#ifdef DISABLE_SLEEP
  // Wait 200 ms
  delay(200);
#else
// ok let's check if we need to calibrate the watch dog timer
  if(WDCalibrationCycle ==0)
    sleep.setCalibrationInterval(2);  // enable watch dog calibration one every 2 sleep
  
  WDCalibrationCycle++;
  WDCalibrationCycle %= WD_CALIB_COUNT;  //modulus 720 count (half day) on every minutes

  sleep.pwrDownMode();
  sleep.sleepDelay(200);
#endif
  // start conversion
  Serial.println("CV");
  ds.reset();
  ds.write(0xcc); // skip rom command
  ds.write(0x44, 1); // start conversion
#ifdef DISABLE_SLEEP
  // Wait 200 ms
  delay(800);
#else

// ok check to calibrate sleep mode


  sleep.pwrDownMode();
  sleep.sleepDelay(800);
  sleep.setCalibrationInterval(1);  //disable internal calibration on next sleep
#endif

  Serial.println("read");
  present = ds.reset();
  ds.write(0xcc);
  ds.write(0XBE); // read Scratchpad


  Serial.println("Get data");
  for (i = 0 ; i < 9 ; i++)
    data[i] = ds.read();

  Serial.print("Check CRC ");
  CRC = OneWire::crc8(data, 8);

  Serial.print(CRC);
  Serial.print(" == ");
  Serial.print(data[8]);
  Serial.println("?");

  if (CRC != data[8])
    return 0;

  int16_t raw = (data[1] << 8) | data[0];

  byte cfg = (data[4] & 0x60);
  if (cfg == 0x00) raw = raw & ~7;
  else if ( cfg == 0x20) raw = raw & ~3;
  else if ( cfg == 0x40) raw = raw & ~1;

  celsius = (float) raw / 16.0;


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

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA, ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both

  long result = (high << 8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return (unsigned short) result; // Vcc in millivolts
}


void loop(void) {
  int loop;
  unsigned long deltaTime;
  /****************** Ping Out Role ***************************/

  if (cycle == ModeInit)
  {
    Count++;
    Txmdata.header = '*';
    Txmdata.structSize = sizeof(TxmDS18B20PacketStruct);
    Txmdata.structType = STRUCT_TYPE_INIT_DATA;
    Txmdata.txmUnitId = UNIT_ID;
    Txmdata.stampTime = 0;
    Txmdata.Status = 0;
    Txmdata.temperature = 0;
    Txmdata.voltageA2D = 0;
    radio.writeAckPayload(1, &Txmdata, sizeof(TxmDS18B20PacketStruct));
    cycle = ModeListen;
    startTimeOnListen = millis();
  }

  if (cycle == ModeListen)
  {
    if (radio.available())
    {
      int rcv_size = radio.getDynamicPayloadSize();
      radio.read( &RcvData, rcv_size);

      if (RcvData.header != '*')
        return;
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

      if (RcvData.nextTimeReading > 50)
      {
        sleepTime =  RcvData.nextTimeReading * 100 - 2000;
        cycle = ModeWait;
      }
      else
        cycle = ModeWriteData;
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


  if (cycle == ModeWait)
  {
    delay(10);
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


  if (cycle == ModeWriteData)
  {
    Txmdata.stampTime = RcvData.currentTime;
    Txmdata.header = '*';
    Txmdata.structSize = sizeof(TxmDS18B20PacketStruct);
    Txmdata.structType = STRUCT_TYPE_DS18B20_DATA;
    Txmdata.txmUnitId = UNIT_ID;
    Txmdata.stampTime = RcvData.currentTime + (deltaTime / 1000);
    Txmdata.Status = gotTimeOut ? STATUS_TIME_OUT : 0;
    Txmdata.temperature = 32767;
    Txmdata.voltageA2D = readVcc();


    if (readSensor())
    {
      Txmdata.temperature = (short) floor(celsius * 100);
      Txmdata.Status |= STATUS_DATA_VALID;
    }

    // power down sensor
    digitalWrite(DS18B20_POWER_PIN, LOW);

    StartRadio();
    radio.writeAckPayload(1, &Txmdata, sizeof(TxmDS18B20PacketStruct));
    startTimeOnListen = millis();
    cycle = ModeListen;
    gotTimeOut = 0;
  }

}
