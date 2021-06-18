#include <nRF24L01.h>
#include <RF24_config.h>
#include <RF24.h>
#include <SPI.h>
#include <Sleep_n0m1.h>
#include "printf.h"


/*
  Copyright (C) 2020 Daniel Perron
  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  version 2 as published by the Free Software Foundation.

  Read Moisture sensor  via nRF24L01 transmitter.

  Version 4.0 June 2021 Daniel Perron

  This is a simplified version using only 3V.
 
  The capacitive sensor has the 555 timer removed and D9 clock the sensor
  
*/



/**** cycle loop behavior *****/

// NO_WAIT    the unit keep looping without doing sleep
//#define NO_WAIT 

// DISABLE_SLEEP    disable deep sleep  use delay instead
//#define DISABLE_SLEEP

#define   SERIAL_DEBUG


/***** capacitive sensor clock definition ****/

#define CLOCK_PIN  9
#define CLOCK_FREQUENCY 800000



/*******  RF24 definition                *****/

// UNIT_ID     ID number of the device for nRF24L01

#define RF24_CE_PIN  8
#define RF24_CSN_PIN 7

//#define RF24_DATA_RATE RF24_1MBPS
//#define RF24_DATA_RATE RF24_2MBPS
#define RF24_DATA_RATE RF24_250KBPS

//#define RADIO_CHANNEL 72
#define RADIO_CHANNEL 1


#define RF24_PA_LEVEL RF24_PA_MIN
//#define RF24_PA_LEVEL RF24_PA_LOW
//#define RF24_PA_LEVEL RF24_PA_HIGH


#define UNIT_ID 0x03

// Radio pipe addresses for the 2 nodes to communicate.
const uint64_t serverPipe = 0xE0E0E0E001LL;
const uint64_t sensorPipe = 0xA5A5A5A500LL | UNIT_ID;


/********   Variables   ******/
short analog0;  // analog capacitive value
short analog1;  // spare analog input
short vcc;      // value of vcc
unsigned long clockFrequency= CLOCK_FREQUENCY;

/*******  sleep mode class ******/
Sleep sleep;
unsigned long sleepTime = 300000;

/******* radio class ******/
RF24 radio(RF24_CE_PIN,RF24_CSN_PIN);

unsigned long Count = 0;

// because of sleep mode calibration problem
// and I don't want to change the library
// we will mimic the calibration
// by disable it and enable only when we reach 0 at modulus 720
// we will calibrate the watch dog  when we wait  for the DS18B20 conversion

#define        WD_CALIB_COUNT   720
unsigned short WDCalibrationCycle = 0;


void StartRadio()
{
#ifdef SERILAL_DEBUG
  Serial.print("*** START RADIO ***\n");
#endif

  radio.begin();                          // Start up the radio
  radio.setChannel(RADIO_CHANNEL);
  radio.setDataRate(RF24_DATA_RATE);
  radio.setPALevel(RF24_PA_LEVEL);
  radio.setPayloadSize(32);
  radio.setAutoAck(1);                    // Ensure autoACK is enabled
  radio.setRetries(7, 4);  // Max delay between retries & number of retries
  radio.enableDynamicPayloads();
  radio.enableAckPayload();
  radio.maskIRQ(true, true, false);
  //role = role_ping_out;                  // Become the primary transmitter (ping out)
  radio.openWritingPipe(serverPipe);
  radio.openReadingPipe(1,sensorPipe);
  radio.startListening();                 // Start listening
}


//***************  TIMER1  CLOCK OSCILLATOR ON OC1A


// clock is on D9
// this only works for frequency CPU_CLOCK/2 to CPU_CLOCK/65536
// adapted to take care of compile cpu clock (F_CPU)
void setClock(unsigned long tempfreq){
  unsigned long SquareWaveClock = (unsigned long) F_CPU /2L;
  pinMode(CLOCK_PIN, OUTPUT);
  cli();//disable interupts
  TCCR1A = 0;//registers for timer 1
  TCCR1B = 0;
  TCNT1=0;
  TCCR1A |= _BV(COM1A0);
  TCCR1B |=_BV(WGM12);
  TCCR1C = _BV(FOC1A);
  OCR1A = (SquareWaveClock/tempfreq)-1;//#TIMER COUNTS
  TCCR1B |= _BV(CS10);
  sei();//enable interupts
}



void killClock(void)
{
  cli();//disable interupts
  TCCR1A = 0;//registers for timer 1
  TCCR1B = 0;
  TCNT1=0;
  sei();
  digitalWrite(CLOCK_PIN,LOW);
}


//****************  800KHZ CLock END



void setup() {
#ifdef SERIAL_DEBUG
  Serial.begin(57600);
  printf_begin();
  printf("\n\rRF24 ANALOG\n\r");
#endif
  StartRadio();
#ifdef SERIAL_DEBUG
  radio.stopListening();
  radio.printDetails();                   // Dump the configuration of the rf unit for debugging
  radio.startListening();
#endif
  // disable calibration interval by setting modulus to
  sleep.setCalibrationInterval(1);

}


/********  cycle state definition *****/

enum cycleMode {ModeInit, ModeListen, ModeReadSensor, ModeWriteData, ModeWait};

cycleMode cycle = ModeInit;



/********  type of sensor definition *****/

#define STRUCT_TYPE_GETDATA    0
#define STRUCT_TYPE_INIT_DATA  1
#define STRUCT_TYPE_DHT22_DATA 2
#define STRUCT_TYPE_DS18B20_DATA 3
#define STRUCT_TYPE_MAX6675_DATA 4
#define STRUCT_TYPE_DIGITAL_OUTPUT 5
#define STRUCT_TYPE_ANALOG_DATA  6
#define STRUCT_TYPE_MOISTURE_DATA 7

#define STATUS_DATA_VALID  1
#define STATUS_TIME_OUT    2


/******* data structure for STRUCT_TYPE_MOISTURE_DATA ******/


typedef struct
{
  char header;
  unsigned char structSize;
  unsigned char structType;
  unsigned char txmUnitId;
  unsigned long currentTime;
  unsigned short nextTimeReading;
  unsigned short nextTimeOnTimeOut;
  unsigned long  clockFrequency;
  unsigned char  clockFrequencyValid;
  char Spare[15];
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
  unsigned short analog0;
  unsigned short analog1;
  unsigned long  frequency;
} TxmMoisturePacketStruct;


unsigned long currentDelay;
unsigned long targetDelay;

RcvPacketStruct RcvData;
TxmMoisturePacketStruct Txmdata;

unsigned char * pt = (unsigned char *) &RcvData;


unsigned char rcvBuffer[32];
unsigned short nextTimeOnTimeOut = 300;
const unsigned short waitTimeOnListen = 5;
unsigned long  startTimeOnListen;

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
  // enable clock output
  if(clockFrequency < 10000)
    clockFrequency=CLOCK_FREQUENCY;
   
  setClock(clockFrequency);

  // Wait 300 ms
  delay(300);

  //ok read sensor and nRF24L01 batteries
  analog0 = analogRead(A0);
  analog1 = analogRead(A1);
  vcc     = readVcc();


#ifdef SERIAL_DEBUG
  Serial.println("Get data");
  Serial.print("Moisture:");
  Serial.println(analog0);
  Serial.print("Two batteries:");
  Serial.println(analog1);
  Serial.print("Frequency:");
  Serial.println(clockFrequency);
  Serial.print("VCC:");
  Serial.println(vcc);
#endif

  // kill clock
  killClock();
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
  int loop;
  unsigned long deltaTime;
  /****************** Ping Out Role ***************************/

  if (cycle == ModeInit)
  {
    Count++;
    Txmdata.header = '*';
    Txmdata.structSize = sizeof(TxmMoisturePacketStruct);
    Txmdata.structType = STRUCT_TYPE_INIT_DATA;
    Txmdata.txmUnitId = UNIT_ID;
    Txmdata.stampTime = 0;
    Txmdata.Status = 0;
    Txmdata.analog0 = 0;
    Txmdata.analog1 = 0;
    Txmdata.frequency = 0;
    Txmdata.voltageA2D = readVcc();
    radio.writeAckPayload(1, &Txmdata, sizeof(TxmMoisturePacketStruct));
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
      {
        cycle = ModeWriteData;
        return;
      }

      if(RcvData.clockFrequencyValid)
         clockFrequency=RcvData.clockFrequency;

#ifdef SERIAL_DEBUG
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
#endif
      currentDelay = millis();
#ifdef DISABLE_SLEEP
      delay(200);
#endif


      nextTimeOnTimeOut = RcvData.nextTimeOnTimeOut;

      if (RcvData.nextTimeReading > 20)
      {
        sleepTime =  (unsigned long) RcvData.nextTimeReading * 100 ;
#ifdef SERIAL_DEBUG
        Serial.print("New sleeptime (ms) =");
        Serial.println(sleepTime);
#endif        
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
#ifdef SERIAL_DEBUG
        Serial.print("got time out!");
        Serial.print(deltaTime);
        Serial.print("sec.  Sleep for ");
        Serial.print(nextTimeOnTimeOut);
        Serial.print("sec.\n");
#endif
       
        sleepTime = (unsigned long) nextTimeOnTimeOut * 1000;
        cycle = ModeWait;

      }
    }

  }


  if (cycle == ModeWait)
  {
    radio.powerDown();
    delay(5);
    SPI.end();
#ifndef NO_WAIT    
#ifdef DISABLE_SLEEP
    delay(sleepTime);
#else
#ifdef SERIAL_DEBUG
    Serial.print("Go to Sleep ");
    Serial.print(sleepTime);
    Serial.println("ms");
    delay(100);
#endif
    sleep.pwrDownMode();
    sleep.sleepDelay(sleepTime);
#endif

#ifdef SERIAL_DEBUG
    Serial.println("Arduino WakeUP");
#endif
    
#endif
    radio.powerUp();
    cycle = ModeReadSensor;
  }

  if (cycle == ModeReadSensor)
  {
    if (readSensor())
    {
      Txmdata.analog0 = analog0;
      Txmdata.analog1 = analog1;
      Txmdata.voltageA2D = vcc;
      Txmdata.frequency = clockFrequency;
      Txmdata.Status = STATUS_DATA_VALID;
    }
    cycle = ModeWriteData;
  }

  if (cycle == ModeWriteData)
  {
    Txmdata.stampTime = RcvData.currentTime;
    Txmdata.header = '*';
    Txmdata.structSize = sizeof(TxmMoisturePacketStruct);
    Txmdata.structType = STRUCT_TYPE_MOISTURE_DATA;
    Txmdata.txmUnitId = UNIT_ID;
    Txmdata.stampTime = RcvData.currentTime + (deltaTime / 1000);
    StartRadio();
    radio.writeAckPayload(1, &Txmdata, sizeof(TxmMoisturePacketStruct));
    startTimeOnListen = millis();
    cycle = ModeListen;
  }

}
