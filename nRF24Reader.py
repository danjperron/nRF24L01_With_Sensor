#!/usr/bin/python
# -*- coding: utf-8 -*-
#
# Example program to send packets to the radio link
#


import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
from lib_nrf24 import NRF24
import time
import spidev
import binascii
import numpy
import sys
from struct import *


STRUCT_TYPE_GETDATA=0
STRUCT_TYPE_INIT=1
STRUCT_TYPE_DHT22=2
STRUCT_TYPE_DS18B20=3

class Logger(object):
  def __init__(self, filename="RFLog.txt"):
    self.terminal = sys.stdout
    self.log = open(filename, "a")

  def write(self, message):
    self.terminal.write(message)
    self.log.write(message)
    self.log.flush()

sys.stdout = Logger()



class DHT22Data:
    time = None
    humidity = 0
    temperature =0
    voltage=0
    valid=False

class DS18B20Data:
    time = None
    temperature =0
    voltage=0
    valid=False

class RF_Device:
  def __init__(self, deviceAddress, nextTime=60):
    self.deviceAddress=deviceAddress
    self.nextTime=nextTime;
    self.lastConnectionTime= time.time()
    self.nextConnectionTime= self.lastConnectionTime
    self.xdata = [0,0,0]
    self.rdata = ''

  def isTimeOut(self):
    return (time.time() > self.nextConnectionTime)


  def readSensorAddress(self):
     buffer = '{:02X}'.format(self.deviceAddress[0])
     for i in range(1,5,1):
       buffer += ':{:02X}'.format(self.deviceAddress[i])   
     return buffer



  def unpackDHT22Data(self , buffer):
     dht22 =  DHT22Data

     if len(buffer) != 15:
         return None
     try:
        self.rdata = unpack('<sBBBLBHHH',''.join(map(chr,buffer)))
        dht22.time = self.rdata[4]
        dht22.valid = self.rdata[5]!=0
        dht22.voltage = self.rdata[6]/1000.0
        dht22.temperature = self.rdata[7]/10.0
        dht22.humidity = self.rdata[8]
        return dht22
     except:
        return None

  def unpackDS18B20Data(self , buffer):
     ds18b20 =  DS18B20Data

     if len(buffer) != 13:
         return None
     try:
        self.rdata = unpack('<sBBBLBHH',''.join(map(chr,buffer)))
        ds18b20.time = self.rdata[4]
        ds18b20.valid = self.rdata[5]!=0
        ds18b20.voltage = self.rdata[6]/1000.0
        ds18b20.temperature = self.rdata[7]/100.0
        return ds18b20
     except:
        return None

  def getData(self):
    if( not self.isTimeOut()):
       return None
    
#    print("timeout={}".format(self.nextTime))
    self.nextConnectionTime += self.nextTime
    #buildpacket
    packet = '*'
    packet += chr(10)  #get packet size
    packet += chr(STRUCT_TYPE_GETDATA)
    packet += chr(0)   #0 mean master
    packet += pack('<L', numpy.uint32(time.time()))  #get current time
    packet += pack('<H', numpy.uint16(self.nextTime *10))  #get next time reading
    
        
 #   print("send : {}".format(list(packet)))

    radio.openWritingPipe(self.deviceAddress)

    radio.write(packet)
    if True:
     if radio.isAckPayloadAvailable():
      in_buffer=[]
      radio.read(in_buffer,radio.getDynamicPayloadSize())
      validFlag= False
      if len(in_buffer)>4:
         # check first four bytes
         if in_buffer[0] == ord('*'):
           if in_buffer[2] == STRUCT_TYPE_INIT:
             #sensor is valid but just boot
             print("Sensor {} - {} - Just boot".format(self.readSensorAddress(),time.ctime()))
             validFlag=True
           if in_buffer[2] == STRUCT_TYPE_DHT22:
             probe = self.unpackDHT22Data(in_buffer)
             if probe != None:
               if probe.valid:
                 print("Sensor {} - {} VCC:{}V - DHT22   T:{:.2f} C H:{} %".format(self.readSensorAddress(),time.ctime(probe.time),probe.voltage,probe.temperature,probe.humidity))
               else:
                 print("Sensor {} - {} VCC:{}V - DHT22   Unable to read".format(self.readSensorAddress(),time.ctime(probe.time),probe.voltage))
               validFlag=True

           if in_buffer[2] == STRUCT_TYPE_DS18B20:
             probe = self.unpackDS18B20Data(in_buffer)
             if probe != None:
               if probe.valid:
                 print("Sensor {} - {} VCC:{}V - DS18B20 T:{} C".format(self.readSensorAddress(),time.ctime(probe.time),probe.voltage,probe.temperature))
               else:
                 print("Sensor {} - {} VCC:{}V - DS18B20 Unable to read DS18B20 sensor".format(self.readSensorAddress(),time.ctime(probe.time),probe.voltage))
               validFlag=True

         
#       except:
#         print("Unable to unpack!Bad packet")

      if not validFlag:
            print("Sensor {} - {}  Invalid packet!".format(self.readSensorAddress(),time.ctime()))
     else:
            print("Sensor {} - {}  time out!".format(self.readSensorAddress(),time.ctime()))



masterAddress = [0xe7, 0xe7, 0xe7, 0xe7, 0xe7]

device = [RF_Device([0xc2,0xc2,0xc2,0xc2,0xc3],10),
	  RF_Device([0xc2,0xc2,0xc2,0xc2,0xc4],10)]



radio = NRF24(GPIO, spidev.SpiDev())
radio.begin(0, 17)
time.sleep(1)
radio.setRetries(15,15)
radio.setPayloadSize(32)
radio.setChannel(78)

radio.setDataRate(NRF24.BR_1MBPS)
radio.setPALevel(NRF24.PA_MAX)
radio.setAutoAck(True)
radio.enableDynamicPayloads()
radio.enableAckPayload()


radio.openWritingPipe(device[0].deviceAddress)
radio.openReadingPipe(1, masterAddress)
radio.printDetails()

time.sleep(1)

try:

 while True:
   for i in  device:
     if(i.isTimeOut()):
      i.getData();
 
   time.sleep(0.01)

except KeyboardInterrupt:
    radio.stopListening();
    radio.powerDown();
    raise
