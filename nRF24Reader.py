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
STRUCT_TYPE_MAX6675=4

class Logger(object):
  def __init__(self, filename="RFLog.txt"):
    self.terminal = sys.stdout
    self.log = open(filename, "a")
    self.LastPing = 0

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
    timeOut=False

class DS18B20Data:
    time = None
    temperature =0
    voltage=0
    valid=False
    timeOut=False

class MAX6675Data:
    time = None
    temperature =0
    voltage=0
    valid=False
    timeOut=False

class RF_Device:
  def __init__(self, deviceAddress, timeInterval=60):
    self.deviceAddress=deviceAddress
    self.timeInterval=timeInterval
    self.targetConnectionTime= time.time()
    self.nextConnectionTime= self.targetConnectionTime + self.timeInterval
    self.preConnectionDelay = 10.0
    self.postConnectionDelay = 5.0
    self.xdata = [0,0,0]
    self.rdata = ''
    self.timeOffsetAdjustment=0
    self.ReceivedTimeOut=False
    self.NoAdjustmentOnNext=True
    self.lastPing= 0
  def isTimeOut(self):
    #check if we need to test communication
    # are we near the target
    # is more than 1 second
    now = time.time();


    while now > (self.targetConnectionTime + self.timeInterval):
       self.targetConnectionTime += self.timeInterval
       self.nextConnectionTime = self.targetConnectionTime + self.timeInterval


    if (now  + self.preConnectionDelay) >= self.targetConnectionTime:
      if (now - self.postConnectionDelay) <= self.targetConnectionTime:
          return True
    if now >= self.nextConnectionTime :
          print("Sensor {} - {}  time out!".format(self.readSensorAddress(),time.ctime()))
          self.targetCOnnectionTime = self.nextConnectionTime
          self.nextConnectionTime += self.timeInterval
          return False
    if (now - self.lastPing) > 0.5:
          return True
    return False



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
        self.rdata = unpack('<sBBBBLHHH',''.join(map(chr,buffer)))
        dht22.valid = ((self.rdata[4] & 1) == 1)
        dht22.timeOut = ((self.rdata[4] & 2) ==2)
        dht22.time = self.rdata[5]
        dht22.voltage = self.rdata[6]/1000.0
        dht22.temperature = self.rdata[7]/10.0
        dht22.humidity = self.rdata[8]
        return dht22
     except:
        return None

  def unpackDS18B20Data(self , buffer):
     ds18b20 =  DS18B20Data

     if len(buffer) != 13:
         print('bad len')
         return None
     try:
        self.rdata = unpack('<sBBBBLHH',''.join(map(chr,buffer)))
        ds18b20.valid = ((self.rdata[4] & 1) == 1)
        ds18b20.timeOut = ((self.rdata[4] & 2) ==2)
        ds18b20.time = self.rdata[5]
        ds18b20.voltage = self.rdata[6]/1000.0
        ds18b20.temperature = self.rdata[7]/100.0
        return ds18b20
     except:
        print('error')
        return None

  def unpackMAX6675Data(self , buffer):
     max6675  =  MAX6675Data

     if len(buffer) != 13:
         return None
     try:
        self.rdata = unpack('<sBBBBLHH',''.join(map(chr,buffer)))
        max6675.valid = ((self.rdata[4] & 1) == 1)
        max6675.timeOut = ((self.rdata[4] & 2) ==2)
        max6675.time = self.rdata[5]
        max6675.voltage = self.rdata[6]/1000.0
        max6675.temperature = self.rdata[7]/4.0
        return max6675
     except:
        return None

  def getData(self):
    if( not self.isTimeOut()):
       return 

    self.lastPing = time.time();
    #buildpacket
    packet = '*'
    packet += chr(10)  #get packet size
    packet += chr(STRUCT_TYPE_GETDATA)
    packet += chr(0)   #0 mean master
    packet += pack('<L', numpy.uint32(time.time()))  #get current time
    #calculate next time sampling
    
    stepNextTime = self.nextConnectionTime - time.time() + self.timeOffsetAdjustment

    if stepNextTime < 5:
       stepNextTime+=60

    stepu16 = numpy.uint16((stepNextTime) * 10)
    
    packet += pack('<H', stepu16)  #get next time reading
    packet += pack('<h', numpy.uint16(60))    
 #    print("step {} unpack{}".format(stepu16,packet))        
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
           stampTime = time.time()
           timeOffset = stampTime - self.targetConnectionTime

           if in_buffer[2] == STRUCT_TYPE_INIT:
             #sensor is valid but just boot
             print("Sensor {} - {} - Just boot".format(self.readSensorAddress(),time.ctime()))
             validFlag=True
           if in_buffer[2] == STRUCT_TYPE_DHT22:
             probe = self.unpackDHT22Data(in_buffer)
             if probe != None:
               if probe.valid:
                 print("Sensor {} D:{:.1f} O:{:.1f}  - {} VCC:{}V - DHT22   T:{:.2f} C H:{} %".format(self.readSensorAddress(),timeOffset,self.timeOffsetAdjustment,time.ctime(),probe.voltage,probe.temperature,probe.humidity))
               else:
                 print("Sensor {} D:{:.1f} O:{:.1f} - {} VCC:{}V - DHT22   Unable to read".format(self.readSensorAddress(),timeOffset,self.timeOffsetAdjustment,time.ctime(),probe.voltage))
               validFlag=True

           if in_buffer[2] == STRUCT_TYPE_DS18B20:
             probe = self.unpackDS18B20Data(in_buffer)
             if probe != None:
               if probe.valid:
                 print("Sensor {} D:{:.1f} O:{:.1f} - {} VCC:{}V - DS18B20 T:{} C".format(self.readSensorAddress(),timeOffset,self.timeOffsetAdjustment,time.ctime(),probe.voltage,probe.temperature))
               else:
                 print("Sensor {} D:{:.1f} O:{:.1f} - {} VCC:{}V - DS18B20 Unable to read DS18B20 sensor".format(self.readSensorAddress(),timeOffset,self.timeOffsetAdjustment,time.ctime(),probe.voltage))
               validFlag=True

           if in_buffer[2] == STRUCT_TYPE_MAX6675:
             probe = self.unpackMAX6675Data(in_buffer)
             if probe != None:
               if probe.valid:
                 print("Sensor {} D:{:.1f} O:{:.1f} - {} VCC:{}V - MAX6675 T:{} C".format(self.readSensorAddress(),timeOffset,self.timeOffsetAdjustment,time.ctime(),probe.voltage,probe.temperature))
               else:
                 print("Sensor {} D:{:.1f} O:{:.1f} - {} VCC:{}V - MAX6675 Unable to read sensor".format(self.readSensorAddress(),timeOffset,self.timeOffsetAdjustment,time.ctime(),probe.voltage))
               validFlag=True
           
           if validFlag:
             self.targetConnectionTime = self.nextConnectionTime
             self.nextConnectionTime += self.timeInterval
             if probe.timeOut:
                self.NoAdjustmentOnNext= True
#             if not self.NoAdjustmentOnNext:
             if abs(timeOffset)<5:
                 self.timeOffsetAdjustment -= timeOffset / 3
             self.NoAdjustementOnNext=False
           

#           print("Next time {} in sec{}".format(time.ctime(self.nextConnectionTime),stepNextTime))
           if not validFlag:
            print("Sensor {} - {}  Invalid packet!".format(self.readSensorAddress(),time.ctime()))



masterAddress = [0xe7, 0xe7, 0xe7, 0xe7, 0xe7]


device = [
	  RF_Device([0xc2,0xc2,0xc2,0xc2,0xc3],60),
	  RF_Device([0xc2,0xc2,0xc2,0xc2,0xc4],60),
	  RF_Device([0xc2,0xc2,0xc2,0xc2,0xc5],60),
	  RF_Device([0xc2,0xc2,0xc2,0xc2,0xc6],60)]

# space each device in time 
delay=0
for i in device:
  i.targetConnectionTime += delay * i.timeInterval/len(device)
  i.nextConnectionTime = i.targetConnectionTime + i.timeInterval
  print("Device {} set to {}".format(i.readSensorAddress(),time.ctime(i.targetConnectionTime)))
  delay+=1


radio = NRF24(GPIO, spidev.SpiDev())
radio.begin(0, 17)
time.sleep(1)
radio.setRetries(7,3)
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
      i.getData();
   time.sleep(0.005)

except KeyboardInterrupt:
    radio.stopListening();
    radio.powerDown();
    raise
