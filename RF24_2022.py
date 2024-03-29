
#!/usr/bin/python3
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
import paho.mqtt.client as paho
from threading import Lock


LedR = 22
LedG = 23
LedB = 24

pins = [ LedR, LedG, LedB]

GPIO.setmode(GPIO.BCM)
for i in pins:
    GPIO.setup(i,GPIO.OUT)

def setLedRed(value):
    GPIO.output(LedR,value)

def setLedGreen(value):
    GPIO.output(LedG,value)

def setLedBlue(value):
    GPIO.output(LedB,value)



setLedBlue(True)

client = paho.Client('RFUnit')
client.connect('127.0.0.1')

# Possible Frequency from Arduino Timer
MoistFrequency=[10000, 364000, 400000, 445000, 500000, 572000, 667000, 800000, 1000000, 1334000]




Debug=False


STRUCT_TYPE_GETDATA=0
STRUCT_TYPE_INIT=1
STRUCT_TYPE_DHT22=2
STRUCT_TYPE_DS18B20=3
STRUCT_TYPE_MAX6675=4
STRUCT_TYPE_DIGITAL_OUTPUT=5
STRUCT_TYPE_ANALOG=6
STRUCT_TYPE_MOISTURE=7


lock = Lock()  # MQTT could send something , we need to sync.
radio = None


## fix python3 printDetails() bug  in  lib_nrf24.py library
## remove extra new line  when is not needed
class bypass_stdout(object):
  def __init__(self,V):
    sysout = sys.stdout
    self.line = V

  def write(self, message):
    self.line+= message

  def flush(self):
    pass


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

class AnalogData:
    time = None
    Analog0 = 0
    Analog1 = 0
    Analog2 = 0
    Analog3 = 0
    voltage=0
    valid=False
    timeOut=False

class MoistureData:
    time = None
    Analog0 = 0
    Analog1 = 0
    Frequency = 0
    voltage=0
    valid=False
    timeOut=False


class RF_Device:
  def __init__(self, deviceAddress,timeInterval=60,mqtt=None,topic="RF_DATA",clockFrequency=None):
    self.mqtt=mqtt
    self.mqttTopic=topic
    self.deviceAddress=deviceAddress
    self.timeInterval=timeInterval
    self.targetConnectionTime= time.time()
    self.preConnectionDelay = 12.0
    self.postConnectionDelay = 10.0
    self.xdata = [0,0,0]
    self.rdata = ''
    self.timeOffsetAdjustment=1.8
    self.timeOutFlag=True
    self.NoAdjustmentOnNext=True
    self.lastPingData= 0
    self.lastPing = 0
    self.gotDataFlag= False
    self.clockFrequency=clockFrequency

  def publish(self,packet):
    if self.mqtt != None:
      self.mqtt.publish(self.mqttTopic,packet)
    print("publish "+ self.mqttTopic + "  msg:" + packet)

  def publishValue(self,id,packet):
    print("publishValue",id,packet)
    buffer = '{:02X}'.format(self.deviceAddress[4])
    if self.mqtt !=None:
       self.mqtt.publish(self.mqttTopic+"/"+buffer+"/"+id,str(packet),retain=True)
    print("publish "+ self.mqttTopic +"/"+buffer+"/"+id+"  msg:" + str(packet))


  def isScheduleUp(self):
   # check if we need to request info

   now = time.time()

   if (now - self.lastPing)>2.0:
     return True

#  Is the sensor on Timeout
   if self.timeOutFlag :
      return False

   # Are we after the target time
   if now > (self.targetConnectionTime + ( self.postConnectionDelay * self.timeInterval / 60.0)):
     #did we received the data
     if self.gotDataFlag:
       self.gotDataFlag=False
     else:
       #set time out and return false
       self.timeOutFlag = True
       setLedRed(True)
       self.publish("Sensor {} - {} - time out".format(self.readSensorAddress(),time.ctime()))
       setLedRed(False)
     #update NextConnectionTime()
     while (self.targetConnectionTime - self.preConnectionDelay)   <= now:
        self.targetConnectionTime+= self.timeInterval


     return False

   #did we got the data already
   if self.gotDataFlag:
      return False

   # are we near the target time
   return (now >= (self.targetConnectionTime - self.preConnectionDelay))





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
        self.rdata = unpack('<sBBBBLHHH',buffer)
        dht22.valid = ((self.rdata[4] & 1) == 1)
        dht22.timeOut = ((self.rdata[4] & 2) ==2)
        dht22.time = self.rdata[5]
        dht22.voltage = self.rdata[6]/1000.0
        if self.rdata[7] >= 32768:
           dht22.temperature = (self.rdata[7] - 65536)/10.0
        else:
           dht22.temperature = self.rdata[7]/10.0
        dht22.humidity = self.rdata[8]
        return dht22
     except:
        return None

  def unpackAnalogData(self , buffer):
     Analog = AnalogData
     if len(buffer) != 19:
         return None
     try:
        self.rdata = unpack('<sBBBBLHHHHH',buffer)
        Analog.valid = ((self.rdata[4] & 1) == 1)
        Analog.timeOut = ((self.rdata[4] & 2) ==2)
        Analog.time = self.rdata[5]
        Analog.voltage = self.rdata[6]/1000.0
        Analog.Analog0 = Analog.voltage * self.rdata[7] / 1023.0
        Analog.Analog1 = Analog.voltage * self.rdata[8] / 1023.0
        Analog.Analog2 = Analog.voltage * self.rdata[9] / 1023.0
        Analog.Analog3 = Analog.voltage * self.rdata[10] / 1023.0
        return Analog
     except:
        return None

  def unpackMoistureData(self , buffer):
     Moisture = MoistureData
     if len(buffer) != 19:
         return None
     try:
        self.rdata = unpack('<sBBBBLHHHL',buffer)
        Moisture.valid = ((self.rdata[4] & 1) == 1)
        Moisture.timeOut = ((self.rdata[4] & 2) ==2)
        Moisture.time = self.rdata[5]
        Moisture.voltage = self.rdata[6]/1000.0
        # on version 3.0 Regulator on moisture is removed
        # we need to recalibrate moisture with Vref at 3.3V
        # Then Moisture base on 3.3V
        # let assume that Voltage of diode is 0.4V
        #  moist V =   (3.3V / CPU voltage  * (analog0 Voltage + Vdiode)) - Vdiode
        # inifity capacitor and V = (3.3V / 2) - Vdiode

        analog0V = Moisture.voltage * self.rdata[7]  / 1023.0
        if  self.rdata[6] == 0:
           return None
        Vdiode = 0
        moistV =  (3.3  / Moisture.voltage) * (analog0V + Vdiode) - Vdiode
        Moisture.Analog0 = self.rdata[7]
        Moisture.Analog1 = Moisture.voltage * self.rdata[8] / 1023.0
        Moisture.Frequency = self.rdata[9]
        return Moisture



     except:
        return None


  def unpackDS18B20Data(self , buffer):
     ds18b20 =  DS18B20Data

     if len(buffer) != 13:
         print('bad len')
         return None
     try:
        self.rdata = unpack('<sBBBBLHH',buffer)
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
        self.rdata = unpack('<sBBBBLHH',buffer)
        max6675.valid = ((self.rdata[4] & 1) == 1)
        max6675.timeOut = ((self.rdata[4] & 2) ==2)
        max6675.time = self.rdata[5]
        max6675.voltage = self.rdata[6]/1000.0
        max6675.temperature = self.rdata[7]/4.0
        return max6675
     except:
        return None



  def getData(self):
    global MoistFreq
    global MoistLen
    global MoistIndex
    global MoistCount
    global MoistMaxCount
    global MoistSum
    global MoistSumCount

    if Debug:
       print("Get data {} {}".format(time.ctime(),self.deviceAddress))
    self.lastPingData = time.time();
    #buildpacket
    packet = b'*'
    if self.clockFrequency is None:
       packet += b'\x0a'  # set packet size
    else:
       packet += b'\x0f'

    packet += pack('<BBL',STRUCT_TYPE_GETDATA,0,numpy.uint32(time.time()))
    stepNextTime = self.targetConnectionTime - time.time() + self.timeOffsetAdjustment + self.timeInterval

    if stepNextTime > (2 * self.timeInterval):
       #something wrong here just timeInterval
       stepNextTime = self.timeInterval - 2

    while stepNextTime < 5:
       #ok too soon update it
       stepNextTime+=self.timeInterval + 2


    stepu16 = numpy.uint16((stepNextTime) * 10)
    if Debug:
       print("time offset : {}   time interval: {} nextTime: {} stepU16: {}".format(self.timeOffsetAdjustment, self.timeInterval,stepNextTime,stepu16))


    packet += pack('<H', stepu16)  #get next time reading
    packet += pack('<h', numpy.uint16(60))


    if self.clockFrequency is not None:
       packet+= pack('<LB', self.clockFrequency,1)

    radio.openWritingPipe(self.deviceAddress)
    probe = None
    lock.acquire()    #prevent On message to disturb
    radio.write(packet)
    if radio.isAckPayloadAvailable():
      in_buffer=[]
      radio.read(in_buffer,radio.getDynamicPayloadSize())
#      if Debug:
      if True:
        print("got something -> ")
        print(in_buffer)
      validFlag= False
      if len(in_buffer)>4:
         # check first four bytes
         if in_buffer[0] == ord('*'):
           if sys.version_info[0] > 2:
               join_buffer = bytes(in_buffer)
           else:
               join_buffer = ''.join(map(chr,in_buffer))

           stampTime = time.time()
           timeOffset = stampTime - self.targetConnectionTime
           if in_buffer[2] == STRUCT_TYPE_INIT:
             #sensor is valid but just boot
             setLedGreen(True)
             self.publish("Sensor {} - {} - Just boot".format(self.readSensorAddress(),time.ctime()))
             setLedGreen(False)
             validFlag=True
           if in_buffer[2] == STRUCT_TYPE_DHT22:
             probe = self.unpackDHT22Data(join_buffer)
             if probe != None:
               if probe.valid:
                 setLedGreen(True)
                 self.publish("Sensor {} D:{:.1f} O:{:.1f}  - {} VCC:{}V - DHT22   T:{:.2f} C H:{} %".format(self.readSensorAddress(),timeOffset,self.timeOffsetAdjustment,time.ctime(),probe.voltage,probe.temperature,probe.humidity))
                 self.publishValue("DHT22/Temp",probe.temperature)
                 self.publishValue("DHT22/Hum",probe.humidity)
                 self.publishValue("BatteryV","{:.3f}".format(probe.voltage))
                 setLedGreen(False)
               else:
                 self.publish("Sensor {} D:{:.1f} O:{:.1f} - {} VCC:{}V - DHT22   Unable to read".format(self.readSensorAddress(),timeOffset,self.timeOffsetAdjustment,time.ctime(),probe.voltage))
               validFlag=True

           if in_buffer[2] == STRUCT_TYPE_DS18B20:
             probe = self.unpackDS18B20Data(join_buffer)
             if probe != None:
               if probe.valid:
                 setLedGreen(True)
                 self.publish("Sensor {} D:{:.1f} O:{:.1f} - {} VCC:{}V - DS18B20 T:{} C".format(self.readSensorAddress(),timeOffset,self.timeOffsetAdjustment,time.ctime(),probe.voltage,probe.temperature))
                 print("probe.temperature",probe.temperature)
                 self.publishValue("DS18B20",probe.temperature)
                 self.publishValue("BatteryV","{:.3f}".format(probe.voltage))
                 setLedGreen(False)
               else:
                 self.publish("Sensor {} D:{:.1f} O:{:.1f} - {} VCC:{}V - DS18B20 Unable to read DS18B20 sensor".format(self.readSensorAddress(),timeOffset,self.timeOffsetAdjustment,time.ctime(),probe.voltage))
               validFlag=True

           if in_buffer[2] == STRUCT_TYPE_ANALOG:
             probe = self.unpackAnalogData(join_buffer)
             if probe != None:
               if probe.valid:
                 setLedGreen(True)
                 self.publish("Sensor {} D:{:.1f} O:{:.1f} - {} VCC:{}V - A0:{:.3f} A1:{:.3f} A2:{:.3f} A3:{:.3f}".format(self.readSensorAddress(),timeOffset,self.timeOffsetAdjustment,time.ctime(),probe.voltage,probe.Analog0,probe.Analog1, probe.Analog2,probe.Analog3))
                 self.publishValue("Analog/0","{:.3f}".format(probe.Analog0))
                 self.publishValue("Analog/1","{:.3f}".format(probe.Analog1))
                 self.publishValue("Analog/2","{:.3f}".format(probe.Analog2))
                 self.publishValue("Analog/3","{:.3f}".format(probe.Analog3))
                 self.publishValue("BatteryV","{:.3f}".format(probe.voltage))
                 setLedGreen(False)
               else:
                 self.publish("Sensor {} D:{:.1f} O:{:.1f} - {} VCC:{}V - Unable to read Analog sensor".format(self.readSensorAddress(),timeOffset,self.timeOffsetAdjustment,time.ctime(),probe.voltage))
               validFlag=True

           if in_buffer[2] == STRUCT_TYPE_MOISTURE:
             if Debug:
               print("Got moisture")
             probe = self.unpackMoistureData(join_buffer)
             if probe != None:
               if probe.valid:
                 setLedGreen(True)
                 self.publish("Sensor {} D:{:.1f} O:{:.1f} - {} VCC:{}V - MOIST A0:{:.3f} A1:{:.3f} F:{:6d}".format(self.readSensorAddress(),timeOffset,self.timeOffsetAdjustment,time.ctime(),probe.voltage,probe.Analog0,probe.Analog1, probe.Frequency))
                 self.publishValue("Moist/Analog/0","{:.3f}".format(probe.Analog0))
                 self.publishValue("Moist/Analog/1","{:.3f}".format(probe.Analog1))
                 self.publishValue("Moist/Frequency","{:8d}".format(probe.Frequency))
                 self.publishValue("BatteryV","{:.3f}".format(probe.voltage))
                 setLedGreen(False)
               else:
                 self.publish("Sensor {} D:{:.1f} O:{:.1f} - {} VCC:{}V - Unable to read Moisture sensor".format(self.readSensorAddress(),timeOffset,self.timeOffsetAdjustment,time.ctime(),probe.voltage))
               validFlag=True




           if in_buffer[2] == STRUCT_TYPE_MAX6675:
             probe = self.unpackMAX6675Data(join_buffer)
             if probe != None:
               if probe.valid:
                 setLedGreen(True)
                 self.publish("Sensor {} D:{:.1f} O:{:.1f} - {} VCC:{}V - MAX6675 T:{} C".format(self.readSensorAddress(),timeOffset,self.timeOffsetAdjustment,time.ctime(),probe.voltage,probe.temperature))
                 setLedGreen(False)
               else:
                 self.publish("Sensor {} D:{:.1f} O:{:.1f} - {} VCC:{}V - MAX6675 Unable to read sensor".format(self.readSensorAddress(),timeOffset,self.timeOffsetAdjustment,time.ctime(),probe.voltage))
               validFlag=True

           if validFlag:
             self.gotDataFlag=True
             self.timeOutFlag=False

             if probe == None:
                self.NoAdjustmentOnNext= True
             else:
                if probe.timeOut:
                  self.NoAdjustmentOnNext= True
#             if not self.NoAdjustmentOnNext:
             if abs(timeOffset)<5:
                 self.timeOffsetAdjustment -= timeOffset / 3
             self.NoAdjustementOnNext=False


#           print("Next time {} in sec{}".format(time.ctime(self.nextConnectionTime),stepNextTime))
           if not validFlag:
            print("Sensor {} - {}  Invalid packet!".format(self.readSensorAddress(),time.ctime()))
    lock.release()


masterAddress = [0xe7, 0xe7, 0xe7, 0xe7, 0xe7]





#  For Moisture sensor clock frequency On PD9 is available
#  default value is 800KHz
#  because of  cpu_clock of 8Mhz or 16MHz step frequency are limited
#
#
#  this is the possible range for 8 and 16Mhz cpu
#
#
# Moisture PD9 clock range
#
#   333333, 363636, 400000, 444444, 500000, 571429, 666667, 800000, 1000000, 1333333
#

device = [
          RF_Device([0xc2,0xc2,0xc2,0xc2,0xc3],60),
          RF_Device([0xc2,0xc2,0xc2,0xc2,0xc4],60),
#         RF_Device([0xc2,0xc2,0xc2,0xc2,0xc5],60),
          RF_Device([0xc2,0xc2,0xc2,0xc2,0xc8],60),
          RF_Device([0xc2,0xc2,0xc2,0xc2,0xc6],60),
          RF_Device([0xc2,0xc2,0xc2,0xc2,0xc1],60,clockFrequency=800000)
         ]





# Moisture sensor V2.0 pacb is ticker set sensor clock higher

# space each device in time 
delay=0
for i in device:
  i.mqtt=client
  i.targetConnectionTime += delay * i.timeInterval/len(device)
  print("Device {} set to {}".format(i.readSensorAddress(),time.ctime(i.targetConnectionTime)))
  delay+=1

radio = NRF24(GPIO, spidev.SpiDev())
radio.begin(0, 18)
time.sleep(1)
radio.setRetries(7,4)
radio.setPayloadSize(32)
radio.setChannel(78)

radio.setDataRate(NRF24.BR_250KBPS)
radio.setPALevel(NRF24.PA_MAX)
radio.setAutoAck(True)
radio.enableDynamicPayloads()
radio.enableAckPayload()


radio.openWritingPipe(device[0].deviceAddress)
radio.openReadingPipe(1, masterAddress)



### fix print details in python3
if sys.version_info[0] > 2:
    # need to deal with new line
    old_stdout =  sys.stdout
    v = []
    sys.stdout = bypass_stdout(V=v)
    radio.printDetails()      # print basic detals of radio
    sys.stdout = old_stdout
    print(''.join(v).replace('=\n','= ').replace('\n0x',' 0x').replace('\n\n','\n').replace('=  ','= '))
else:
    radio.printDetails()
###########################


time.sleep(1)
setLedBlue(False)

try:

 while True:
   for i in  device:
     if i.isScheduleUp():
        i.lastPing = time.time();
        i.getData()
        if client is not None:
          client.loop(timeout=0.05)
     time.sleep(0.05)


except KeyboardInterrupt:
    radio.stopListening()
    radio.powerDown()
    client.disconnect()
    raise
