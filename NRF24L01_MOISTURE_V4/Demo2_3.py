#!/usr/bin/python3
import RPi.GPIO as GPIO  # import gpio
import time      #import time library
import spidev
from lib_nrf24 import NRF24   #import NRF24 library
from struct import *
from datetime import datetime
import sys
import copy

GPIO.setmode(GPIO.BCM)       # set the gpio mode
GPIO.setwarnings(False)

  # set the pipe address. this address shoeld be entered on the receiver alo
serverPipe = [0xE0, 0xE0, 0xE0, 0xE0, 0x01]
#sensorPipe = [0xA5, 0xA5, 0xA5, 0xA5, 0x01]
radio = NRF24(GPIO, spidev.SpiDev())   # use the gpio pins
radio.begin(0, 18)   # start the radio and set the ce,csn pin ce= GPIO08, csn= GPIO25
radio.setPayloadSize(32)  #set the payload size as 32 bytes
radio.setChannel(80) # set the channel as 76 hex
radio.setDataRate(NRF24.BR_250KBPS)    # set radio data rate
radio.setPALevel(NRF24.PA_MIN)  # set PA level
radio.enableDynamicPayloads()
radio.openReadingPipe(1,serverPipe)
radio.stopListening()


## fix python3 printDetails() bug  in  lib_nrf24.py library
## remove extra new line  when is not needed
class Logger(object):
  def __init__(self,V):
    sysout = sys.stdout
    self.line = V

  def write(self, message):
    self.line+= message

  def flush(self):
    pass


if sys.version_info[0] > 2:
    # need to deal with new line
    old_stdout =  sys.stdout
    v = []
    sys.stdout = Logger(V=v)
    radio.printDetails()      # print basic detals of radio
    sys.stdout = old_stdout
    print(''.join(v).replace('=\n','= ').replace('\n0x',' 0x').replace('\n\n','\n').replace('=  ','= '))
else:
    radio.printDetails()

#### end of fix for python 3




radio.startListening()



#############  packet structure info

STRUCT_TYPE_GETDATA=0
STRUCT_TYPE_INIT=1
STRUCT_TYPE_DHT22=2
STRUCT_TYPE_DS18B20=3
STRUCT_TYPE_MAX6675=4
STRUCT_TYPE_DIGITAL_OUTPUT=5
STRUCT_TYPE_ANALOG=6
STRUCT_TYPE_MOISTURE=7


##############  moisture data  class information ##########

class MoistureData:
    ID = 0
    time = None
    Analog0 = 0
    Analog1 = 0
    Frequency = 0
    voltage=0
    valid=False
    timeOut=False


def PrepBuffer(buffer):
    if sys.version_info[0] == 2:
        return ''.join(map(chr,buffer))
    return bytes(buffer)



def unpackMoistureData(buffer):
     Moisture = MoistureData
     if len(buffer) != 19:
         return None
     #try:
     if True:
        rdata = unpack('<sBBBBLHHHL',PrepBuffer(buffer))
        Moisture.ID = rdata[3]
        Moisture.valid = ((rdata[4] & 1) == 1)
        Moisture.timeOut = ((rdata[4] & 2) ==2)
        Moisture.time = rdata[5]
        Moisture.voltage = rdata[6]/1000.0
        Moisture.Analog0 = Moisture.voltage * rdata[7] / 1023.0
        Moisture.Analog1 = Moisture.voltage * rdata[8] / 1023.0
        Moisture.Frequency = rdata[9]
        return Moisture
     #except:
     #   return None
     return None

##############    main loop ######################

try:

	while True:

	    if radio.available(0):
	        in_buffer=[]
	        radio.read(in_buffer,radio.getDynamicPayloadSize())
	        validFlag= False
	        if len(in_buffer)>4:
	            # check if packet is valid
	            if in_buffer[0] == ord('*'):
	                if in_buffer[2] == STRUCT_TYPE_MOISTURE:
	                    #  Got Humidity Sensor data
	                    probe = unpackMoistureData(in_buffer)
	                    if  probe != None:
	                        now = datetime.now().strftime("%d/%m/%Y %H:%M:%S")
	                        if probe.valid:
	                            print("{}\tID:{}\tCap Level:{:.3f}V\tVCC:{}V\tCenterTap:{:.3f}V\tFrequency:{}KHz".format(now,probe.ID,probe.Analog0, probe.voltage, probe.Analog1,probe.Frequency//1000))
	                            validFlag = True
	                        else:
	                            print("{}\tID:{}\t --------  invalid data ------------".format(now,probe.ID))
	    time.sleep(0.01)

except KeyboardInterrupt:
	radio.powerDown()
	radio.end()

