#!/usr/bin/python3
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)

from lib_nrf24 import NRF24
import time
import spidev
import sys

master = [ 0xe0, 0xE0, 0xE0, 0xE0,1 ]
unit =   [ 0xA5, 0xA5, 0xA5, 0xA5, 0xA5 ]

radio = NRF24(GPIO, spidev.SpiDev())
radio.begin(0, 18)
radio.setChannel(80)
radio.setDataRate(NRF24.BR_250KBPS)
radio.setPayloadSize(32)
radio.setPALevel(NRF24.PA_MIN)
radio.enableDynamicPayloads()
radio.openReadingPipe(1,master)
radio.startListening()


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


print("En ecoute...")
radio.startListening()

try:
	while True:
	#    pipe = [0]
	    while not radio.available():
	        time.sleep(0.01)
	    recv_buffer = []
	    radio.read(recv_buffer, radio.getDynamicPayloadSize())
	    print (bytes(recv_buffer).decode('ascii'))

except KeyboardInterrupt:
      radio.powerDown()
      radio.end()

