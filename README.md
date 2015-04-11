# nRF24L01_With_Sensor
Read DHT22 sensor via nRF24L01 transmitter with an arduino 

Arduino pin out
  
  <pre>
  2   DHT22     DATA with a 4k7 pull-up resistor
  4   DHT22     POWER  I use IO pin to reduce power
  GND DHT22     GND
  3   nRF24L01  IRQ   pin 8.
  7   nRF24L01  CS    pin 4
  8   nRF24L01  CE pin 3
  11  nRF24L01  MOSI  pin 6
  12  nRF24L01  MISO  pin 7
  13  nRF24L01  SCK   pin 5
  3.3V nRF24L01 VCC   pin 2
  GND nRF24L01  GND p in 1
</pre>

Needed library
 - https://github.com/n0m1/Sleep_n0m1
 - http://playground.arduino.cc//Main/DHTLib
 - http://www.pjrc.com/teensy/td_libs_OneWire.html
 - http://milesburton.com/Dallas_Temperature_Control_Library

And for nRF24Reader.py
 - https://github.com/BLavery/lib_nrf24.git
