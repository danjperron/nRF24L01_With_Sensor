# nRF24L01_With_Sensor
Lire humidité par un capteur capacitif 

Arduino pin out 
 
  
  <pre>
  GND Batterie, capteur  GND
  7   nRF24L01  CS    pin 4
  8   nRF24L01  CE pin 3
  11  nRF24L01  MOSI  pin 6
  12  nRF24L01  MISO  pin 7
  13  nRF24L01  SCK   pin 5
  3.3V nRF24L01 VCC   pin 2
  GND nRF24L01  GND p in 1
  9   Capteur CLOCK OUT
  A0  capteur signal
  A1  input foure-tout (pas utilis&acute; mais le systeme retourne la valeur)
</pre>


Librairie nécessaire extraite directement d'arduino 
 - RF24 de TMRh20 
 - Sleep_n0m1 de Noah Shibley 


Librairie Python pour le Raspberry Pi 

 - https://github.com/BLavery/lib_nrf24.git
   *** Il faut corriger la fonction begin pour include la vitesse d`horloge du SPI, sinon c'est trop vite.
    self.spidev.max_speed_hz=8000000
   P.S. une version corrigé du lib_nrf24.git est inclus.
 - numpy      sudo pip3 install numpy  (cela va prendre du temps).
 - mosquitto-clients   sudo pip3 install mosquitto-clients
 
 Si vous n'avez pas de serveur MQTT  vous pouvez en installer un sur le PI
   sudo apt install mosquitto


Information sur les répertoires,

- Demo1    Démo  Transférer du texte.
- Demo2    Démo  Transférer des données en packet.
- Demo3    Démo  Transférer des données en packet mais avec le capteur
           capacitif modifié avec le clock de l`arduino et aussi mettre
           le système en mode sleep.

-RF24L01_MOISTURE_V4  Version final incluant le mode du Demo3 mais en shockburst ce qui permet de synchroniser
           et d'avoir plusieur capteurs. 


 

