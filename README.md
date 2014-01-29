aprs-teensy31
============

Holds the source code for generating APRS packet "sounds" on the teensy 3.1's DAC pin.

Notes
-----
* This code is intended to be used with the Arduino Library for Teensy 3.1. For simplicity you can use <a href="http://www.pjrc.com/teensy/teensyduino.html">Teensyduino</a>.
* aprs.h contains the two function calls you need.
  * Call aprs_setup with the parameters you want.
  * Call aprs_send to send a packet. Note: It will not return until the entire packet has been sent.
* The code is structured as generic C-code with no clases or object oriented features. It is purely functional in nature anyway.

Acknowledgement
---------------
This code is based on code retrieved from the Trackuino project on the web <a href="http://www.trackuino.org">here</a>. This is a hardware/software project designed to use the Arduino and a Radiometrix HX1 transmitter as a position tracking system. The code was written by Javier Martin under the same GNU General Public License I am using for this project.