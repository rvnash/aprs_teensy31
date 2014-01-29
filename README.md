aprs-teensy31
=============

Holds the source code for generating APRS packet "sounds" on the teensy 3.1's DAC pin.

Notes
* This code is intended to be used with the Arduino Library for Teensy 3.1. For simplicity you can use <a href="http://www.pjrc.com/teensy/teensyduino.html">Teensyduino</a>.
* aprs.h contains the two function calls you need.
* Call aprs_setup with the parameters you want.
* Call aprs_send to send a packet. Note: It will not return until the entire packet has been sent.
