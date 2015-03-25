// Example program for the APRS code in this repository

/*
 * Copyright (C) 2014 by Richard Nash (KC3ARY)
 * 
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <WProgram.h>
// Note: this example uses my GPS library for the Adafruit Ultimate GPS
// Code located here: https://github.com/rvnash/ultimate_gps_teensy3
#include <GPS.h>
#include <aprs.h>

// APRS Information
#define PTT_PIN 13 // Push to talk pin

// Set your callsign and SSID here. Common values for the SSID are
#define S_CALLSIGN      "KC3ARY"
#define S_CALLSIGN_ID   1   // 11 is usually for balloons
// Destination callsign: APRS (with SSID=0) is usually okay.
#define D_CALLSIGN      "APRS"
#define D_CALLSIGN_ID   0
// Symbol Table: '/' is primary table '\' is secondary table
#define SYMBOL_TABLE '/' 
// Primary Table Symbols: /O=balloon, /-=House, /v=Blue Van, />=Red Car
#define SYMBOL_CHAR 'v'

struct PathAddress addresses[] = {
  {(char *)D_CALLSIGN, D_CALLSIGN_ID},  // Destination callsign
  {(char *)S_CALLSIGN, S_CALLSIGN_ID},  // Source callsign
  {(char *)NULL, 0}, // Digi1 (first digi in the chain)
  {(char *)NULL, 0}  // Digi2 (second digi in the chain)
};


HardwareSerial &gpsSerial = Serial1;
GPS gps(&gpsSerial,true);

// setup() method runs once, when the sketch starts
void setup()
{
  Serial.begin(38400); // For debugging output over the USB port
  gps.startSerial(9600);
  delay(1000);
  gps.setSentencesToReceive(OUTPUT_RMC_GGA);

  // Set up the APRS module
  aprs_setup(50, // number of preamble flags to send
	     PTT_PIN, // Use PTT pin
	     100, // ms to wait after PTT to transmit
	     0, 0 // No VOX ton
	     );
}

// Function to broadcast your location
void broadcastLocation(GPS &gps, const char *comment)
{
  // If above 5000 feet switch to a single hop path
  int nAddresses;
  if (gps.altitude > 1500) {
    // APRS recomendations for > 5000 feet is:
    // Path: WIDE2-1 is acceptable, but no path is preferred.
    nAddresses = 3;
    addresses[2].callsign = "WIDE2";
    addresses[2].ssid = 1;
  } else {
    // Below 1500 meters use a much more generous path (assuming a mobile station)
    // Path is "WIDE1-1,WIDE2-2"
    nAddresses = 4;
    addresses[2].callsign = "WIDE1";
    addresses[2].ssid = 1;
    addresses[3].callsign = "WIDE2";
    addresses[3].ssid = 2;
  }

  // For debugging print out the path
  Serial.print("APRS(");
  Serial.print(nAddresses);
  Serial.print("): ");
  for (int i=0; i < nAddresses; i++) {
    Serial.print(addresses[i].callsign);
    Serial.print('-');
    Serial.print(addresses[i].ssid);
    if (i < nAddresses-1)
      Serial.print(',');
  }
  Serial.print(' ');
  Serial.print(SYMBOL_TABLE);
  Serial.print(SYMBOL_CHAR);
  Serial.println();

  // Send the packet
  aprs_send(addresses, nAddresses
	    ,gps.day, gps.hour, gps.minute
	    ,gps.latitude, gps.longitude // degrees
	    ,gps.altitude // meters
	    ,gps.heading
	    ,gps.speed
	    ,SYMBOL_TABLE
	    ,SYMBOL_CHAR
	    ,comment);
}

uint32_t timeOfAPRS = 0;
bool gotGPS = false;
// the loop() methor runs over and over again,
// as long as the board has power
void loop()
{
  if (gps.sentenceAvailable()) gps.parseSentence();

  if (gps.newValuesSinceDataRead()) {
    gotGPS = true; // @TODO: Should really check to see if the location data is still valid
    gps.dataRead();
    Serial.printf("Location: %f, %f altitude %f\n\r",
		  gps.latitude, gps.longitude, gps.altitude);
  }

  if (gotGPS && timeOfAPRS + 60000 < millis()) {
    broadcastLocation(gps, "HELLO" );
    timeOfAPRS = millis();
  }
}

// Called from the powerup interrupt servicing routine.
int main(void)
{
  setup();
  while (true) {
    loop();
    yield();
  }
  return 0; // Never reached.
}
