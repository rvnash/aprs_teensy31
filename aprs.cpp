/*
 * Copyright (C) 2013 by KC3ARY Rich Nash
 * 
 * Module modified after being inheritted from EA5HAV. The same license described below
 * is in effect.
 *
 * The below copyright is kept intact.
 *
 */

/* Credit to the trackuino project written by EA5HAV Javi
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
#include "afsk.h"
#include "ax25.h"
#include "aprs.h"

#define MAXSENDBUFFER 500 // Used to allocate a static buffer on the stack to build the AX25 buffer

uint16_t preambleFlags;

// Convert latitude from a float to a string
void latToStr(char * const s, const int size, float lat)
{
  char hemisphere = 'N';
  if (lat < 0) {
    lat = -lat;
    hemisphere = 'S';
  }
  const int deg = (int) lat;
  lat = (lat - (float) deg) * 60.0f;
  const int min = (int) lat;
  lat = (lat - (float) min) * 100.0f;
  const int minTenths = (int) (lat + 0.5); // Round the tenths
  snprintf(s, size, "%02d%02d.%02d%c", deg, min, minTenths, hemisphere);
}

// Convert latitude from a float to a string
void lonToStr(char * const s, const int size, float lon)
{
  char hemisphere = 'E';
  if (lon < 0) {
    lon = -lon;
    hemisphere = 'W';
  }
  const int deg = (int) lon;
  lon = (lon - (float) deg) * 60.0f;
  const int min = (int) lon;
  lon = (lon - (float) min) * 100.0f;
  const int minTenths = (int) (lon + 0.5); // Round the tenths
  snprintf(s, size, "%03d%02d.%02d%c", deg, min, minTenths, hemisphere);
}

// Dump out the AX25 packet in a semi-readable way
// Note, the end of header and CRC are sent as ASCII characters, which they aren't
void logBuffer(const uint8_t * const buf, const int bitsSent,
    const uint8_t dayOfMonth, const uint8_t hour, const uint8_t min)
{
  Serial.printf("Bits in packet %d: ", bitsSent);
  Serial.print(dayOfMonth);
  Serial.print(',');
  Serial.print(hour);
  Serial.print(',');
  Serial.print(min);
  Serial.print(',');

  uint8_t frameState = 0; // 0-No start, 1-start, 2-in header, 3-In info
  uint8_t bSoFar = 0x00;
  uint8_t gotBit = 0;
  int numOnes = 0;
  for (int onBit = 0; onBit < bitsSent; onBit++) {
    uint8_t bit = buf[onBit >> 3] & (1 << (onBit & 7));
    if (numOnes == 5) {
      // This may be a 0 due to bit stuffing
      if (bit) { // Maybe it's a 0x7e
        onBit++;
        bit = buf[onBit >> 3] & (1 << (onBit & 7));
        if (gotBit == 6 && bSoFar == 0x3e && !bit) {
          // Got 0x7e frame start/end
          if (frameState == 0) {
            frameState = 1;
            Serial.print('[');
          } else
            if (frameState == 3 || frameState == 2) {
              frameState = 0;
              Serial.print(']');
            }
          bSoFar = 0x00;
          gotBit = 0;
        } else {
          Serial.print('X'); // Error
        }
      }
      numOnes = 0;
      continue;
    }
    if (bit) {
      // Set the one bit
      bSoFar |= (1 << gotBit);
      numOnes++;
    } else {
      numOnes = 0;
    }
    gotBit++;
    if (gotBit == 8) {
      // Got a byte;
      if (frameState == 1) {
        frameState = 2;
      } else
        if (frameState == 2 && bSoFar == 0xf0) { // 0xf0 is the last byte of the header
          frameState = 3;
        }
      if (frameState == 2) {
        // In header
        Serial.print((char) (bSoFar >> 1));
      } else {
        // In info
        Serial.print((char) bSoFar);
      }
      bSoFar = 0x00;
      gotBit = 0;
    }
  }
  Serial.println();
}

void aprs_setup(const uint16_t p_preambleFlags, const uint8_t pttPin,
    const uint16_t pttDelay, const uint32_t toneLength,
    const uint32_t silenceLength)
{
  preambleFlags = p_preambleFlags;
  afsk_setup(pttPin, pttDelay, toneLength, silenceLength);
}

// This can operate in two modes, PTT or VOX
// In VOX mode send a toneLength and silenceLength. The audible tone will be sent for that
// many milliseconds to open up VOX, and then that many MS of silence, then the packet will
// begin.
// In PTT mode the pin given will be raised high, and then PTT_DELAY ms later, the packet will
// begin
void aprs_send(const PathAddress * const paths, const int nPaths,
    const uint8_t dayOfMonth, const uint8_t hour, const uint8_t min,
    const float lat,
    const float lon, // degrees
    const float altitude, // meters
    const uint16_t heading, // degrees
    const float speed, const char symbolTableIndicator, const char symbol,
    const char * const comment)
{
  uint8_t buf[MAXSENDBUFFER];
  char temp[12];

  ax25_initBuffer(buf, sizeof(buf));

  ax25_send_header(paths, nPaths, preambleFlags);

  ax25_send_byte('/'); // Report w/ timestamp, no APRS messaging. $ = NMEA raw data
  snprintf(temp, sizeof(temp), "%02u%02u%02uz", (unsigned int) dayOfMonth,
      (unsigned int) hour, (unsigned int) min);
  ax25_send_string(temp);

  latToStr(temp, sizeof(temp), lat);
  ax25_send_string(temp);             // Lat:

  ax25_send_byte(symbolTableIndicator);           // Which Symbol table to use

  lonToStr(temp, sizeof(temp), lon);
  ax25_send_string(temp);     // Lon: 000deg and 25.80 min

  ax25_send_byte(symbol);  // The symbol

  snprintf(temp, sizeof(temp), "%03u", heading);
  ax25_send_string(temp);             // Heading (degrees)

  ax25_send_byte('/');                // and

  snprintf(temp, sizeof(temp), "%03d", (unsigned int) (speed + 0.5));
  ax25_send_string(temp);             // speed (knots)

  ax25_send_string("/A="); // Altitude (feet). Goes anywhere in the comment area

  snprintf(temp, sizeof(temp), "%06ld", (long) (altitude / 0.3048)); // 10000 ft = 3048 m
  ax25_send_string(temp);

  ax25_send_string(comment);     // Comment

  ax25_send_footer();

  Serial.flush(); // Make sure all Serial data (which is based on interrupts) is done before you start sending.

  // Set the buffer of bits we are going to send
  afsk_set_buffer(buf, ax25_getPacketSize());

  Serial.flush(); // Wait until all characters are sent

  // OK, no more operations until this is done.
  afsk_start();
  while (afsk_busy())
    ;

  logBuffer(buf, ax25_getPacketSize(), dayOfMonth, hour, min);
}
