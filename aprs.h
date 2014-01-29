/*
 * Copyright (C) 2014 by KC3ARY Rich Nash
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

#ifndef __APRS_H__
#define __APRS_H__

struct PathAddress {
	const char *callsign;
	uint8_t ssid;
};

void aprs_setup(const uint16_t txDelay, // ms to transmit ax25 flag
                const uint8_t pttPin, // Use PTT pin, 0 = do not use PTT
                const uint16_t pttDelay, // ms to wait after PTT to transmit
                const uint32_t p_toneLength, // if <> 0 then emit a sub-audio tone (300hz) before packet to trigger VOX
		const uint32_t p_silenceLength // Length of time (ms) after tone before packet
                );

void aprs_send(const PathAddress *const paths, const int nPaths,
               const uint8_t dayOfMonth,
	       const uint8_t hour,
	       const uint8_t min,
               const float lat,
	       const float lon, // degrees
               const float altitude, // meters
               const uint16_t heading, // degrees
               const float speed,
               const char symbolTableIndicator,
               const char symbol,
               const char *const comment); // Synchronized ... won't return until the entire packet is sent
#endif
