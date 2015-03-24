/*
 * Copyright (C) 2014 by KC3ARY Rich Nash
 * 
 * Module modified code from EA5HAV.
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

#ifndef __AFSK_H__
#define __AFSK_H__

#include <stdint.h>

// Exported functions
void
afsk_setup(const uint8_t pttPin, // Use PTT pin, 0 = do not use PTT
    const uint16_t pttDelay, // ms to wait after PTT to transmit
    const uint32_t toneLength, const uint32_t p_silenceLength);

void afsk_set_buffer(const uint8_t * const buffer, const uint16_t len);
void afsk_start();
int afsk_busy();
uint16_t afsk_getTxDelay();

#endif
