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

#ifndef __AX25_H__
#define __AX25_H__

#include <stdint.h>

struct PathAddress;

void ax25_initBuffer(uint8_t * buf, const int bufSize);
void ax25_send_header(const struct PathAddress * const paths, const uint16_t nPaths,
    const uint16_t txDelay);
void ax25_send_byte(uint8_t a_byte);
void ax25_send_string(const char * const string);
void ax25_send_footer();
unsigned int ax25_getPacketSize();

#endif
