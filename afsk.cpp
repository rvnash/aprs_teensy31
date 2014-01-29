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
 *
 */

#include <WProgram.h>
#include "afsk.h"

// Timing constants
// All of the calculations are done in the precompiler in float point, and then
// converted to fixed point at the end (scaled up by FIXED_PNT_SCALE)

#define SAMPLES_PER_CYCLE 512 // How many samples in our sin-wave table

// INTERRUPT_RATE sets the rate at which the analog output pin is adjusted
// to generate the sound waves for the audio frequency-shif keying. I've run
// this as low as 8 microseconds and the Teensy 3.1 seems to do just fine.
#define INTERRUPT_RATE 16  // in Microseconds

#define PLAYBACK_RATE (1000000.0f / INTERRUPT_RATE)     // In Hertz

// 1200 Baud settings
#define BAUD_RATE 1200.0f
#define MARK_FREQUENCY 1200.0f
#define SPACE_FREQUENCY 2200.0f
#define TONE_FREQUENCY 300.0f

// How many samples to send from the sin table for each bit. This is a
// integer, which makes it a potential source of output timing drift, 
// as this isn't very precise at large INTERRUPT_RATEs
// At 16 microseconds it is off by 0.1%
// @TODO: Perhaps the same fixed point trick that is used in the stride calculation
// could be used here too, which would make it more accurate.
#define SAMPLES_PER_BIT (uint16_t)(PLAYBACK_RATE / BAUD_RATE + 0.5f)

// Fixed point arithmentic on the stride length through the sin wave table
// Note, the bottom "FIXED_PNT_BITS" bits of the counter are the fractional part
// This is done to give more accuracy to the tones' frequency
#define FIXED_PNT_BITS 6
#define FIXED_PNT_SCALE (1 << FIXED_PNT_BITS)

// The stride rates below are how much to add to the index through the sin table
// for each interrupt call
#define SAMPLE_STRIDE_TONE_FLOAT ((float)SAMPLES_PER_CYCLE * TONE_FREQUENCY / PLAYBACK_RATE)
#define SAMPLE_STRIDE_TONE_FIXED_PNT ((uint16_t)(SAMPLE_STRIDE_TONE_FLOAT * (float)FIXED_PNT_SCALE + 0.5f))

#define SAMPLE_STRIDE_MARK_FLOAT ((float)SAMPLES_PER_CYCLE * MARK_FREQUENCY / PLAYBACK_RATE)
#define SAMPLE_STRIDE_MARK_FIXED_PNT ((uint16_t)(SAMPLE_STRIDE_MARK_FLOAT * (float)FIXED_PNT_SCALE + 0.5f))

#define SAMPLE_STRIDE_SPACE_FLOAT ((float)SAMPLES_PER_CYCLE * SPACE_FREQUENCY / PLAYBACK_RATE)
#define SAMPLE_STRIDE_SPACE_FIXED_PNT ((uint16_t)(SAMPLE_STRIDE_SPACE_FLOAT * (float)FIXED_PNT_SCALE + 0.5f))

// We keep potentially 2 different tables for the two frequencies.
// The idea has to do with the complications of pre-emphasis if this signal
// is put on the microphone input of a radio that tries to amplify
// the higher frequencies to get a better SNR. Read about it here:
// http://en.wikipedia.org/wiki/Preemphasis_improvement
#define SIN_TABLE_MARK afsk_sine_table2
#define SIN_TABLE_SPACE afsk_sine_table1

// Keep the sin wave data in a different file as to not take up space
#include "progmemSin.h"


// Module globals
volatile bool busy = false;                         // Modem is on

static uint16_t *afsk_sine_table; // The sin table we are going through
static uint32_t sine_table_switcher; // The bit mask to switch sin tables quick

static uint16_t currentSampleCount;    // 1 bit = SAMPLES_PER_BAUD samples
static uint16_t currentStride;    // 1200/2200 for standard AX.25
static uint16_t currentSample;           // Fixed point 
static uint32_t totalSamplesSent; // Book keeping
static uint8_t current_byte = 0;   // The current outgoing byte


// Buffer of bytes to send
static const uint8_t *afsk_packet;  // The buffer of bytes we are sending
static uint32_t afsk_packet_size = 0; // How many bytes in the packet
static uint32_t packet_pos;       // Next bit to be sent out 

static bool fDoTone;
static uint32_t toneStop, silenceStop;

static uint16_t txDelay;
static uint8_t pttPin; // 0 indicates no PTT
static uint16_t pttDelay;
static uint32_t toneLength;
static uint32_t silenceLength;

IntervalTimer timer; // The timer used for tone generation.


// Returns an entry from the sin table
// The sin table is 128 elements long, but it emulates a 512 element table
// by reflecting and duplicating the first quarterwave
inline uint16_t afsk_read_sample(const uint16_t *const sine_table, const uint16_t phase)
{
    if (phase <= 127) {
        return sine_table[phase];
    } else if (phase <= 255) {
        return sine_table[256-phase];
    } else if (phase <= 383) {
        return (uint16_t)4097 - sine_table[phase-256];
    } else {
        return (uint16_t)4097 - sine_table[512-phase];
    }
}

inline void afsk_output_sample( uint16_t b)
{
  analogWrite(A14, b); // I love the teensy 3.1. This is soooo much harder on the Arduino!
}

void afsk_timer_stop()
{
  timer.end();
  afsk_output_sample(2047); // 50% audio out
}

// Called at INTERRUPT_RATE to put out the next sample
// This is called from within an interrupt service
// This should be as tight as possible
// It's grown bloated a bit since I added the VOX opening pre-tone code
void interrupt(void)
{
  if (fDoTone) {
    // Send the sound and the silence.
    const uint32_t now = millis();
    if (toneStop) {
      if (now >= toneStop) {
	toneStop = 0;
      }
    } else {
      currentSample -= currentStride; // stop the stride to create silence
      if (now >= silenceStop) {
	// Times up, play the afsk packet now
	fDoTone = false;
	currentStride = SAMPLE_STRIDE_MARK_FIXED_PNT;
	currentSample = 0;

      }
    }
  } else {
    // Send the packet one bit at a time
    // If done sending packet
    if (packet_pos == afsk_packet_size) {
      afsk_timer_stop();  // Disable timer
      delay(100); // Leave key open for .1 seconds
      if (pttPin) digitalWrite(pttPin, LOW);
      busy = false;         // End of transmission
      return;       // Done.
    }

    // If sent SAMPLES_PER_BAUD already, go to the next bit
    if (currentSampleCount == 0) {    // Load up next bit
      if ((packet_pos & 7) == 0)          // Load up next byte
	current_byte = afsk_packet[packet_pos >> 3];
      else
	current_byte = current_byte / 2;  // ">>1" forces int conversion
      if ((current_byte & 1) == 0) {
	// Toggle tone (1200 <> 2200)
	// Use XOR's to quickly toggle these values without if statements
	currentStride ^= (SAMPLE_STRIDE_MARK_FIXED_PNT ^ SAMPLE_STRIDE_SPACE_FIXED_PNT);
	afsk_sine_table = (uint16_t *)((uint32_t)afsk_sine_table ^ sine_table_switcher);
      }
    }
    currentSampleCount += 1;
    if(currentSampleCount >= SAMPLES_PER_BIT) {
      currentSampleCount = 0;
      packet_pos++;
    }
  }
  currentSample += currentStride;
  uint16_t s = afsk_read_sample(afsk_sine_table, (uint16_t)((currentSample >> FIXED_PNT_BITS) & (SAMPLES_PER_CYCLE-1)));
  totalSamplesSent++;
  afsk_output_sample(s);
}

void afsk_timer_start()
{
  timer.begin( interrupt, INTERRUPT_RATE );
}

void afsk_setup(const uint16_t p_txDelay, // ms to transmit ax25 flag
                const uint8_t p_pttPin, // Use PTT pin, 0 = do not use PTT
                const uint16_t p_pttDelay, // ms to wait after PTT to transmit
                const uint32_t p_toneLength, const uint32_t p_silenceLength)
{
    
    txDelay = p_txDelay;
    pttPin = p_pttPin;
    pttDelay = p_pttDelay;
    toneLength = p_toneLength;
    silenceLength = p_silenceLength;
    
    if (p_pttPin) {
        // Setup radio
        // Configure pins
        pinMode(pttPin, OUTPUT);
        digitalWrite(pttPin, LOW);
    }
    analogWriteResolution(12);
    afsk_output_sample(2047); // 50%
    digitalWrite(pttPin, LOW);
}

void afsk_set_buffer(const uint8_t *const buffer, const uint16_t len)
{
    afsk_packet_size = len; // Length in bits
    afsk_packet = buffer;
}

int afsk_busy()
{
    return busy;
}

// toneLength: How long to play the pre signal tone, (0 = no tone)
// silenceLength: How long of silence before starting the packet
void afsk_start()
{
  // Setup for AFSK transmission
  afsk_sine_table = SIN_TABLE_MARK;
  currentSample = 0;
  packet_pos = 0;
  currentSampleCount = 0;
  totalSamplesSent = 0;
  busy = true;
  if (toneLength == 0) {
    fDoTone = false;
    currentStride = SAMPLE_STRIDE_MARK_FIXED_PNT;
  } else {
    fDoTone = true;
    toneStop = millis() + toneLength;
    silenceStop = toneStop + silenceLength;
    currentStride = SAMPLE_STRIDE_TONE_FIXED_PNT;
  }
  sine_table_switcher = ((uint32_t)SIN_TABLE_MARK ^ (uint32_t)SIN_TABLE_SPACE);
  
  if (pttPin) {
    // Key the radio
    // Configure pins
    digitalWrite(pttPin, HIGH);
    delay(pttDelay);
  }
  
  // Start transmission
  afsk_timer_start();
}

uint16_t afsk_getTxDelay()
{
  return txDelay;
}
