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
 *
 */

#include <WProgram.h>
#include "afsk.h"

// Timing constants
// All of the calculations are done in the precompiler in float point, and then
// converted to fixed point at the end (scaled up by FIXED_PNT_SCALE)

#define SAMPLES_PER_CYCLE 512 // How many samples in our sin-wave table

#define INTERRUPT_RATE 8  // in Microseconds
#define PLAYBACK_RATE (1000000.0f / INTERRUPT_RATE)     // In Hertz

// 1200 Baud settings
#define BAUD_RATE 1200.0f
#define MARK_FREQUENCY 1200.0f
#define SPACE_FREQUENCY 2200.0f
#define TONE_FREQUENCY 300.0f

// How many samples to send from the sin table for each bit
// This is a potential source of output timing drift, as this isn't very precise
// at low INTERRUPT_RATEs
// At 16 microseconds it is off by 0.1%
#define SAMPLE_COUNTER_SCALE 1000  // Increase the resolution of the sample counter
#define SAMPLES_PER_BIT_SCALED (int32_t)(SAMPLE_COUNTER_SCALE * PLAYBACK_RATE / BAUD_RATE + 0.5f)

// Fixed point arithmentic on the stride length through the sin wave table
// Note, the bottom "FIXED_PNT_BITS" bits of the counter are the fractional part
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

// The scale of the second sin table
#define TABLE2_SCALE 1.0f

// Store only 1/4th of a sin wave (plus one byte) to save progmem space.
// afsk_read_sample() does the math for the other 3/4ths.
const uint16_t afsk_sine_table1[129] = { 2047, 2072, 2097, 2122, 2147, 2173,
    2198, 2223, 2248, 2273, 2298, 2323, 2348, 2372, 2397, 2422, 2447, 2471,
    2496, 2520, 2545, 2569, 2593, 2617, 2642, 2666, 2689, 2713, 2737, 2761,
    2784, 2807, 2831, 2854, 2877, 2900, 2923, 2945, 2968, 2990, 3012, 3035,
    3056, 3078, 3100, 3121, 3143, 3164, 3185, 3206, 3226, 3247, 3267, 3287,
    3307, 3327, 3346, 3366, 3385, 3404, 3422, 3441, 3459, 3477, 3495, 3513,
    3530, 3547, 3564, 3581, 3598, 3614, 3630, 3646, 3662, 3677, 3692, 3707,
    3721, 3736, 3750, 3764, 3777, 3791, 3804, 3816, 3829, 3841, 3853, 3865,
    3876, 3887, 3898, 3909, 3919, 3929, 3939, 3949, 3958, 3967, 3975, 3984,
    3992, 3999, 4007, 4014, 4021, 4027, 4034, 4040, 4045, 4051, 4056, 4060,
    4065, 4069, 4073, 4076, 4080, 4083, 4085, 4087, 4089, 4091, 4093, 4094,
    4094, 4095, 4095, };

#define SINSCALE(x)     (uint16_t)((((float)(x)-2047.0f) * TABLE2_SCALE) + 2047.49f)

// After noticing on the oscilloscope that one of the frequency signals come accross at
// higher aplitutde than the others, I added the ability to have two sine-wave tables,
// one a scale from the other. The scale (TABLE2_SCALE) was should be determined experimentally.
const uint16_t afsk_sine_table2[129] = { SINSCALE(2047), SINSCALE(2072),
    SINSCALE(2097), SINSCALE(2122), SINSCALE(2147), SINSCALE(2173), SINSCALE(
        2198), SINSCALE(2223), SINSCALE(2248), SINSCALE(2273), SINSCALE(2298),
    SINSCALE(2323), SINSCALE(2348), SINSCALE(2372), SINSCALE(2397), SINSCALE(
        2422), SINSCALE(2447), SINSCALE(2471), SINSCALE(2496), SINSCALE(2520),
    SINSCALE(2545), SINSCALE(2569), SINSCALE(2593), SINSCALE(2617), SINSCALE(
        2642), SINSCALE(2666), SINSCALE(2689), SINSCALE(2713), SINSCALE(2737),
    SINSCALE(2761), SINSCALE(2784), SINSCALE(2807), SINSCALE(2831), SINSCALE(
        2854), SINSCALE(2877), SINSCALE(2900), SINSCALE(2923), SINSCALE(2945),
    SINSCALE(2968), SINSCALE(2990), SINSCALE(3012), SINSCALE(3035), SINSCALE(
        3056), SINSCALE(3078), SINSCALE(3100), SINSCALE(3121), SINSCALE(3143),
    SINSCALE(3164), SINSCALE(3185), SINSCALE(3206), SINSCALE(3226), SINSCALE(
        3247), SINSCALE(3267), SINSCALE(3287), SINSCALE(3307), SINSCALE(3327),
    SINSCALE(3346), SINSCALE(3366), SINSCALE(3385), SINSCALE(3404), SINSCALE(
        3422), SINSCALE(3441), SINSCALE(3459), SINSCALE(3477), SINSCALE(3495),
    SINSCALE(3513), SINSCALE(3530), SINSCALE(3547), SINSCALE(3564), SINSCALE(
        3581), SINSCALE(3598), SINSCALE(3614), SINSCALE(3630), SINSCALE(3646),
    SINSCALE(3662), SINSCALE(3677), SINSCALE(3692), SINSCALE(3707), SINSCALE(
        3721), SINSCALE(3736), SINSCALE(3750), SINSCALE(3764), SINSCALE(3777),
    SINSCALE(3791), SINSCALE(3804), SINSCALE(3816), SINSCALE(3829), SINSCALE(
        3841), SINSCALE(3853), SINSCALE(3865), SINSCALE(3876), SINSCALE(3887),
    SINSCALE(3898), SINSCALE(3909), SINSCALE(3919), SINSCALE(3929), SINSCALE(
        3939), SINSCALE(3949), SINSCALE(3958), SINSCALE(3967), SINSCALE(3975),
    SINSCALE(3984), SINSCALE(3992), SINSCALE(3999), SINSCALE(4007), SINSCALE(
        4014), SINSCALE(4021), SINSCALE(4027), SINSCALE(4034), SINSCALE(4040),
    SINSCALE(4045), SINSCALE(4051), SINSCALE(4056), SINSCALE(4060), SINSCALE(
        4065), SINSCALE(4069), SINSCALE(4073), SINSCALE(4076), SINSCALE(4080),
    SINSCALE(4083), SINSCALE(4085), SINSCALE(4087), SINSCALE(4089), SINSCALE(
        4091), SINSCALE(4093), SINSCALE(4094), SINSCALE(4094), SINSCALE(4095),
    SINSCALE(4095),

};

// We keep potentially 2 different tables for the two frequencies.
// The idea has to do with the complications of pre-emphasis if this signal
// is put on the microphone input of a radio that tries to amplify
// the higher frequencies to get a better SNR. Read about it here:
// http://en.wikipedia.org/wiki/Preemphasis_improvement
#define SIN_TABLE_MARK afsk_sine_table2
#define SIN_TABLE_SPACE afsk_sine_table1

// Module globals
volatile bool busy = false;                         // Modem is on

static const uint16_t *afsk_sine_table; // The sin table we are going through
static uint32_t sine_table_switcher; // The bit mask to switch sin tables quick

static uint32_t currentSampleCount; // How many output sample have been sent for the current bit
static uint16_t currentStride;    // 1200/2200 for standard AX.25
static uint16_t currentSample;           // Fixed point 
static uint32_t totalSamplesSent;
static uint8_t current_byte = 0;

// Buffer of bytes to send
static const uint8_t *afsk_packet;
static uint32_t afsk_packet_size = 0; // How many bytes in the packet
static uint32_t packet_pos;       // Next bit to be sent out 

static bool fDoTone;
static uint32_t toneStop, silenceStop;

static uint8_t pttPin; // 0 indicates no PTT
static uint16_t pttDelay;
static uint32_t toneLength;
static uint32_t silenceLength;

IntervalTimer timer;

// Returns an entry from the sin table
// The sin table is 128 elements long, but it emulates a 512 element table
// by reflecting and duplicating the first quarterwave
inline uint16_t afsk_read_sample(const uint16_t * const sine_table,
    const uint16_t phase)
{
  if (phase <= 127) {
    return sine_table[phase];
  } else
    if (phase <= 255) {
      return sine_table[256 - phase];
    } else
      if (phase <= 383) {
        return (uint16_t) 4097 - sine_table[phase - 256];
      } else {
        return (uint16_t) 4097 - sine_table[512 - phase];
      }
}

inline void afsk_output_sample(uint16_t b)
{
  analogWrite(A14, b);
}

void afsk_timer_stop()
{
  timer.end();
  afsk_output_sample(2047); // 50%
}

// Should be called to put out the next sample
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
      if (pttPin) {
        pinMode(pttPin, OUTPUT);
        digitalWrite(pttPin, LOW);
      }
      busy = false;         // End of transmission
      return;       // Done.
    }

    // If sent SAMPLES_PER_BAUD already, go to the next bit
    if (currentSampleCount < SAMPLE_COUNTER_SCALE) {    // Load up next bit
      if ((packet_pos & 7) == 0)          // Load up next byte
        current_byte = afsk_packet[packet_pos >> 3];
      else
        current_byte = current_byte / 2;  // ">>1" forces int conversion
      if ((current_byte & 1) == 0) {
        // Toggle tone (1200 <> 2200)
        currentStride ^= (SAMPLE_STRIDE_MARK_FIXED_PNT
            ^ SAMPLE_STRIDE_SPACE_FIXED_PNT);
        afsk_sine_table = (uint16_t *) ((uint32_t) afsk_sine_table
            ^ sine_table_switcher);
      }
    }
    currentSampleCount += SAMPLE_COUNTER_SCALE;
    if (currentSampleCount >= SAMPLES_PER_BIT_SCALED) { // Is this bit done
      currentSampleCount -= SAMPLES_PER_BIT_SCALED; // Leave residual (fractional) count behind in counter
      packet_pos++; // Go to the next bit
    }
  }
  currentSample += currentStride;
  uint16_t s = afsk_read_sample(afsk_sine_table,
      (uint16_t) ((currentSample >> FIXED_PNT_BITS) & (SAMPLES_PER_CYCLE - 1)));
  totalSamplesSent += 1;
  afsk_output_sample(s);
}

void afsk_timer_start()
{
  timer.begin(interrupt, INTERRUPT_RATE);
}

void afsk_setup(const uint8_t p_pttPin, // Use PTT pin, 0 = do not use PTT
    const uint16_t p_pttDelay, // ms to wait after PTT to transmit
    const uint32_t p_toneLength, const uint32_t p_silenceLength)
{

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
}

void afsk_set_buffer(const uint8_t * const buffer, const uint16_t len)
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
  sine_table_switcher =
      ((uint32_t) SIN_TABLE_MARK ^ (uint32_t) SIN_TABLE_SPACE);

  if (pttPin) {
    // Key the radio
    // Configure pins
    pinMode(pttPin, INPUT);
    delay(pttDelay);
  }

  // Start transmission
  afsk_timer_start();
}

