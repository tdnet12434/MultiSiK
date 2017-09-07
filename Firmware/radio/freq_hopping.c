// -*- Mode: C; c-basic-offset: 8; -*-
//
// Copyright (c) 2012 Andrew Tridgell, All Rights Reserved
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  o Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  o Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in
//    the documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
// STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
// OF THE POSSIBILITY OF SUCH DAMAGE.
//

///
/// @file	freq_hopping.c
///
/// Frequency hop managerment
///

#include <stdarg.h>
#include "radio.h"
#include "freq_hopping.h"

/// how many channels are we hopping over
__pdata uint8_t num_fh_channels;

/// whether we current have good lock with the other end
static bool have_radio_lock;

/// current channel
__pdata static volatile uint8_t fhop_channel;

/// map between hopping channel numbers and physical channel numbers
__xdata static uint8_t channel_map[MAX_FREQ_CHANNELS];

// a vary simple array shuffle
// based on shuffle from
// http://benpfaff.org/writings/clc/shuffle.html
static inline void shuffle(__xdata uint8_t *array, uint8_t n) __nonbanked
{
	uint8_t i;
	for (i = 0; i < n - 1; i++) {
		uint8_t j = ((uint8_t)rand()) % n;
		uint8_t t = array[j];
		array[j] = array[i];
		array[i] = t;
	}
}

// initialise frequency hopping logic
void 
fhop_init(uint16_t netid)
{
	uint8_t i;
	// create a random mapping between virtual and physical channel
	// numbers, seeded by the network ID
	for (i = 0; i < num_fh_channels; i++) {
		channel_map[i] = i;
	}
	srand(netid);
	shuffle(channel_map, num_fh_channels);
}

// tell the TDM code what channel to receive on
uint8_t 
fhop_receive_channel(void) __nonbanked
{
	return channel_map[fhop_channel];
}

// tell the TDM code what channel to transmit on
uint8_t 
fhop_sync_channel(void) __nonbanked
{
	// Fixed sync channel
	return channel_map[SYNC_CHANNEL % num_fh_channels];
}

// get the current transmit channel (NOT the map frequency)
uint8_t
get_transmit_channel(void) __nonbanked
{
	return fhop_channel;
}

// set the current transmit channel (NOT the map frequency)
void
set_transmit_channel(uint8_t channel) __nonbanked
{
	fhop_channel = channel;
}

// called when the transmit windows changes owner
void 
fhop_window_change(void) __nonbanked
{
	fhop_channel = (fhop_channel + 1) % num_fh_channels;
	if (!have_radio_lock) {
		// when we don't have lock, listen on the sync channel
		fhop_channel = SYNC_CHANNEL % num_fh_channels;
		debug("Trying RCV on channel %d\n", (int)receive_channel);
	}
}

// called when we get or lose radio lock
void 
fhop_set_locked(bool locked) __nonbanked
{
#if DEBUG
	if (locked && !have_radio_lock) {
		debug("FH lock\n");
	}
#endif
	have_radio_lock = locked;
}

