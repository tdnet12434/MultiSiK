// -*- Mode: C; c-basic-offset: 8; -*-
//
// Copyright (c) 2013 Luke Hovington, All Rights Reserved
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
/// @file	tdm.h
///
/// Interface to the time division multiplexing code
///

#ifndef _TDM_H_
#define _TDM_H_

#include <stdbool.h>
#include "board.h"

// Debug pins
//#define DEBUG_PINS_YIELD // P2.6
//#define DEBUG_PINS_SYNC  // P0.1
//#define DEBUG_PINS_TRANSMIT_RECEIVE // P2.2
//#define DEBUG_PINS_TX_RX_STATE // P2.3

// Sync Logic on Pin // P2.2
//#define TDM_SYNC_LOGIC

// Activity logic on Pin // P2.1 P2.2
//#define TDM_ACTIVITY_LOGIC

// Time in seconds to reset the node if in sync mode
#define UNLOCK_RESET_SEC 60

// Hard coded for the inital testing, need to figure out how this could be changed on the fly later down the track
// Maybe a new command seprate from the AT? Without entering +++ mode? more thought needed
#define BASE_NODEID 0
#define USE_TICK_YIELD 1

#ifdef TDM_SYNC_LOGIC
SBIT (TDM_SYNC_PIN, SFR_P2, 2);
#endif // TDM_SYNC_LOGIC

#ifdef TDM_ACTIVITY_LOGIC
#ifdef CPU_SI1030
SBIT (TDM_ACTIVITY_PIN, SFR_P3, 3);
SBIT (TDM_SYNC_PIN, SFR_P1, 2);
#else
SBIT (TDM_ACTIVITY_PIN, SFR_P2, 2);
SBIT (TDM_SYNC_PIN, SFR_P2, 1);
#endif
#define ACTIVITY_LED(logic) TDM_ACTIVITY_PIN = logic; LED_ACTIVITY = logic;
#define RADIO_LED(logic)    TDM_SYNC_PIN = logic;     LED_RADIO = logic;
#else
#define ACTIVITY_LED(logic) LED_ACTIVITY = logic
#define RADIO_LED(logic) LED_RADIO = logic
#endif

/// initialise tdm subsystem
extern void tdm_init(void);

/// tdm main loop
extern void tdm_serial_loop(void);

/// tdm sync state
extern bool tdm_state_sync();

/// setup a 16 bit node count
extern void tdm_set_node_count(__pdata uint16_t count);

/// setup a 16 bit node destination
extern void tdm_set_node_destination(__pdata uint16_t destination);

/// setup if the node can sync from any
extern void tdm_set_sync_any(__pdata uint8_t any);

/// report tdm timings
extern void tdm_report_timing(void);

/// dispatch a remote AT command
extern void tdm_remote_at(__pdata uint16_t destination);

/// show RSSI information
extern void tdm_show_rssi(void);

/// the long term duty cycle we are aiming for
extern __pdata uint8_t duty_cycle;

/// the LBT threshold
extern __pdata uint8_t lbt_rssi;

#endif // _TDM_H_
