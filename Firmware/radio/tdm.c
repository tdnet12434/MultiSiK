// -*- Mode: C; c-basic-offset: 8; -*-
//
// Copyright (c) 2013 Luke Hovington, All Rights Reserved
// Copyright (c) 2012 Andrew Tridgell, All Rights Reserved
// Copyright (c) 2011 Michael Smith, All Rights Reserved
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
/// @file	tdm.c
///
/// time division multiplexing code
///

#include <stdarg.h>
#include "radio.h"
#include "tdm.h"
#include "timer.h"
#include "packet.h"
#include "golay.h"
#include "freq_hopping.h"
#include "crc.h"

/// the state of the tdm system
enum tdm_state { TDM_TRANSMIT, TDM_RECEIVE, TDM_SYNC };
__pdata static enum tdm_state tdm_state;
__pdata static uint16_t nodeTransmitSeq; // sequence the nodes can transmit in.
__pdata static uint16_t paramNodeDestination; // User defined Packet destination
__pdata static uint16_t nodeDestination; // Real Packet Destination (as some messages should be broadcasted)

/// a packet buffer for the TDM code
__xdata uint8_t	pbuf[MAX_PACKET_LENGTH];

/// how many 16usec ticks are remaining in the current state
__pdata static uint16_t tdm_state_remaining;

/// This is enough to hold at least 3 packets and is based
/// on the configured air data rate.
__pdata static uint16_t tx_window_width;
__pdata static uint16_t tx_sync_width;

/// the maximum data packet size we can fit
__pdata static uint8_t max_data_packet_length;

/// the silence period between transmit windows
/// This is calculated as the number of ticks it would take to transmit
/// two zero length packets
__pdata static uint16_t silence_period;

// Half the linkupdate frequency to display test data
static __bit test_display_toggle;

#if USE_TICK_YIELD
// records if the node so far has yielded to us,
// as soon as a node doesn't yield we stop transmitting until our turn again
__pdata static uint16_t lastTransmitWindow;
// if it's our transmitters turn, we have yielded and someone else has transmitted
static __bit  received_packet;
static __bit  yielded_slot; 

/// whether we have yielded our window to the other radio, or should send a yield packet
static __bit transmit_yield;

enum tdm_yield { YIELD_SET=true, YIELD_GET=false, YIELD_TRANSMIT=true, YIELD_RECEIVE=false, YIELD_NO_DATA=false, YIELD_DATA=true };
#endif // USE_TICK_YIELD

// activity indication
// when the 16 bit timer2_tick() value wraps we check if we have received a
// packet since the last wrap (ie. every second)
// If we have the green radio LED is held on.
// Otherwise it blinks every 1 seconds. The received_sync flag
// is set for any received sync packet
static __bit blink_state;
static __bit received_sync;
__pdata static uint8_t sync_count; // the amount of successfull times synced 
static __bit sync_any;

/// the latency in 16usec timer2 ticks for sending a zero length packet
__pdata static uint16_t packet_latency;

/// the time in 16usec ticks for sending one byte
__pdata static uint16_t ticks_per_byte;

/// number of 16usec ticks to wait for a preamble to turn into a packet
/// This is set when we get a preamble interrupt, and causes us to delay
/// sending for a maximum packet latency. This is used to make it more likely
/// that two radios that happen to be exactly in sync in their sends
/// will eventually get a packet through and get their transmit windows
/// sorted out
__pdata uint16_t transmit_wait;

/// the long term duty cycle we are aiming for
__pdata uint8_t duty_cycle;

/// the average duty cycle we have been transmitting
__data static float average_duty_cycle;

/// duty cycle offset due to temperature
__pdata uint8_t duty_cycle_offset;

/// set to true if we need to wait for our duty cycle average to drop
static bool duty_cycle_wait;

/// how many ticks we have transmitted for in this TDM round
__pdata static uint16_t transmitted_ticks;

/// the LDB (listen before talk) RSSI threshold
__pdata uint8_t lbt_rssi;

/// how long we have listened for for LBT
__pdata static uint16_t lbt_listen_time;

/// how long we have to listen for before LBT is OK
__pdata static uint16_t lbt_min_time;

/// random addition to LBT listen time (see European regs)
__pdata static uint16_t lbt_rand;

/// test data to display in the main loop. Updated when the tick
/// counter wraps, zeroed when display has happened
__pdata uint8_t test_display;

// Statisics packet recive count
__pdata uint16_t statistics_receive_count;
// set to 0 when we should send statistics packets
__pdata uint16_t statistics_transmit_stats;
// handle ati5 command, as this is a long and doesn't fit into the buffer
__pdata uint8_t ati5_id;

/// set when we should send a MAVLink report pkt
extern bool seen_mavlink;

struct tdm_trailer {
	uint16_t window:13;
	uint16_t command:1;
	uint16_t bonus:1;
	uint16_t resend:1;
	uint16_t nodeid;
};
__pdata struct tdm_trailer trailer;

/// buffer to hold a remote AT command before sending
static __bit            send_at_command;
static __pdata uint16_t send_at_command_to;
static __xdata char remote_at_cmd[AT_CMD_MAXLEN + 1];

// local nodeCount
__pdata static uint16_t nodeCount;

/// display RSSI output
void
tdm_show_rssi(void)
{
	// Using printfl helps a bit but still overloads the cpu when AT&T=RSSI is used.
	// This causes pauses and eventualy the nodes drift out of sync
	__pdata uint8_t i;
	for(i=0; i<(nodeCount-1) && i<MAX_NODE_RSSI_STATS; i++)
	{
		if (i != nodeId) {
			printfl("[%u] L/R RSSI: %u/%u  L/R noise: %u/%u\n",
				   (unsigned)i,
				   (unsigned)statistics[i].average_rssi,
				   (unsigned)remote_statistics[i].average_rssi,
				   (unsigned)statistics[nodeId].average_noise,
				   (unsigned)remote_statistics[i].average_noise);
		}
	}
	printfl("[%u] pkts: %u txe=%u rxe=%u stx=%u srx=%u ecc=%u/%u temp=%d dco=%u\n",
		   (unsigned)nodeId,
		   (unsigned)statistics_receive_count,
		   (unsigned)errors.tx_errors,
		   (unsigned)errors.rx_errors,
		   (unsigned)errors.serial_tx_overflow,
		   (unsigned)errors.serial_rx_overflow,
		   (unsigned)errors.corrected_errors,
		   (unsigned)errors.corrected_packets,
		   (int)radio_temperature(),
		   (unsigned)duty_cycle_offset);
	statistics_receive_count = 0;
}

/// display test output
static void
display_test_output(void)
{
	if (test_display & AT_TEST_RSSI) {
		tdm_show_rssi();
	}
}


/// estimate the flight time for a packet given the payload size
///
/// @param packet_len		payload length in bytes
///
/// @return			flight time in 16usec ticks
static uint16_t flight_time_estimate(__pdata uint8_t packet_len)
{
	return packet_latency + (packet_len * ticks_per_byte);
}

/// update the TDM state machine
///
static void
tdm_state_update(__pdata uint16_t tdelta)
{
	// update the amount of time we are waiting for a preamble
	// to turn into a real packet
	if (tdelta > transmit_wait) {
		transmit_wait = 0;
	} else {
		transmit_wait -= tdelta;
	}

	// have we passed the next transition point?
	while (tdelta >= tdm_state_remaining) {
#ifdef WATCH_DOG_ENABLE
		// Tickle Watchdog
		PCA0CPH5 = 0;
#endif // WATCH_DOG_ENABLE
		if ((nodeTransmitSeq < 0x8000 || nodeId == BASE_NODEID) && (nodeTransmitSeq++ % nodeCount) == nodeId) {
			tdm_state = TDM_TRANSMIT;
			nodeTransmitSeq %= nodeCount;
		}
		// We need to -1 from nodeTransmitSeq as it was incremented above
		// Remember we have incremented nodeCount to allow for the sync period
		else if (nodeTransmitSeq < 0x8000 && (nodeTransmitSeq-1 % nodeCount) == nodeCount-1) {
			tdm_state = TDM_SYNC;
		}
		else {
			// Check for Bonus?
			tdm_state = TDM_RECEIVE; // If there are other nodes yet to transmit lets hear them first
		}
#ifdef DEBUG_PINS_SYNC
		if(tdm_state == TDM_SYNC) {
			P0 |= 0x02;
		}
		else {
			P0 &= ~0x02;
		}
#endif // DEBUG_PINS_SYNC
		
		// work out the time remaining in this state
		tdelta -= tdm_state_remaining;

		if (tdm_state == TDM_SYNC)
			tdm_state_remaining = tx_sync_width;
		else {
			tdm_state_remaining = tx_window_width;
			// change frequency when finishing transmitting or reciving
			fhop_window_change();
		}
		
		radio_receiver_on();

		if (num_fh_channels > 1) {
			// reset the LBT listen time
			lbt_listen_time = 0;
			lbt_rand = 0;
		}

		if (tdm_state == TDM_TRANSMIT && (duty_cycle - duty_cycle_offset) != 100) {
			// update duty cycle averages
			average_duty_cycle = (0.95*average_duty_cycle) + (0.05*(100.0*transmitted_ticks)/(2*(silence_period+tx_window_width)));
			transmitted_ticks = 0;
			duty_cycle_wait = (average_duty_cycle >= (duty_cycle - duty_cycle_offset));
		}

		// no longer waiting for a packet
		transmit_wait = 0;
	}

	// set right receive channel
	if (tdm_state == TDM_SYNC) {
		radio_set_channel(fhop_sync_channel());
	} else {
		radio_set_channel(fhop_receive_channel());
	}
	
	tdm_state_remaining -= tdelta;
}

#if USE_TICK_YIELD
/// update if another is yielding or has yielded (if we want to transmit)
///
static uint8_t
tdm_yield_update(__pdata uint8_t set_yield, __pdata uint8_t no_data)
{
	// Sort out the sync period first..
	if(tdm_state == TDM_SYNC && !set_yield) {
		if (nodeId == BASE_NODEID) {
			return YIELD_TRANSMIT;
		}
		else {
			lastTransmitWindow = nodeId | 0x8000;
			return YIELD_RECEIVE;
		}
	}
	
	if (tdm_state != TDM_TRANSMIT) {
		if(received_packet) {
			received_packet = false;
#ifdef DEBUG_PINS_YIELD
			P2 &= ~0x40;
#endif // DEBUG_PINS_YIELD
		}
		
		// REMEMBER nodeCount is set one higher than the user has set, this is to add sync to the sequence
		// nodeTransmitSeq points to the next slot so we also have to remove one from here
		if(set_yield == YIELD_GET) {
			if((nodeTransmitSeq != 0 && (lastTransmitWindow & 0x7FFF) == ((nodeTransmitSeq-1) % (nodeCount-1))) || 
			   (nodeTransmitSeq == 0 && (lastTransmitWindow & 0x7FFF) == (nodeCount-2)) ) {
				return YIELD_TRANSMIT;
			}
			else {
				return YIELD_RECEIVE;
			}
		}
		else if(no_data) {
			// Mark the first packet to send as a interrupt packet
			if((lastTransmitWindow & 0x7FFF) != trailer.nodeid){
				transmit_yield = true;
			}
			
			// Make sure all nodes so far have yielded to us..
			// Make sure the node waits for a random amount of time..
			if (lastTransmitWindow < 0x8000 && trailer.nodeid == ((lastTransmitWindow+1) % (nodeCount-1))) {
				lastTransmitWindow = trailer.nodeid;
				transmit_wait = packet_latency + ((uint16_t)rand())%(packet_latency*2);
			}
			// Ensure that other nodes get a chance to respond before we take their slot.
			else {
				lastTransmitWindow = trailer.nodeid | 0x8000;
				// This gives prefrance to a lower nodeId, need to think of a better way of yielding a slot
				transmit_wait = (packet_latency*(nodeId+1) + ((uint16_t)rand())%(packet_latency*(nodeId+2)));
			}
			// We can't have a delay more than 4* packet_lantency..
			transmit_wait %= (packet_latency*4);
		}
		// Change the Window so we don't send any data without politely asking first
		else {
			lastTransmitWindow = nodeId | 0x8000;
		}

		// Ensure it defaults to receive though this should only be reached when setting yield
		return YIELD_RECEIVE;
	}
	else if(tdm_state == TDM_TRANSMIT) { // We must be in Transmit Mode
		// If we have recived a packet during our Transmit window we have been yielded..
		if(received_packet) {
			lastTransmitWindow = 0x8000;
			return YIELD_RECEIVE;
		}
		
		if(yielded_slot) {
			// Change the Window so we don't send any data without politely asking first
			lastTransmitWindow = nodeId | 0x8000;
		}
		else {
			lastTransmitWindow = nodeId;
		}
			
		if (transmit_yield) {
			// reset the yield flag
			transmit_yield = false;
			
			// Lets wait to see if anyone else needs to transmit
			transmit_wait = packet_latency*6;

			return YIELD_RECEIVE;
		}
		return YIELD_TRANSMIT;
	}
	return YIELD_TRANSMIT;	
}
#endif // USE_TICK_YIELD

/// called to check temperature
///
static void temperature_update(void)
{
	register int16_t diff;
	if (radio_get_transmit_power() <= 20) {
		duty_cycle_offset = 0;
		return;
	}

	diff = radio_temperature() - MAX_PA_TEMPERATURE;
	if (diff <= 0 && duty_cycle_offset > 0) {
		// under temperature
		duty_cycle_offset -= 1;
	} else if (diff > 10) {
		// getting hot!
		duty_cycle_offset += 10;
	} else if (diff > 5) {
		// well over temperature
		duty_cycle_offset += 5;
	} else if (diff > 0) {
		// slightly over temperature
		duty_cycle_offset += 1;				
	}
	// limit to minimum of 20% duty cycle to ensure link stays up OK
	if ((duty_cycle-duty_cycle_offset) < 20) {
		duty_cycle_offset = duty_cycle - 20;
	}
}


/// blink the radio LED if we have not received any packets
///
static uint8_t unlock_count, temperature_count;
static void
link_update(void)
{
	if (nodeId == BASE_NODEID || received_sync) {
		unlock_count = 0;
		received_sync = false;
		fhop_set_locked(true);
#ifdef TDM_SYNC_LOGIC
		TDM_SYNC_PIN = true;
#endif // TDM_SYNC_LOGIC
	} else {
		unlock_count++;
	}
	
  // Unlock greater than second
	if (unlock_count < 2) {
		RADIO_LED(LED_ON);
	} else {
		sync_count = 0;
		RADIO_LED(blink_state);
		blink_state = !blink_state;
		nodeTransmitSeq = 0xFFFF;
		
		memset(remote_statistics, 0, sizeof(remote_statistics));
		memset(statistics, 0, sizeof(statistics));
		
		// reset statistics when unlocked
		statistics_receive_count = 0;
		
#ifdef TDM_SYNC_LOGIC
		TDM_SYNC_PIN = false;
#endif // TDM_SYNC_LOGIC
	}
	
  // Go into sync mode if we have dropped out
	if (unlock_count > 3) {
		if(sync_any) {
      if(unlock_count % 5 == 4) {
        fhop_window_change(); // Try our luck on another channel
      }
		}
		else {
			fhop_set_locked(false); // Set channel back to sync and try again
			radio_set_channel(fhop_sync_channel());
		}
    
#ifdef UNLOCK_RESET_SEC
    // Everything is broken :(
    if (unlock_count > UNLOCK_RESET_SEC) {
      // Reset the device using sofware reset
      RSTSRC |= 0x10;
    }
#endif // UNLOCK_RESET_SEC
	}

	statistics_transmit_stats = 0;

	// Half the display rate
	if(test_display_toggle) {
		test_display = at_testmode;
	}
	test_display_toggle = !test_display_toggle;
	
	temperature_count++;
	if (temperature_count == 4) {
		// check every 2 seconds
		temperature_update();
		temperature_count = 0;
	}
}

// dispatch an AT command to the remote system
void
tdm_remote_at(__pdata uint16_t destination)
{
	memcpy(remote_at_cmd, at_cmd, strlen(at_cmd)+1);
	send_at_command_to = destination;
	send_at_command = true;
}

// handle an incoming at command from the remote radio
static void
handle_at_command(__pdata uint8_t len)
{
	if (len < 2 || len > AT_CMD_MAXLEN || 
		pbuf[0] != (uint8_t)'R' || 
		pbuf[1] != (uint8_t)'T') {
		// assume its an AT command reply
		register uint8_t i;
		for (i=0; i<len; i++) {
			putchar(pbuf[i]);
		}
		return;
	}
	
	// Set the return address..
	send_at_command_to = trailer.nodeid;
	
	// setup the command in the at_cmd buffer
	memcpy(at_cmd, pbuf, len);
	at_cmd[len] = '\0';
	at_cmd[0] = 'A'; // replace 'R'
	at_cmd_len = len;

#ifdef WATCH_DOG_ENABLE
	// Pat the Watchdog
	PCA0CPH5 = 0;
#endif // WATCH_DOG_ENABLE
	
	// Capture ATI5 and proccess separatly
	if(len == 4 && at_cmd[2] == (uint8_t)'I' && at_cmd[3] == (uint8_t)'5'){
		ati5_id=0;
		packet_ati5_inject(ati5_id++);
	}
	else {
		// run the AT command, capturing any output to the packet buffer
		// this reply buffer will be sent at the next opportunity
		packet_at_inject();
	}
	
#ifdef WATCH_DOG_ENABLE
	// Pat the Watchdog
	PCA0CPH5 = 0;
#endif // WATCH_DOG_ENABLE
}

// a stack carary to detect a stack overflow
__at(0xFF) uint8_t __idata _canary;

/// main loop for time division multiplexing transparent serial
///
void
tdm_serial_loop(void)
{
	__pdata uint16_t last_t = timer2_tick();
	__pdata uint16_t last_link_update = last_t;

	_canary = 42;

	for (;;) {
		__pdata uint8_t	len;
		__pdata uint16_t tnow, tdelta;
		__pdata uint8_t max_xmit;

		if (_canary != 42) {
			panic("stack blown\n");
		}

		if (pdata_canary != 0x41) {
			panic("pdata canary changed\n");
		}
#ifdef WATCH_DOG_ENABLE
		// Throw the Watchdog a stick
		PCA0CPH5 = 0;
#endif // WATCH_DOG_ENABLE
		
		// give the AT command processor a chance to handle a command
		at_command();

		// display test data if needed
		if (test_display) {
			display_test_output();
			test_display = 0;
		}

		if (seen_mavlink && feature_mavlink_framing && !at_mode_active) {
			seen_mavlink = false;
			MAVLink_report();
		}

		// get the time before we check for a packet coming in
		tnow = timer2_tick();

		// see if we have received a packet
		if (radio_receive_packet(&len, pbuf)) {			
			// we're not waiting for a preamble
			// any more
			transmit_wait = 0;

			if (len < sizeof(trailer)) {
				// not a valid packet. We always send
				// trailer at the end of every packet
				
				printf("Invalid length.. %u\n",len);
				continue;
			}

#if USE_TICK_YIELD
			// If we have recived a packet there must be another node taking our slot..
			if(tdm_state == TDM_TRANSMIT){
				received_packet = true;
				lastTransmitWindow = 0x8000;
#ifdef DEBUG_PINS_YIELD
				P2 |= 0x40;
#endif // DEBUG_PINS_YIELD
			}
#endif // USE_TICK_YIELD
			
			// extract control bytes from end of packet
			memcpy(&trailer, pbuf +len-sizeof(trailer), sizeof(trailer));
			len -= sizeof(trailer);

			// Sync the timing sequence with the incoming packet
			// trailer.nodeid in a sync byte is the next channel to receive/transmit on
			if(trailer.nodeid & 0x8000){
				if(sync_count < 0xFF && nodeTransmitSeq == 0){
					sync_count += 1;
				}
				nodeTransmitSeq = 0;
				set_transmit_channel(trailer.nodeid & 0x7FFF);
				received_sync = true;
				continue;
			}
			// We dont want to sync off nodes sending bonus data
			else if (sync_any && !trailer.bonus) {
				if(sync_count < 0xFF && nodeTransmitSeq == trailer.nodeid + 1){
					sync_count += 1;
				}
				nodeTransmitSeq = trailer.nodeid + 1;
				received_sync = true;
			}
			
			// update filtered RSSI value and packet stats
			if(trailer.nodeid < MAX_NODE_RSSI_STATS) {
				statistics[trailer.nodeid].average_rssi = (radio_last_rssi() + 7*(uint16_t)statistics[trailer.nodeid].average_rssi)/8;
			}
			statistics_receive_count++;
			
			if (trailer.window == 0 && len != 0) {
				// its a control packet
				if (len == (sizeof(struct statistics)+sizeof(statistics_transmit_stats)) && trailer.nodeid < MAX_NODE_RSSI_STATS) {
					len -= sizeof(statistics_transmit_stats);
					// Get the last two bytes from the packet and compare them against our nodeId
					if(((uint16_t*)(pbuf+len))[0] == nodeId)
					{
						memcpy(remote_statistics +trailer.nodeid, pbuf, len);
					}
				}

				// don't count control packets in the stats
				statistics_receive_count--;
			} else if (trailer.window != 0) {
				tdm_state_remaining = trailer.window;
				
#if USE_TICK_YIELD
				// if the other end has sent a zero length packet and we are
				// in their transmit window then they are yielding some ticks to us.
				tdm_yield_update(YIELD_SET, len==0);
#endif // USE_TICK_YIELD
				last_t = tnow;

				if (trailer.command == 1) {
					// Skip Interupt packets (sent at the start of talking control of someone elses slot)
					if(len > 1)
					{
						handle_at_command(len);
					}
				} else if (len != 0 && 
					   !packet_is_duplicate(len, pbuf, trailer.resend) &&
					   !at_mode_active) {
					// its user data - send it out the serial port
					ACTIVITY_LED(LED_ON);
					serial_write_buf(pbuf, len);
					ACTIVITY_LED(LED_OFF);
				}
			}
			continue;
		}
		
		// see how many 16usec ticks have passed and update
		// the tdm state machine. We re-fetch tnow as a bad
		// packet could have cost us a lot of time.
		tnow = timer2_tick();
		tdelta = tnow - last_t;
		tdm_state_update(tdelta);
		last_t = tnow;

		// wait for the silence period to expire, to allow radio's to switch channel
		if( (tdm_state_remaining > tx_window_width-silence_period) ||
		    (tdm_state == TDM_SYNC && tdm_state_remaining > tx_sync_width-silence_period))
		{
			continue;
		}
		
		// update link status approximately every 0.5s
		if (tnow - last_link_update > 32768) {
			link_update();
			last_link_update = tnow;
		}

		if (lbt_rssi != 0) {
			// implement listen before talk
			if (radio_current_rssi() < lbt_rssi) {
				lbt_listen_time += tdelta;
			} else {
				lbt_listen_time = 0;
				if (lbt_rand == 0) {
					lbt_rand = ((uint16_t)rand()) % lbt_min_time;
				}
			}
			if (lbt_listen_time < lbt_min_time + lbt_rand) {
				// we need to listen some more
				continue;
			}
		}

		// we are allowed to transmit in our transmit window
		// or in the other radios transmit window if we have
		// bonus ticks
#if USE_TICK_YIELD
		if (tdm_yield_update(YIELD_GET, YIELD_NO_DATA) == YIELD_RECEIVE) {
#ifdef DEBUG_PINS_TRANSMIT_RECEIVE
			P2 &= ~0x04;
#endif // DEBUG_PINS_TRANSMIT_RECEIVE
			continue;
		}
#ifdef DEBUG_PINS_TRANSMIT_RECEIVE
		P2 |= 0x04;
#endif // DEBUG_PINS_TRANSMIT_RECEIVE
#ifdef DEBUG_PINS_TX_RX_STATE
		P2 &= ~0x08;
#endif // DEBUG_PINS_TX_RX_STATE
#else // USE_TICK_YIELD
		// If we arn't in transmit or our node id isn't BASE_NODEID and in tdm_sync
		if (tdm_state != TDM_TRANSMIT) {
			if(tdm_state != TDM_SYNC || nodeId != BASE_NODEID) {
				continue;
			}
		}		
#endif // USE_TICK_YIELD

		if (transmit_wait != 0) {
			// we're waiting for a preamble to turn into a packet
			continue;
		}

		if (radio_preamble_detected() ||
		    radio_receive_in_progress()) {
			// a preamble has been detected. Don't
			// transmit for a while
			transmit_wait = packet_latency;
			
#if USE_TICK_YIELD
			// If we detect a incoming packet during our transmit period
			// lets be quiet for the remainder of our period.
			// Sometimes we may not recive a packet as it has been filtered by the radio
			if(tdm_state == TDM_TRANSMIT){
				received_packet = true;
				lastTransmitWindow = 0x8000;
#ifdef DEBUG_PINS_YIELD
				P2 |= 0x40;
#endif // DEBUG_PINS_YIELD
			}
#endif // USE_TICK_YIELD
			
			continue;
		}
		
#ifdef WATCH_DOG_ENABLE
		// Pat the Watchdog
		PCA0CPH5 = 0;
#endif // WATCH_DOG_ENABLE
		
		// Dont send anything until we have received 20 good sync bytes
		if (nodeId != BASE_NODEID && sync_count < 20) {
			continue;
		}

		// sample the background noise when it is out turn to
		// transmit, but we are not transmitting,
		// averaged over around 4 samples
		statistics[nodeId].average_noise = (radio_current_rssi() + 3*(uint16_t)statistics[nodeId].average_noise)/4;

		if (duty_cycle_wait) {
			// we're waiting for our duty cycle to drop
			continue;
		}

		// how many bytes could we transmit in the time we
		// have left?
		if (tdm_state_remaining < packet_latency) {
			// none ....
			continue;
		}
		
		// leave 1 packet_latency at the end of the transmit and another at the end of the sequence
		if((signed) tdm_state_remaining - 2*(signed)packet_latency < 0) {
			max_xmit = 0;
		}
		else {
			max_xmit = (tdm_state_remaining - 2*packet_latency) / ticks_per_byte;
		}
		if (max_xmit < sizeof(trailer)+1) {
			// can't fit the trailer in with a byte to spare
			
			continue;
		}
		max_xmit -= sizeof(trailer)+1;
		if (max_xmit > max_data_packet_length) {
			max_xmit = max_data_packet_length;
		}

#if USE_TICK_YIELD
		// Check to see if we need to send a dummy packet to inform everyone in the network we want to send data.
		// This is done when we are yielding only
		if(serial_read_available() > 0 && transmit_yield && tdm_state == TDM_RECEIVE)
		{
			// if more than 1/4 of the slot is passed it wouldn't be worth transmitting in this slot
			if(tdm_state_remaining < tx_window_width/4) {
				continue;
			}
			
			pbuf[0] = 0xff;
			len = 1;
			trailer.command = 1;
			// Broadcast Interupt flag
			nodeDestination = 0xFFFF;
		}
		else
#endif // USE_TICK_YIELD
		// ask the packet system for the next packet to send
		// no data is to be sent during a sync period
		if (tdm_state != TDM_SYNC) {
			if (send_at_command && max_xmit >= strlen(remote_at_cmd)) {
				// send a remote AT command
				len = strlen(remote_at_cmd);
				memcpy(pbuf, remote_at_cmd, len);
				trailer.command = 1;
				nodeDestination = send_at_command_to;
				send_at_command = false;
			} else {
				// get a packet from the serial port
				len = packet_get_next(max_xmit, pbuf);
				trailer.command = packet_is_injected();
				
				// If it's a AT return packet, set the return address
				if(trailer.command) {
					nodeDestination = send_at_command_to;
					packet_ati5_inject(ati5_id++);
				}
			}
		}
		else {
			len = 0;
		}

		if (len > max_data_packet_length) {
			panic("oversized tdm packet");
		}

		trailer.bonus = (tdm_state == TDM_RECEIVE);
		trailer.resend = packet_is_resend();
			
		// Are we in transmit phase and have space for a stats packet
		if (tdm_state == TDM_TRANSMIT && len == 0 && max_xmit >= (sizeof(statistics)+sizeof(statistics_transmit_stats))
		// Do we need to send a stats packet
			&& statistics_transmit_stats < (nodeCount-1) && nodeId < MAX_NODE_RSSI_STATS 
		// Yeild at the start of our time period to allow better data throughput
			&& tdm_state_remaining < (tx_window_width-packet_latency*2)) {
			
			// Catch for Node 0
			if(statistics_transmit_stats == nodeId) {
				statistics_transmit_stats++;
			}
			
			len = sizeof(struct statistics);
			statistics[statistics_transmit_stats].average_noise = statistics[nodeId].average_noise;
			memcpy(pbuf, statistics+statistics_transmit_stats, len);
			memcpy(pbuf+len, &statistics_transmit_stats, sizeof(statistics_transmit_stats));
			len += sizeof(statistics_transmit_stats);
			
			statistics_transmit_stats++;
			
			// Catch for last node
			if(statistics_transmit_stats == nodeId) {
				statistics_transmit_stats++;
			}

			// mark a stats packet with a zero window
			trailer.window = 0;
			trailer.resend = 0;
			trailer.command = 0;
		} 
		else if (tdm_state != TDM_TRANSMIT && len == 0 && !(tdm_state == TDM_SYNC && nodeId == BASE_NODEID)) {
			continue; // If we have nothing contructive to send be quiet..
		}
		else {
			// calculate the control word as the number of
			// 16usec ticks that will be left in this
			// tdm state after this packet is transmitted
			trailer.window = (uint16_t)(tdm_state_remaining - flight_time_estimate(len+sizeof(trailer)));
		}

		// if in sync mode and we are the base, add the channel and sync bit
		if (tdm_state == TDM_SYNC && nodeId == BASE_NODEID) {
			trailer.nodeid = get_transmit_channel() | 0x8000;
		} else {
			trailer.nodeid = nodeId;
		}

		memcpy(pbuf+len, &trailer, sizeof(trailer));

		// If the command byte is set the nodeDestination has already been set
		if(!trailer.command)
		{
			if (len != 0 && trailer.window != 0) {
				// show the user that we're sending real data
				ACTIVITY_LED(LED_ON);
				nodeDestination = paramNodeDestination;
			}
			else { // Default to broadcast
				nodeDestination = 0xFFFF; 
			}
		}

#if USE_TICK_YIELD
		if(tdm_state == TDM_TRANSMIT) {
			if (len == 0) {
				// sending a zero byte packet gives up
				// our window, but doesn't change the
				// start of the next window
				transmit_yield = true;
				yielded_slot = true;
			}
			else {
				yielded_slot = false;
			}
		}
		// If we are here we must have permission to send data
		else if (tdm_state == TDM_RECEIVE) {
			lastTransmitWindow &= 0x7FFF;
		}
#endif // USE_TICK_YIELD
		
		// after sending a packet leave a bit of time before
		// sending the next one. The receivers don't cope well
		// with back to back packets
#if USE_TICK_YIELD
		if (transmit_yield && tdm_state == TDM_RECEIVE) {
			transmit_yield = false;
			transmit_wait = 2*packet_latency;
#ifdef DEBUG_PINS_TX_RX_STATE
			P2 |= 0x08;
#endif // DEBUG_PINS_TX_RX_STATE
		}
		else
		{
			transmit_wait = packet_latency;
		}
#else
		transmit_wait = packet_latency;
#endif // USE_TICK_YIELD

		// if we're implementing a duty cycle, add the
		// transmit time to the number of ticks we've been transmitting
		if ((duty_cycle - duty_cycle_offset) != 100) {
			transmitted_ticks += flight_time_estimate(len+sizeof(trailer));
		}

#ifdef WATCH_DOG_ENABLE
		// Feed Watchdog
		PCA0CPH5 = 0;
#endif // WATCH_DOG_ENABLE
		
		// start transmitting the packet
		if (!radio_transmit(len + sizeof(trailer), pbuf, nodeDestination, tdm_state_remaining) && len != 0) {
			packet_force_resend();
		}
		
		if (lbt_rssi != 0) {
			// reset the LBT listen time
			lbt_listen_time = 0;
			lbt_rand = 0;
		}

		// set right receive channel
		radio_set_channel(fhop_receive_channel());

		// re-enable the receiver
		radio_receiver_on();

		if (len != 0 && trailer.window != 0) {
			ACTIVITY_LED(LED_OFF);
		}
	}
}

bool
tdm_state_sync()
{
	return received_sync;
}

// setup a 16 bit node count
//
void
tdm_set_node_count(__pdata uint16_t count)
{
	nodeCount = count + 1; // add 1 for the sync channel
}

// setup a 16 bit node destination
//
void
tdm_set_node_destination(__pdata uint16_t destination)
{
	paramNodeDestination = destination;
}

void
tdm_set_sync_any(__pdata uint8_t any)
{
	sync_any = any;
}

#if 0
/// build the timing table
static void 
tdm_build_timing_table(void)
{
	__pdata uint8_t j;
	__pdata uint16_t rate;
	bool golay_saved = feature_golay;
	feature_golay = false;
	nodeDestination = 0xffff;

	for (rate=2; rate<256; rate=(rate*3)/2) {
		__pdata uint32_t latency_sum=0, per_byte_sum=0;
		uint8_t size = MAX_PACKET_LENGTH;
		radio_configure(rate);
		for (j=0; j<50; j++) {
			__pdata uint16_t time_0, time_max, t1, t2;
			radio_set_channel(1);
			radio_receiver_on();
			if (serial_read_available() > 0) {
				feature_golay = golay_saved;
				return;
			}
			t1 = timer2_tick();
			if (!radio_transmit(0, pbuf, nodeDestination, 0xFFFF)) {
				break;
			}
			t2 = timer2_tick();
			radio_receiver_on();

			time_0 = t2-t1;

			radio_set_channel(2);
			t1 = timer2_tick();
			if (!radio_transmit(size, pbuf, nodeDestination, 0xFFFF)) {
				if (size == 0) {
					break;
				}
				size /= 4;
				j--;
				continue;
			}

			t2 = timer2_tick();
			radio_receiver_on();

			time_max = t2-t1;
			latency_sum += time_0;
			per_byte_sum += ((size/2) + (time_max - time_0))/size;
		}
		if (j > 0) {
			printf("{ %u, %u, %u },\n",
			       (unsigned)(radio_air_rate()),
			       (unsigned)(latency_sum/j),
			       (unsigned)(per_byte_sum/j));
		}
	}
	feature_golay = golay_saved;
}


// test hardware CRC code
static void 
crc_test(void)
{
	__xdata uint8_t d[4] = { 0x01, 0x00, 0xbb, 0xcc };
	__pdata uint16_t crc;
	uint16_t t1, t2;
	crc = crc16(4, &d[0]);
	printf("CRC: %x %x\n", crc, 0xb166);	
	t1 = timer2_tick();
	crc16(MAX_PACKET_LENGTH/2, pbuf);
	t2 = timer2_tick();
	printf("crc %u bytes took %u 16usec ticks\n",
	       (unsigned)MAX_PACKET_LENGTH/2,
	       t2-t1);
}

// test golay encoding
static void 
golay_test(void)
{
	uint8_t i;
	uint16_t t1, t2;
	__xdata uint8_t	buf[MAX_PACKET_LENGTH];
	for (i=0; i<MAX_PACKET_LENGTH/2; i++) {
		pbuf[i] = i;
	}
	t1 = timer2_tick();
	golay_encode(MAX_PACKET_LENGTH/2, pbuf, buf);
	t2 = timer2_tick();
	printf("encode %u bytes took %u 16usec ticks\n",
	       (unsigned)MAX_PACKET_LENGTH/2,
	       t2-t1);
	// add an error in the middle
	buf[MAX_PACKET_LENGTH/2] ^= 0x23;
	buf[1] ^= 0x70;
	t1 = timer2_tick();
	golay_decode(MAX_PACKET_LENGTH, buf, pbuf);
	t2 = timer2_tick();
	printf("decode %u bytes took %u 16usec ticks\n",
	       (unsigned)MAX_PACKET_LENGTH,
	       t2-t1);
	for (i=0; i<MAX_PACKET_LENGTH/2; i++) {
		if (pbuf[i] != i) {
			printf("golay error at %u\n", (unsigned)i);
		}
	}
}
#endif


// initialise the TDM subsystem
void
tdm_init(void)
{
	__pdata uint16_t i;
	__pdata uint8_t air_rate = radio_air_rate();
	__pdata uint32_t window_width;

#define REGULATORY_MAX_WINDOW (((1000000UL/16)*4)/10)
#define LBT_MIN_TIME_USEC 5000

	// tdm_build_timing_table();

	// calculate how many 16usec ticks it takes to send each byte
	ticks_per_byte = (8+(8000000UL/(air_rate*1000UL)))/16;

	// Check for rounding errors, and round up if needed..
	if(10000UL*ticks_per_byte < (8+(8000000UL/(air_rate*1000UL)))*625) {
		ticks_per_byte += 1;
	}
	
	// calculate the minimum packet latency in 16 usec units
	// we initially assume a preamble length of 40 bits, then
	// adjust later based on actual preamble length. This is done
	// so that if one radio has antenna diversity and the other
	// doesn't, then they will both using the same TDM round timings
	packet_latency = (8+(10/2)) * ticks_per_byte + 13;

	if (feature_golay) {
		max_data_packet_length = (MAX_PACKET_LENGTH/2) - (6+sizeof(trailer));

		// golay encoding doubles the cost per byte
		ticks_per_byte *= 2;

		// and adds 4 bytes
		packet_latency += 4*ticks_per_byte;
	} else {
		max_data_packet_length = MAX_PACKET_LENGTH - sizeof(trailer);
	}

	// set the silence period to between changing channels
	silence_period = 2*packet_latency;

	// set the transmit window to allow for 2 full sized packets
	window_width = 2*((max_data_packet_length*(uint32_t)ticks_per_byte)+packet_latency) + silence_period + packet_latency;

	// if LBT is enabled, we need at least 3*5ms of window width
	if (lbt_rssi != 0) {
		// min listen time is 5ms
		lbt_min_time = LBT_MIN_TIME_USEC/16;
		window_width = constrain(window_width, 3*lbt_min_time, window_width);
	}

	// make sure it fits in the 13 bits of the trailer window
	if (window_width > 0x1FFF) {
		window_width = 0x1FFF;
	}
	
	// the window width cannot be more than 0.4 seconds to meet US regulations
	if (window_width >= REGULATORY_MAX_WINDOW) {
		window_width = REGULATORY_MAX_WINDOW;
	}
	
	tx_window_width = window_width;
	
	// Window size of 4 statistic packets
	window_width = 4*(((sizeof(trailer))*(uint32_t)ticks_per_byte)+packet_latency) + silence_period + packet_latency;
	tx_sync_width = window_width;
	
	// now adjust the packet_latency for the actual preamble
	// length, so we get the right flight time estimates, while
	// not changing the round timings
	packet_latency += ((settings.preamble_length-10)/2) * ticks_per_byte;

	// tell the packet subsystem our max packet size, which it
	// needs to know for MAVLink packet boundary detection
	i = (tx_window_width - packet_latency) / ticks_per_byte;
	if (i > max_data_packet_length) {
		i = max_data_packet_length;
	}
	packet_set_max_xmit(i);

	// Clear Values..
	trailer.nodeid  = 0xFFFF;
	nodeTransmitSeq = 0xFFFF;
	memset(remote_statistics, 0, sizeof(remote_statistics));
	memset(statistics, 0, sizeof(statistics));
	
	// crc_test();

	// tdm_test_timing();
	
	// golay_test();

	ati5_id = PARAM_MAX;
  unlock_count = 6;
  RADIO_LED(LED_OFF);
  
#if USE_TICK_YIELD
	received_packet = false;
#ifdef DEBUG_PINS_YIELD
	P2 &= ~0x40;
#endif // DEBUG_PINS_YIELD
#endif // USE_TICK_YIELD
	
#ifdef TDM_SYNC_LOGIC
	TDM_SYNC_PIN = false;
#endif // TDM_SYNC_LOGIC
}


/// report tdm timings
///
void 
tdm_report_timing(void)
{
	printf("[%u] silence_period: %u\n", nodeId, (unsigned)silence_period); delay_msec(1);
	printf("[%u] tx_window_width: %u\n", nodeId, (unsigned)tx_window_width); delay_msec(1);
	printf("[%u] max_data_packet_length: %u\n", nodeId, (unsigned)max_data_packet_length); delay_msec(1);
}

