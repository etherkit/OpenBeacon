/*
 * main.c
 *
 *  Created on: Jan 8, 2012
 *      Author: Jason Milldrum
 *     Company: Etherkit
 *
 *     Copyright (c) 2012, Jason Milldrum
 *     All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without modification,
 *  are permitted provided that the following conditions are met:
 *
 *  - Redistributions of source code must retain the above copyright notice, this list
 *  of conditions and the following disclaimer.
 *
 *  - Redistributions in binary form must reproduce the above copyright notice, this list
 *  of conditions and the following disclaimer in the documentation and/or other
 *  materials provided with the distribution.
 *
 *  - Neither the name of Etherkit nor the names of its contributors may be
 *  used to endorse or promote products derived from this software without specific
 *  prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
 *  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 *  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 *  SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 *  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 *  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <avr/io.h>
#include <avr/sfr_defs.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <avr/wdt.h>
#include <avr/pgmspace.h>   /* required by usbdrv.h */

#include "usbdrv/usbdrv.h"

#include "requests.h"
#include "usbconfig.h"
#include "morsechar.h"
#include "font.h"
#include "modes.h"

// Hardware Defines
#define FSK_DDR					DDRB
#define FSK_PORT				PORTB
#define FSK						PB4

#define KEY_DDR					DDRB
#define KEY_PORT				PORTB
#define KEY						PB3

#define S1_DDR					DDRB
#define S1_PORT					PORTB
#define S1_PIN					PINB
#define S1						PB1

// Firmware constant defines
#define DFCW_DEFAULT_OFFSET		100

#define DEBOUNCE_PRESS_TIME		3			// Amount of captures for press keybounce (in 2 ms increments)
#define DEBOUNCE_HOLD_TIME		250			// Amount of captures for hold keybounce (in 2 ms increments)

#define MULT_DAH				3			// DAH is 3x a DIT
#define MULT_WORDDELAY			7			// Space between words is 7 dits
#define MULT_HELL_WORDDELAY		200
#define MULT_HELL_CHARDELAY		20
#define MULT_HELL_GLYPHDELAY	60
#define HELL_ROW_RPT			3

#define DEFAULT_MODE			0
#define DEFAULT_WPM				12000
#define DEFAULT_MSG_DELAY		0			// in minutes

#define WSPR_SYMBOL_LENGTH		342			// 684 ms symbol TX length / 2 ms tick

#define CWID_DELAY				10			// CW ID interval in minutes
#define CWID_WPM				20000		// CW ID at 20 WPM

// Enumerations
enum BOOL {FALSE, TRUE};
enum BUFFER {BUFFER_1 = 1, BUFFER_2};
enum USB_WRITE {USB_BUFFER_1, USB_BUFFER_2, USB_GLYPH, USB_WSPR_BUFFER};
enum STATE {STATE_IDLE, STATE_DIT, STATE_DAH, STATE_DITDELAY, STATE_DAHDELAY, STATE_WORDDELAY, STATE_MSGDELAY,
			STATE_CHARDELAY, STATE_HELLCOL, STATE_HELLROW, STATE_CAL, STATE_WSPR, STATE_WSPR_INIT, STATE_PREAMBLE, STATE_HELLIDLE};

// Global variables
uint32_t cur_timer = 0;
uint32_t dit_length;
enum MODE cur_mode, prev_mode;
enum STATE cur_state, prev_state;
uint32_t cur_state_end, msg_delay_end, prev_state_end;
static char msg_buffer[WSPR_BUFFER_SIZE];
static char temp_buffer[WSPR_BUFFER_SIZE];
char * cur_msg_p;
char * prev_msg_p;
char * cur_glyph_p;
char cur_character = '\0';
char prev_character = '\0';
char cur_hell_char = '\0';
uint8_t cur_hell_col = 0;
uint8_t cur_hell_row = 0;
uint16_t wpm, prev_wpm;
uint8_t msg_delay;
enum BUFFER cur_buffer, prev_buffer;
enum USB_WRITE write_to;
uint8_t dfcw_offset;
enum BOOL cwid = FALSE;
uint32_t next_cwid;

static uchar currentPosition = 0;
static uchar bytesRemaining = 0;

// Global variables used in ISRs
volatile uint32_t timer; // A 32-bit timer will count for 2^32 * 2 ms = ~16 years
volatile enum BOOL tx_on;
volatile uint8_t fsk_tune;
volatile enum BOOL S1_active;

// EEPROM variables
uint8_t EEMEM ee_osc;
uint8_t	EEMEM ee_mode = DEFAULT_MODE;
uint16_t EEMEM ee_wpm = DEFAULT_WPM;
uint8_t EEMEM ee_msg_delay = DEFAULT_MSG_DELAY;
uint8_t EEMEM ee_dfcw_offset = DFCW_DEFAULT_OFFSET;
uchar EEMEM ee_msg_mem_1[MSG_BUFFER_SIZE] = "N0CALL";
uchar EEMEM ee_msg_mem_2[MSG_BUFFER_SIZE] = "MSG2";
uchar EEMEM ee_wspr_symbols[WSPR_BUFFER_SIZE] = "";
uchar EEMEM ee_buffer = BUFFER_1;
uchar EEMEM ee_glyph_1[GLYPH_SIZE] = {0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x80};
uchar EEMEM ee_glyph_2[GLYPH_SIZE] = {0x1C, 0x3E, 0x7F, 0x7F, 0x7F, 0x3E, 0x1C, 0x80};
uchar EEMEM ee_glyph_3[GLYPH_SIZE] = "";
uchar EEMEM ee_glyph_4[GLYPH_SIZE] = "";
//enum BOOL EEMEM ee_cwid = TRUE;

// Global constants
const uint8_t hell_tune[HELL_ROWS] PROGMEM = {252, 185, 140, 85, 53, 23, 0};

// Function prototypes
void set_wpm(uint32_t);
uint32_t get_msg_delay(uint8_t);
void init_tx(void);
void debounce(void);
void reset_buffer(void);
void init_cwid(void);

// Interrupt service routine
ISR(TIM0_COMPA_vect)
{
	// Tick the clock
	timer++;

	debounce();
}

usbMsgLen_t usbFunctionSetup(uchar data[8])
{
	usbRequest_t *rq = (void *)data;
	usbMsgLen_t len;
	static uchar dataBuffer[4];  /* buffer must stay valid when usbFunctionSetup returns */

	switch(rq->bRequest)
	{
	case CUSTOM_RQ_ECHO:
		/* echo -- used for reliability tests */
        //usbMsgPtr = dataBuffer;         /* tell the driver which data to return */
        return 0;
        break;

	case CUSTOM_RQ_SET_MODE:
       	// Set mode
        cur_mode = rq->wValue.bytes[0];
        eeprom_write_byte(&ee_mode, cur_mode);

        init_tx();

        /*
        // Reset message buffer
        if(cur_mode == MODE_WSPR)
        {
        	//wspr_buffer
        	eeprom_read_block((void*)&msg_buffer, (const void*)&ee_wspr_symbols, WSPR_BUFFER_SIZE - 1);
			cur_msg_p = msg_buffer;
			cur_character = '\0';
        }
        else
        {
			eeprom_read_block((void*)&msg_buffer, (const void*)&ee_msg_mem_1, MSG_BUFFER_SIZE - 1);
			cur_msg_p = msg_buffer;
			cur_character = '\0';
        }

        // If in message delay mode, set the delay
        msg_delay_end = cur_timer + get_msg_delay(msg_delay);

		// Reset Hell index
        cur_hell_row = 0;
        cur_hell_col = 0;

        // Reset WPM
        if(cur_mode == MODE_CW)
        	wpm = eeprom_read_word(&ee_wpm);
        else
        	wpm = pgm_read_word(&dit_speed[cur_mode]);

        set_wpm(wpm);

        // Put back in IDLE state
        cur_state_end = cur_timer;
        cur_state = STATE_IDLE;*/

        return 0;
        break;

	case CUSTOM_RQ_GET_MODE:
        dataBuffer[0] = cur_mode;
        usbMsgPtr = dataBuffer;         /* tell the driver which data to return */
        return 1;                       /* tell the driver to send 1 byte */
        break;

	case CUSTOM_RQ_SET_MSG_1:
        currentPosition = 0;					// initialize position index
        bytesRemaining = rq->wLength.word;		// store the amount of data requested
        if(bytesRemaining > sizeof(temp_buffer))	// limit to buffer size
            bytesRemaining = sizeof(temp_buffer);
        write_to = USB_BUFFER_1;
        return USB_NO_MSG;						// tell driver to use usbFunctionWrite()
    	break;

	case CUSTOM_RQ_GET_MSG_1:
    	len = MSG_BUFFER_SIZE;					// we return up to 64 bytes
		if(len > rq->wLength.word)				// if the host requests less than we have
			len = rq->wLength.word;				// return only the amount requested
		eeprom_read_block((void*)&temp_buffer, (const void*)&ee_msg_mem_1, MSG_BUFFER_SIZE - 1);
		usbMsgPtr = (uchar *)temp_buffer;		// tell driver where the buffer starts
		return len;                         	// tell driver how many bytes to send
    	break;

	case CUSTOM_RQ_SET_MSG_2:
        currentPosition = 0;					// initialize position index
        bytesRemaining = rq->wLength.word;		// store the amount of data requested
        if(bytesRemaining > sizeof(temp_buffer))	// limit to buffer size
            bytesRemaining = sizeof(temp_buffer);
        write_to = USB_BUFFER_2;
        return USB_NO_MSG;						// tell driver to use usbFunctionWrite()
    	break;

	case CUSTOM_RQ_GET_MSG_2:
    	len = MSG_BUFFER_SIZE;					// we return up to 64 bytes
		if(len > rq->wLength.word)				// if the host requests less than we have
			len = rq->wLength.word;				// return only the amount requested
		eeprom_read_block((void*)&temp_buffer, (const void*)&ee_msg_mem_2, MSG_BUFFER_SIZE - 1);
		usbMsgPtr = (uchar *)temp_buffer;		// tell driver where the buffer starts
		return len;                         	// tell driver how many bytes to send
    	break;

	case CUSTOM_RQ_SET_WPM:
    	wpm = rq->wValue.bytes[0] * 1000;
    	set_wpm(wpm);
    	eeprom_write_word(&ee_wpm, wpm);
    	return 0;
    	break;

	case CUSTOM_RQ_GET_WPM:
        dataBuffer[0] = (uchar)(wpm / 1000);
        usbMsgPtr = dataBuffer;         		// tell the driver which data to return
        return 1;								// tell the driver to send 1 byte
    	break;

	case CUSTOM_RQ_SET_MSG_DLY:
    	eeprom_write_byte(&ee_msg_delay, rq->wValue.bytes[0]);
		msg_delay = rq->wValue.bytes[0];

		msg_delay_end = cur_timer + get_msg_delay(msg_delay);
		return 0;
		break;

	case CUSTOM_RQ_GET_MSG_DLY:
        dataBuffer[0] = (uchar)msg_delay;
        usbMsgPtr = dataBuffer;         // tell the driver which data to return
        return 1;                       // tell the driver to send 1 byte
    	break;

	case CUSTOM_RQ_SET_CUR_BUFFER:
		if(rq->wValue.bytes[0] == 1)
		{
			cur_buffer = BUFFER_1;
			eeprom_write_byte(&ee_buffer, cur_buffer);
		}
		else if(rq->wValue.bytes[0] == 2)
		{
			cur_buffer = BUFFER_2;
			eeprom_write_byte(&ee_buffer, cur_buffer);
		}
		return 0;
		break;

	case CUSTOM_RQ_GET_CUR_BUFFER:
		dataBuffer[0] = (uchar)cur_buffer;
		usbMsgPtr = dataBuffer;         // tell the driver which data to return
		return 1;                       // tell the driver to send 1 byte
		break;

	case CUSTOM_RQ_SET_DFCW_OFFSET:
    	dfcw_offset = rq->wValue.bytes[0];
    	eeprom_write_byte(&ee_dfcw_offset, dfcw_offset);
    	return 0;
    	break;

	case CUSTOM_RQ_GET_DFCW_OFFSET:
        dataBuffer[0] = (uchar)(dfcw_offset);
        usbMsgPtr = dataBuffer;         		// tell the driver which data to return
        return 1;								// tell the driver to send 1 byte
    	break;

	case CUSTOM_RQ_SET_GLYPH:
		currentPosition = 0;					// initialize position index
		bytesRemaining = rq->wLength.word;		// store the amount of data requested
		if(bytesRemaining > sizeof(temp_buffer))	// limit to buffer size
			bytesRemaining = sizeof(temp_buffer);
		write_to = USB_GLYPH;
		return USB_NO_MSG;						// tell driver to use usbFunctionWrite()
		break;

	case CUSTOM_RQ_SET_WSPR:
		currentPosition = 0;					// initialize position index
		bytesRemaining = rq->wLength.word;		// store the amount of data requested
		if(bytesRemaining > sizeof(temp_buffer))	// limit to buffer size
			bytesRemaining = sizeof(temp_buffer);
		write_to = USB_WSPR_BUFFER;
		return USB_NO_MSG;						// tell driver to use usbFunctionWrite()
		break;

	case CUSTOM_RQ_START_TX:
		init_tx();
		return 0;
		break;

	default:
		break;
	}
    return 0;   /* default for not implemented requests: return no data back to host */
}

uchar usbFunctionWrite(uchar *data, uchar len)
{
	uchar i;
	uint8_t glyph_number;
	char *glyph_ptr = NULL;

    if(len > bytesRemaining)                // if this is the last incomplete chunk
        len = bytesRemaining;               // limit to the amount we can store
    bytesRemaining -= len;
    for(i = 0; i < len; i++)
        temp_buffer[currentPosition++] = data[i];

    if(bytesRemaining == 0)
    {
    	switch(write_to)
    	{
    	case USB_BUFFER_1:
    		eeprom_write_block((const void*)&temp_buffer, (void*)&ee_msg_mem_1, MSG_BUFFER_SIZE - 1);
    		break;

    	case USB_BUFFER_2:
    		eeprom_write_block((const void*)&temp_buffer, (void*)&ee_msg_mem_2, MSG_BUFFER_SIZE - 1);
    		break;

    	case USB_GLYPH:
    		// Get the glyph number from the first char, then point to beginning of glyph string
    		strncpy(glyph_ptr, temp_buffer, 1);
    		glyph_number = atoi(glyph_ptr);
    		glyph_ptr = temp_buffer;
    		glyph_ptr++;

    		switch(glyph_number)
    		{
    		case 1:
    			eeprom_write_block((const void*)glyph_ptr, (void*)&ee_glyph_1, GLYPH_SIZE - 1);
    			break;
    		case 2:
				eeprom_write_block((const void*)glyph_ptr, (void*)&ee_glyph_2, GLYPH_SIZE - 1);
				break;
    		}
    		break;

    	case USB_WSPR_BUFFER:
    		eeprom_write_block((const void*)&temp_buffer, (void*)&ee_wspr_symbols, WSPR_BUFFER_SIZE - 1);
    		break;

    	default:
    		break;
    	}

		// Put back in IDLE state
		cur_state = STATE_IDLE;
    }

    return bytesRemaining == 0;             // return 1 if we have all data
}

static void calibrateOscillator(void)
{
uchar       step = 128;
uchar       trialValue = 0, optimumValue;
int         x, optimumDev, targetValue = (unsigned)(1499 * (double)F_CPU / 10.5e6 + 0.5);

    /* do a binary search: */
    do{
        OSCCAL = trialValue + step;
        x = usbMeasureFrameLength();    // proportional to current real frequency
        if(x < targetValue)             // frequency still too low
            trialValue += step;
        step >>= 1;
    }while(step > 0);
    /* We have a precision of +/- 1 for optimum OSCCAL here */
    /* now do a neighborhood search for optimum value */
    optimumValue = trialValue;
    optimumDev = x; // this is certainly far away from optimum
    for(OSCCAL = trialValue - 1; OSCCAL <= trialValue + 1; OSCCAL++){
        x = usbMeasureFrameLength() - targetValue;
        if(x < 0)
            x = -x;
        if(x < optimumDev){
            optimumDev = x;
            optimumValue = OSCCAL;
        }
    }
    OSCCAL = optimumValue;
}

void usbEventResetReady(void)
{
    cli();  // usbMeasureFrameLength() counts CPU cycles, so disable interrupts.
    calibrateOscillator();
    sei();
    eeprom_write_byte(&ee_osc, OSCCAL);   // store the calibrated value in EEPROM
}

void set_wpm(uint32_t new_wpm)
{
	// This is WPM * 1000 due to need for fractional WPM for slow modes
	//
	// Dit length in milliseconds is 1200 ms / WPM
	// Divide by 2 ms to get number of timer ticks
	dit_length = (1200000L / new_wpm) / 2;
}

uint32_t get_msg_delay(uint8_t delay_minutes)
{
	// The single function parameter is the message delay time in minutes
	if(delay_minutes > MAX_MSG_DELAY)
		delay_minutes = MAX_MSG_DELAY;

	// Number of clock ticks is the number of minutes * 29978 ticks/per min  --63?
	return (uint32_t)delay_minutes * 29978L;
}

void init_tx(void)
{
		if(cur_mode == MODE_WSPR)
		{
			// Reset the WSPR symbol buffer
			eeprom_read_block((void*)&msg_buffer, (const void*)&ee_wspr_symbols, WSPR_BUFFER_SIZE - 1);
			cur_msg_p = msg_buffer;
			cur_character = '\0';

			// Reset to IDLE state
			cur_state_end = cur_timer;
			cur_state = STATE_IDLE;
		}
		else
		{
			// Reset the message buffer
			reset_buffer();

			// If in message delay mode, set the delay
			msg_delay_end = cur_timer + get_msg_delay(msg_delay);

			// Reset Hell index
			cur_hell_row = 0;
			cur_hell_col = 0;

			// Reset WPM
			if(cur_mode == MODE_CW)
				wpm = eeprom_read_word(&ee_wpm);
			else
				wpm = pgm_read_word(&dit_speed[cur_mode]);
			set_wpm(wpm);


			// Reset to IDLE state
			cur_state_end = cur_timer;
			cur_state = STATE_IDLE;
		}

}

void debounce(void)
{
	static uint16_t S1_on_count, S1_off_count;

	// Debounce S1
	if(bit_is_clear(S1_PIN, S1))
	{
		if(S1_on_count < DEBOUNCE_PRESS_TIME)
			S1_on_count++;
		S1_off_count = 0;
	}
	else
	{
		if(S1_off_count < DEBOUNCE_PRESS_TIME)
			S1_off_count++;
		S1_on_count = 0;
	}

	// Set button flags according to final debounce count
	if(S1_on_count >= DEBOUNCE_PRESS_TIME)
		S1_active = TRUE;
	if(S1_off_count >= DEBOUNCE_PRESS_TIME)
		S1_active = FALSE;
}

void reset_buffer(void)
{
	memset(msg_buffer, '\0', WSPR_BUFFER_SIZE);
	if(cur_buffer == BUFFER_1)
		eeprom_read_block((void*)&msg_buffer, (const void*)&ee_msg_mem_1, MSG_BUFFER_SIZE - 1);
	else
		eeprom_read_block((void*)&msg_buffer, (const void*)&ee_msg_mem_2, MSG_BUFFER_SIZE - 1);
	cur_msg_p = msg_buffer;
	cur_character = '\0';
}

void init_cwid(void)
{
	cwid = TRUE;
	prev_mode = cur_mode;
	prev_wpm = wpm;
	prev_buffer = cur_buffer;
	prev_character = cur_character;
	prev_msg_p = cur_msg_p;
	//prev_state_end = cur_state_end;
	//prev_state = cur_state;

	cur_mode = MODE_CW;
	wpm = CWID_WPM;
	set_wpm(wpm);
	cur_buffer = BUFFER_2;
	reset_buffer();

	// Give a DAH delay w/ TX off so we can properly distinguish CW from QRSS
	cur_state_end = cur_timer + (dit_length * MULT_DAH);
	cur_state = STATE_DAHDELAY;
}

int main(void)
{
	uchar i;
	uchar calibrationValue;

	//static uint32_t cur_timer = 0;

	//-------------------------------------------------------------------------------------------
	// VUSB init
	//-------------------------------------------------------------------------------------------

	// Load the osc calibration value
	calibrationValue = eeprom_read_byte(&ee_osc); /* calibration value from last time */
	if(calibrationValue != 0xff){
		OSCCAL = calibrationValue;
	}

	wdt_enable(WDTO_1S);
	/* Even if you don't use the watchdog, turn it off here. On newer devices,
	 * the status of the watchdog (on/off, period) is PRESERVED OVER RESET!
	 */

	/* RESET status: all port bits are inputs without pull-up.
	 * That's the way we need D+ and D-. Therefore we don't need any
	 * additional hardware initialization.
	 */

	usbInit();
	usbDeviceDisconnect();  /* enforce re-enumeration, do this while interrupts are disabled! */
	i = 0;
	while(--i){             /* fake USB disconnect for > 250 ms */
		wdt_reset();
		_delay_ms(1);
	}
	usbDeviceConnect();


	//-------------------------------------------------------------------------------------------
	// OpenBeacon init
	//-------------------------------------------------------------------------------------------

	// Set up Timer0 for event timer
	// 16.5 MHz clock, /256 prescale, 129 count = 2.0014 ms
	// We'll consider it 2 ms for our purposes
	TCCR0A |= _BV(WGM01); // CTC mode
	TCCR0B = _BV(CS02); // Prescale /256
	OCR0A = 128;
	TIMSK |= _BV(OCIE0A); // Enable CTC interrupt

	// Set up Timer1 for fast PWM (500 kHz)
	TCCR1 = _BV(CS10);
	GTCCR = _BV(PWM1B) | _BV(COM1B1);
	OCR1B = 0; // Initial PWM value
	OCR1C = 255;
	PLLCSR = _BV(PLLE) | _BV(PCKE);

	// Initialize ports
	FSK_DDR |= _BV(FSK);
	FSK_PORT &= ~(_BV(FSK));

	KEY_DDR |= _BV(KEY);
	KEY_PORT |= _BV(KEY);

	S1_DDR &= ~(_BV(S1));
	S1_PORT |= _BV(S1); // Enable pull-up

	OCR1B = 255;

	// Transmitter off
	KEY_PORT |= _BV(KEY);

	// Set up the message buffer
	memset(msg_buffer, '\0', WSPR_BUFFER_SIZE);
	cur_msg_p = msg_buffer;

	// Initialize states
	cur_mode = eeprom_read_byte(&ee_mode);
	cur_state = STATE_IDLE;


	if(cur_mode == MODE_CW)
		wpm = eeprom_read_word(&ee_wpm);
	else
		wpm = pgm_read_word(&dit_speed[cur_mode]);
	set_wpm(wpm);

	cur_buffer = eeprom_read_byte(&ee_buffer);
	dfcw_offset = eeprom_read_byte(&ee_dfcw_offset);

	if(cur_buffer == BUFFER_1)
		eeprom_read_block((void*)&msg_buffer, (const void*)&ee_msg_mem_1, MSG_BUFFER_SIZE - 1);
	else if(cur_buffer == BUFFER_2)
		eeprom_read_block((void*)&msg_buffer, (const void*)&ee_msg_mem_2, MSG_BUFFER_SIZE - 1);

	msg_delay = eeprom_read_byte(&ee_msg_delay);
	msg_delay_end = cur_timer + get_msg_delay(msg_delay);

	next_cwid = cur_timer + get_msg_delay(CWID_DELAY);

	init_tx();

	sei();


	while(1)
	{
		// Latch the current time
		// MUST disable interrupts during this read or there will be an occasional corruption of cur_timer
		cli();
		cur_timer = timer;
		sei();

		// Process button press
		if(S1_active)
			init_tx();

		// Handle CW ID if one hasn't been triggered in 10 minutes
		if(cur_timer > next_cwid && !cwid && (cur_mode != MODE_WSPR || cur_mode != MODE_CW))
		{
			init_cwid();
			next_cwid = cur_timer + get_msg_delay(CWID_DELAY);
		}

		// State machine
		switch(cur_mode)
		{
		case MODE_DFCW3:
		case MODE_DFCW6:
		case MODE_DFCW10:
		case MODE_DFCW120:
		case MODE_QRSS3:
		case MODE_QRSS6:
		case MODE_QRSS10:
		case MODE_QRSS120:
		case MODE_CW:
			switch(cur_state)
			{
			case STATE_IDLE:
				// TX off
				KEY_PORT &= ~(_BV(KEY));

				if(msg_delay > 0 && msg_delay_end <= cur_timer && cur_msg_p == msg_buffer)
				{
					msg_delay_end = cur_timer + get_msg_delay(msg_delay);
					cur_state_end = cur_timer + (dit_length * MULT_WORDDELAY);
					//if(cur_mode != MODE_CW)
						cur_state = STATE_PREAMBLE;
					//else
						//cur_state = STATE_IDLE;
					break;
				}

				// If this is the first time thru the message loop, get the first character, then wait a moment before starting message if not CW
				if((cur_msg_p == msg_buffer) && (cur_character == '\0'))
				{
					cur_character = pgm_read_byte(&morsechar[(*cur_msg_p) - MORSE_CHAR_START]);
					if(cur_mode != MODE_CW)
					{
						cur_state_end = cur_timer + (dit_length * MULT_DAH);
						cur_state = STATE_PREAMBLE;
						break;
					}
				}

				// Get the current element in the current character
				if(cur_character != '\0')
				{
					if(cur_character == 0b10000000 || cur_character == 0b11111111)	// End of character marker or SPACE
					{
						// Set next state based on whether EOC or SPACE
						if(cur_character == 0b10000000)
						{
							cur_state_end = cur_timer + (dit_length * MULT_DAH);
							cur_state = STATE_DAHDELAY;
						}
						else
						{
							cur_state_end = cur_timer + (dit_length * MULT_WORDDELAY);
							cur_state = STATE_WORDDELAY;
						}

						// Grab next character, set state to inter-character delay
						cur_msg_p++;

						// If we read a NULL from the announce buffer, set cur_character to NULL,
						// otherwise set to correct morse character
						if((*cur_msg_p) == '\0')
							cur_character = '\0';
						else
							cur_character = pgm_read_byte(&morsechar[(*cur_msg_p) - MORSE_CHAR_START]);
					}
					else
					{
						// Mask off MSb, set cur_element
						if((cur_character & 0b10000000) == 0b10000000)
						{
							cur_state_end = cur_timer + (dit_length * MULT_DAH);
							cur_state = STATE_DAH;
						}
						else
						{
							cur_state_end = cur_timer + dit_length;
							cur_state = STATE_DIT;
						}

						// Shift left to get next element
						cur_character = cur_character << 1;
					}
				}
				else // Buffer is now empty
				{
					// If in CW ID mode, reset back to original parameters
					if(cwid)
					{
						cur_mode = prev_mode;
						wpm = prev_wpm;
						set_wpm(wpm);
						cur_character = prev_character;
						cur_buffer = prev_buffer;
						//cur_state_end = prev_state_end;
						memset(msg_buffer, '\0', WSPR_BUFFER_SIZE);
						if(cur_buffer == BUFFER_1)
							eeprom_read_block((void*)&msg_buffer, (const void*)&ee_msg_mem_1, MSG_BUFFER_SIZE - 1);
						else if(cur_buffer == BUFFER_2)
							eeprom_read_block((void*)&msg_buffer, (const void*)&ee_msg_mem_2, MSG_BUFFER_SIZE - 1);
						cur_msg_p = prev_msg_p;
						next_cwid = cur_timer + get_msg_delay(CWID_DELAY);
						cwid = FALSE;
						//cur_state = prev_state;
						cur_state = STATE_IDLE;
					}
					else
					{
						// Reload the message buffer and set buffer pointer back to beginning
						reset_buffer();

						if(msg_delay == 0)
						{
							// If a constantly repeating message, put a word delay at the end of message
							cur_state_end = cur_timer + (dit_length * MULT_WORDDELAY);
							cur_state = STATE_WORDDELAY;
						}
						else
						{
							// Otherwise, set the message delay time
							if(msg_delay_end < cur_timer + (dit_length * MULT_WORDDELAY))
								cur_state_end = cur_timer + (dit_length * MULT_WORDDELAY);
							else
								cur_state_end = msg_delay_end;

							cur_state = STATE_MSGDELAY;
						}


						// Do a CW ID
						if(cur_mode != MODE_CW)
						{
							init_cwid();
							next_cwid = cur_timer + get_msg_delay(CWID_DELAY);
						}

					}
				}

				break;

			case STATE_PREAMBLE:
				// Wait a word delay with TX on before starting message
				OCR1B = 0;

				if(cur_mode == MODE_QRSS3 || cur_mode == MODE_QRSS6 || cur_mode == MODE_QRSS10 || cur_mode == MODE_QRSS120 || cur_mode == MODE_CW)
				{
					// Transmitter off
					KEY_PORT &= ~(_BV(KEY));
				}
				else
				{
					// Transmitter on
					KEY_PORT |= _BV(KEY);
				}


				// When done waiting, go back to IDLE state to start the message
				if(cur_timer > cur_state_end)
				{
					cur_state = STATE_IDLE;
				}
				break;

			case STATE_DIT:
			case STATE_DAH:
				switch(cur_mode)
				{
				case MODE_DFCW3:
				case MODE_DFCW6:
				case MODE_DFCW10:
				case MODE_DFCW120:
					// Transmitter on
					KEY_PORT |= _BV(KEY);

					// Set FSK to MARK (lower capacitance/higher freq)
					OCR1B = dfcw_offset;
					break;
				case MODE_QRSS3:
				case MODE_QRSS6:
				case MODE_QRSS10:
				case MODE_QRSS120:
				case MODE_CW:
					// Transmitter on
					KEY_PORT |= _BV(KEY);

					// Set FSK to 0 (maximum capacitance/minimum freq)
					OCR1B = 0;
					break;
				default:
					break;
				}

				if(cur_timer > cur_state_end)
				{
					switch(cur_mode)
					{
					case MODE_DFCW3:
					case MODE_DFCW6:
					case MODE_DFCW10:
					case MODE_DFCW120:
						// Transmitter on
						KEY_PORT |= _BV(KEY);

						// Set FSK to 0 (maximum capacitance/minimum freq)
						OCR1B = 0;
						break;
					case MODE_QRSS3:
					case MODE_QRSS6:
					case MODE_QRSS10:
					case MODE_QRSS120:
					case MODE_CW:
						// Transmitter off
						KEY_PORT &= ~(_BV(KEY));

						// Set FSK to 0 (maximum capacitance/minimum freq)
						OCR1B = 0;
						break;
					default:
						break;
					}

					cur_state_end = cur_timer + dit_length;
					cur_state = STATE_DITDELAY;
				}
				break;
			case STATE_DITDELAY:
			case STATE_DAHDELAY:
			case STATE_WORDDELAY:
			case STATE_MSGDELAY:
				OCR1B = 0;

				if(cur_state == STATE_MSGDELAY || cur_mode == MODE_QRSS3 || cur_mode == MODE_QRSS6 || cur_mode == MODE_QRSS10 || cur_mode == MODE_QRSS120 || cur_mode == MODE_CW)
				{
					// Transmitter off
					KEY_PORT &= ~(_BV(KEY));
				}
				else
				{
					// Transmitter on
					KEY_PORT |= _BV(KEY);
				}

				if(cur_timer > cur_state_end)
				{
					cur_state = STATE_IDLE;
				}
				break;


			default:
				break;
			}
			break;

		case MODE_HELL:
			switch(cur_state)
			{
			case STATE_IDLE:
				if(msg_delay > 0 && msg_delay_end <= cur_timer)
				{
					msg_delay_end = cur_timer + get_msg_delay(msg_delay);
				}

				// If this is the first time thru the message loop, get the first character
				if((cur_msg_p == msg_buffer) && (cur_hell_char == '\0'))
				{
					cur_hell_col = 0;
					cur_hell_char = pgm_read_byte(&fontchar[(*cur_msg_p) - FONT_START][cur_hell_col++]);
					cur_state_end = cur_timer + (dit_length);
					cur_state = STATE_HELLCOL;
				}
				else
				{
					cur_hell_char = pgm_read_byte(&fontchar[(*cur_msg_p) - FONT_START][cur_hell_col++]);

					if(cur_hell_col > HELL_COLS)
					{
						// Reset Hell column
						cur_hell_col = 0;

						// Grab next character
						cur_msg_p++;

						if((*cur_msg_p) == '\0')
						{
							// End of message
							// Reload the message buffer and set buffer pointer back to beginning
							reset_buffer();

							if(msg_delay == 0)
							{
								// If a constantly repeating message, put a word delay at the end of message
								cur_state_end = cur_timer + (dit_length * MULT_HELL_WORDDELAY);
							}
							else
							{
								// Otherwise, set the message delay time
								if(msg_delay_end < cur_timer + (dit_length * MULT_HELL_WORDDELAY))
									cur_state_end = cur_timer + (dit_length * MULT_HELL_WORDDELAY);
								else
									cur_state_end = msg_delay_end;
							}

							cur_state = STATE_WORDDELAY;

							// Do a CW ID
							init_cwid();
							next_cwid = cur_timer + get_msg_delay(CWID_DELAY);
						}
						else
						{
							cur_state_end = cur_timer + (dit_length * MULT_HELL_CHARDELAY);
							cur_state = STATE_CHARDELAY;
						}
					}
					else
					{
						//cur_hell_char = pgm_read_byte(&fontchar[(*cur_msg_p) - FONT_START][cur_hell_col]);
						cur_state_end = cur_timer + (dit_length);
						cur_state = STATE_HELLCOL;
					}
				}
				break;
			case STATE_HELLCOL:
				OCR1B = pgm_read_byte(&hell_tune[cur_hell_row]);
				if((cur_hell_char & (1 << cur_hell_row)) != 0)
				{
					// Pixel on
					KEY_PORT |= _BV(KEY);
				}
				else
				{
					// Pixel off
					KEY_PORT &= ~(_BV(KEY));
				}

				if(cur_timer > cur_state_end)
				{
					cur_hell_row++;
					if(cur_hell_row > HELL_ROWS)
					{
						cur_hell_row = 0;
						cur_state = STATE_IDLE;
					}
					else
					{
						cur_state_end = cur_timer + dit_length;
						cur_state = STATE_HELLCOL;
					}
				}
				break;

			case STATE_WORDDELAY:
			case STATE_CHARDELAY:
				OCR1B = 0;

				// Transmitter off
				KEY_PORT &= ~(_BV(KEY));

				if(cur_timer > cur_state_end)
					cur_state = STATE_IDLE;
				break;
			default:
				cur_state = STATE_IDLE;
				break;
			}
			break;

		case MODE_GLYPHCODE:
			switch(cur_state)
			{
			case STATE_IDLE:
				// Transmitter off
				KEY_PORT &= ~(_BV(KEY));
				OCR1B = 0;

				if(msg_delay > 0 && msg_delay_end <= cur_timer)
				{
					msg_delay_end = cur_timer + get_msg_delay(msg_delay);
				}

				// If this is the first time thru the message loop, get the first character
				if((cur_msg_p == msg_buffer) && (cur_character == '\0'))
				{
					cur_character = pgm_read_byte(&morsechar[(*cur_msg_p) - MORSE_CHAR_START]);
				}

				// Get the current element in the current character
				if(cur_character != '\0')
				{
					if(cur_character == 0b10000000 || cur_character == 0b11111111)	// End of character marker or SPACE
					{
						// Set next state based on whether EOC or SPACE
						if(cur_character == 0b10000000)
						{
							cur_state_end = cur_timer + (dit_length * MULT_HELL_GLYPHDELAY);
							cur_state = STATE_CHARDELAY;
						}
						else
						{
							cur_state_end = cur_timer + (dit_length * MULT_HELL_WORDDELAY);
							cur_state = STATE_WORDDELAY;
						}

						// Grab next character, set state to inter-character delay
						cur_msg_p++;

						// If we read a NULL from the announce buffer, set cur_character to NULL,
						// otherwise set to correct morse character
						if((*cur_msg_p) == '\0')
							cur_character = '\0';
						else
							cur_character = pgm_read_byte(&morsechar[(*cur_msg_p) - MORSE_CHAR_START]);
					}
					else
					{
						// Mask off MSb, set cur_element
						if((cur_character & 0b10000000) == 0b10000000)
						{
							// Set to output glyph 1 if a dah
							eeprom_read_block((void*)&temp_buffer, (const void*)&ee_glyph_1, GLYPH_SIZE - 1);
						}
						else
						{
							// Set to output glyph 2 if a dit
							eeprom_read_block((void*)&temp_buffer, (const void*)&ee_glyph_2, GLYPH_SIZE - 1);
						}

						cur_glyph_p = temp_buffer;
						cur_hell_col = 0;
						cur_hell_char = *cur_glyph_p;
						cur_state_end = cur_timer + dit_length;
						cur_state = STATE_HELLCOL;

						// Shift left to get next element
						cur_character = cur_character << 1;
					}
				}
				else
				{
					// Reload the message buffer and set buffer pointer back to beginning
					reset_buffer();

					if(msg_delay == 0)
					{
						// If a constantly repeating message, put a word delay at the end of message
						cur_state_end = cur_timer + (dit_length * MULT_HELL_WORDDELAY);
						cur_state = STATE_WORDDELAY;
					}
					else
					{
						// Otherwise, set the message delay time
						if(msg_delay_end < cur_timer + (dit_length * MULT_HELL_WORDDELAY))
							cur_state_end = cur_timer + (dit_length * MULT_HELL_WORDDELAY);
						else
							cur_state_end = msg_delay_end;

						cur_state = STATE_MSGDELAY;
					}

					// Do a CW ID
					init_cwid();
					next_cwid = cur_timer + get_msg_delay(CWID_DELAY);
				}

				break;

			case STATE_HELLCOL:
				OCR1B = pgm_read_byte(&hell_tune[cur_hell_row]);
				if((cur_hell_char & (1 << cur_hell_row)) != 0)
				{
					// Pixel on
					KEY_PORT |= _BV(KEY);
				}
				else
				{
					// Pixel off
					KEY_PORT &= ~(_BV(KEY));
				}

				if(cur_timer > cur_state_end)
				{
					cur_hell_row++;
					if(cur_hell_row > HELL_ROWS)
					{
						cur_hell_row = 0;
						cur_glyph_p++;
						cur_hell_char = *cur_glyph_p;
						if(cur_hell_char == 0x80)
						{
							cur_hell_col = 0;
							cur_state_end = cur_timer + (dit_length * MULT_HELL_CHARDELAY);
							cur_state = STATE_CHARDELAY;
						}
						else
						{
							cur_state_end = cur_timer + dit_length;
							cur_state = STATE_HELLCOL;
						}
					}
					else
					{
						cur_state_end = cur_timer + dit_length;
						cur_state = STATE_HELLCOL;
					}
				}
				break;

			case STATE_WORDDELAY:
			case STATE_CHARDELAY:
				// Transmitter off
				KEY_PORT &= ~(_BV(KEY));

				OCR1B = 0;

				if(cur_timer > cur_state_end)
					cur_state = STATE_IDLE;
				break;
			default:
				cur_state = STATE_IDLE;
				break;
			}
			break;

		case MODE_WSPR:
			switch(cur_state)
			{
			case STATE_IDLE:
				// Transmitter off
				KEY_PORT &= ~(_BV(KEY));

				if(cur_timer > cur_state_end)
				{
					cur_state_end = cur_timer + WSPR_SYMBOL_LENGTH;
					cur_state = STATE_WSPR;
				}
				break;

			case STATE_WSPR:
				// Transmitter on
				KEY_PORT |= _BV(KEY);

				// Transmit the WSPR symbol
				switch(*cur_msg_p)
				{
				case '0':
					OCR1B = pgm_read_byte(&hell_tune[6]);
					break;

				case '1':
					OCR1B = pgm_read_byte(&hell_tune[5]);
					break;

				case '2':
					OCR1B = pgm_read_byte(&hell_tune[4]);
					break;

				case '3':
					OCR1B = pgm_read_byte(&hell_tune[3]);
					break;

				default:
					break;
				}

				if(cur_timer > cur_state_end)
				{
					// Get the next symbol from the buffer
					cur_msg_p++;

					// If at end of buffer, reset and go into message delay
					if((*cur_msg_p) == '\0')
					{
						eeprom_read_block((void*)&msg_buffer, (const void*)&ee_wspr_symbols, WSPR_BUFFER_SIZE - 1);
						cur_msg_p = msg_buffer;

						cur_state_end = UINT32_MAX;
						cur_state = STATE_IDLE;
					}
					else
					{
						cur_state_end = cur_timer + WSPR_SYMBOL_LENGTH;
						cur_state = STATE_WSPR;
					}
				}
				break;
			default:
				cur_state = STATE_IDLE;
				break;
			}
			break;

		case MODE_CAL:
			switch(cur_state)
			{
			case STATE_IDLE:
				KEY_PORT &= ~(_BV(KEY));
				cur_hell_row++;
				if(cur_hell_row > HELL_ROWS)
					cur_hell_row = 0;

				cur_state_end = cur_timer + dit_length;
				cur_state = STATE_CAL;
				break;
			case STATE_CAL:
				KEY_PORT |= _BV(KEY);
				OCR1B = pgm_read_byte(&hell_tune[cur_hell_row]);

				if(cur_timer > cur_state_end)
					cur_state = STATE_IDLE;
				break;
			default:
				cur_state = STATE_CAL;
				break;
			}
			break;
		default:
			// Switch to a default mode???
			break;
		}


		// Housekeeping and handle USB
		wdt_reset();
		usbPoll();
	}
}
