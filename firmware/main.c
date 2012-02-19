/*
 * main.c
 *
 *  Created on: Jan 8, 2012
 *      Author: jason
 */

#include <stdint.h>
#include <string.h>
#include <ctype.h>
#include <avr/io.h>
#include <avr/sfr_defs.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <avr/wdt.h>
#include <avr/pgmspace.h>   /* required by usbdrv.h */

#include "usbdrv/usbdrv.h"
#include "usbdrv/oddebug.h"        /* This is also an example for using debug macros */

#include "requests.h"
#include "usbconfig.h"
#include "morsechar.h"
#include "font.h"
#include "modes.h"

// Defines
#define FSK_DDR					DDRB
#define FSK_PORT				PORTB
#define FSK						PB4

#define KEY_DDR					DDRB
#define KEY_PORT				PORTB
#define KEY						PB3

#define MSG_BUFFER_SIZE			41			// Message size in characters

#define DFCW_DEFAULT_OFFSET		100

#define MULT_DAH				3			// DAH is 3x a DIT
#define MULT_WORDDELAY			7			// Space between words is 7 dits
#define MULT_HELL_WORDDELAY		500
#define MULT_HELL_CHARDELAY		10
#define HELL_ROW_RPT			3

#define DEFAULT_MODE			0
#define DEFAULT_WPM				40

// Enumerations
enum BOOL {TRUE, FALSE};
enum STATE {STATE_IDLE, STATE_DIT, STATE_DAH, STATE_DITDELAY, STATE_DAHDELAY, STATE_WORDDELAY, STATE_CHARDELAY, STATE_HELLCOL, STATE_HELLROW,
			STATE_CAL};

// Global variables
uint16_t dit_length;
enum MODE cur_mode;
enum STATE cur_state, next_state;
uint32_t cur_state_end;
static char msg_buffer[MSG_BUFFER_SIZE];
static char in_buffer[MSG_BUFFER_SIZE];
char * cur_msg_p;
char cur_character = '\0';
char cur_hell_char = '\0';
uint8_t cur_hell_col = 0;
uint8_t cur_hell_row = 0;

static uchar currentPosition = 0;
static uchar bytesRemaining = 0;

// Global variables used in ISRs
volatile uint32_t timer; // A 32-bit timer will count for 2^32 * 2 ms = ~16 years
volatile enum BOOL tx_on;
volatile uint8_t fsk_tune;

// EEPROM variables
uint8_t EEMEM ee_osc;
uint8_t	EEMEM ee_mode = DEFAULT_MODE;
uint16_t EEMEM ee_wpm = DEFAULT_WPM;
uchar EEMEM ee_msg_mem_1[MSG_BUFFER_SIZE] = "N0CALL";

// Global constants
//const uint8_t hell_tune[HELL_ROWS] = {0, 42, 84, 126, 169, 210, 252};
//const uint8_t hell_tune[HELL_ROWS] = {252, 210, 169, 126, 84, 42, 0};
const uint8_t hell_tune[HELL_ROWS] = {252, 185, 140, 85, 53, 23, 0};

// Function prototypes
void set_wpm(uint16_t);

// Interrupt service routines
ISR(TIM0_COMPA_vect)
{
	// Tick the clock
	timer++;
}

usbMsgLen_t usbFunctionSetup(uchar data[8])
{
	usbRequest_t *rq = (void *)data;
	static uchar dataBuffer[4];  /* buffer must stay valid when usbFunctionSetup returns */

    if(rq->bRequest == CUSTOM_RQ_ECHO)
    { /* echo -- used for reliability tests */
        dataBuffer[0] = rq->wValue.bytes[0];
        dataBuffer[1] = rq->wValue.bytes[1];
        dataBuffer[2] = rq->wIndex.bytes[0];
        dataBuffer[3] = rq->wIndex.bytes[1];
        usbMsgPtr = dataBuffer;         /* tell the driver which data to return */
        return 4;
    }
    else if(rq->bRequest == CUSTOM_RQ_SET_MODE)
    {
    	// Set mode
        cur_mode = rq->wValue.bytes[0];
        eeprom_write_byte(&ee_mode, cur_mode);

        // Reset message buffer
        eeprom_read_block((void*)&msg_buffer, (const void*)&ee_msg_mem_1, MSG_BUFFER_SIZE - 1);
		cur_msg_p = msg_buffer;
		cur_character = '\0';

		// Reset Hell index
        cur_hell_row = 0;
        cur_hell_col = 0;

        // Reset WPM
        set_wpm(dit_speed[cur_mode]);
        eeprom_write_word(&ee_wpm, dit_speed[cur_mode]);

        // Put back in IDLE state
        cur_state = STATE_IDLE;
    }
    else if(rq->bRequest == CUSTOM_RQ_GET_MODE)
    {
        dataBuffer[0] = cur_mode;
        usbMsgPtr = dataBuffer;         /* tell the driver which data to return */
        return 1;                       /* tell the driver to send 1 byte */
    }
    else if(rq->bRequest == CUSTOM_RQ_SET_MSG_1)
    {
        currentPosition = 0;                // initialize position index
        bytesRemaining = rq->wLength.word;  // store the amount of data requested
        if(bytesRemaining > sizeof(msg_buffer)) // limit to buffer size
            bytesRemaining = sizeof(msg_buffer);
        return USB_NO_MSG;        // tell driver to use usbFunctionWrite()
    }
    else if(rq->bRequest == CUSTOM_RQ_GET_MSG_1)
    {
    	usbMsgLen_t len = MSG_BUFFER_SIZE;                     // we return up to 64 bytes
		if(len > rq->wLength.word)          // if the host requests less than we have
			len = rq->wLength.word;         // return only the amount requested
		usbMsgPtr = (uchar *)msg_buffer;                 // tell driver where the buffer starts
		return len;                         // tell driver how many bytes to send
    }
    return 0;   /* default for not implemented requests: return no data back to host */
}

uchar usbFunctionWrite(uchar *data, uchar len)
{
	uchar i;

    if(len > bytesRemaining)                // if this is the last incomplete chunk
        len = bytesRemaining;               // limit to the amount we can store
    bytesRemaining -= len;
    for(i = 0; i < len; i++)
        msg_buffer[currentPosition++] = data[i];

    if(bytesRemaining == 0)
    {
		// Update EEPROM with new message
		//eeprom_busy_wait();
    	//strcpy(msg_buffer, in_buffer);
		eeprom_write_block((const void*)&msg_buffer, (void*)&ee_msg_mem_1, MSG_BUFFER_SIZE - 1);

		// Reset message buffer
		cur_msg_p = msg_buffer;
		cur_character = '\0';

		// Reset Hell index
		cur_hell_row = 0;
		cur_hell_col = 0;

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

void set_wpm(uint16_t new_wpm)
{
	// This is WPM * 100 due to need for fractional WPM for slow modes
	//
	// Dit length in milliseconds is 1200 ms / WPM
	// Divide by 2 ms to get number of timer ticks
	dit_length = (120000 / new_wpm) / 2;
}

int main(void)
{
	uchar i;
	uchar calibrationValue;

	static uint32_t cur_timer = 0;


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
	// OpenQRSS init
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

	OCR1B = 255;

	// Transmitter off
	KEY_PORT |= _BV(KEY);

	// Set up the message buffer
	memset(msg_buffer, '\0', MSG_BUFFER_SIZE);
	cur_msg_p = msg_buffer;

	// Initialize states
	cur_mode = eeprom_read_byte(&ee_mode);
	cur_state = STATE_IDLE;
	set_wpm(eeprom_read_word(&ee_wpm));
	//set_wpm(40);
	eeprom_read_block((void*)&msg_buffer, (const void*)&ee_msg_mem_1, MSG_BUFFER_SIZE - 1);
	//strcpy(msg_buffer, "NT7S CN85");

	sei();


	while(1)
	{
		// Latch the current time
		// MUST disable interrupts during this read or there will be an occasional corruption of cur_timer
		cli();
		cur_timer = timer;
		sei();

		switch(cur_mode)
		{
		case MODE_DFCW3:
		case MODE_DFCW6:
		case MODE_DFCW10:
		case MODE_QRSS3:
		case MODE_QRSS6:
		case MODE_QRSS10:
		case MODE_CW5:
		case MODE_CW13:
			switch(cur_state)
			{
			case STATE_IDLE:
				// We should only be in IDLE to figure out what to do next

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
				else
				{
					// Reload the message buffer and set buffer pointer back to beginning
					eeprom_read_block((void*)&msg_buffer, (const void*)&ee_msg_mem_1, MSG_BUFFER_SIZE - 1);
					cur_msg_p = msg_buffer;
					cur_character = '\0';

					// Put a word delay at the end of message
					cur_state_end = cur_timer + (dit_length * MULT_WORDDELAY);
					cur_state = STATE_WORDDELAY;
				}

				break;
			case STATE_DIT:
			case STATE_DAH:
				switch(cur_mode)
				{
				case MODE_DFCW3:
				case MODE_DFCW6:
				case MODE_DFCW10:
					// Transmitter on
					KEY_PORT |= _BV(KEY);

					// Set FSK to MARK (lower capacitance/higher freq)
					OCR1B = DFCW_DEFAULT_OFFSET;
					break;
				case MODE_QRSS3:
				case MODE_QRSS6:
				case MODE_QRSS10:
				case MODE_CW5:
				case MODE_CW13:
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
						// Transmitter on
						KEY_PORT |= _BV(KEY);

						// Set FSK to 0 (maximum capacitance/minimum freq)
						OCR1B = 0;
						break;
					case MODE_QRSS3:
					case MODE_QRSS6:
					case MODE_QRSS10:
					case MODE_CW5:
					case MODE_CW13:
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
				OCR1B = 0;
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
				// If this is the first time thru the message loop, get the first character
				/*
				if((cur_msg_p == msg_buffer) && (cur_hell_char == '\0'))
				{
					cur_hell_col = 0;
					cur_hell_char = pgm_read_byte(&fontchar[(*cur_msg_p) - FONT_START][cur_hell_col]);
					cur_state_end = cur_timer + (dit_length);
					cur_state = STATE_HELLCOL;
				}
				else
				{
				*/
					cur_hell_char = pgm_read_byte(&fontchar[(*cur_msg_p) - FONT_START][cur_hell_col]);

					cur_hell_col++;
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
							eeprom_read_block((void*)&msg_buffer, (const void*)&ee_msg_mem_1, MSG_BUFFER_SIZE - 1);
							cur_msg_p = msg_buffer;
							cur_hell_char = '\0';

							// Put a word delay at the end of message
							cur_state_end = cur_timer + (dit_length * MULT_HELL_WORDDELAY);
							cur_state = STATE_WORDDELAY;
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
					//}
				}
				break;
			case STATE_HELLCOL:
				OCR1B = hell_tune[cur_hell_row];
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
		case MODE_CAL:
			switch(cur_state)
			{
			case STATE_IDLE:
				cur_hell_row++;
				if(cur_hell_row > HELL_ROWS)
					cur_hell_row = 0;

				cur_state_end = cur_timer + dit_length;
				cur_state = STATE_CAL;
				break;
			case STATE_CAL:
				OCR1B = hell_tune[cur_hell_row];

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
			cur_mode = MODE_DFCW3;
			set_wpm(dit_speed[cur_mode]);
			break;
		}


		// Housekeeping and handle USB
		wdt_reset();
		usbPoll();
	}
}
