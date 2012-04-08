/*
 * modes.h
 *
 *  Created on: Jan 16, 2012
 *     Author: Jason Milldrum
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

#ifndef MODES_H_
#define MODES_H_

#include <stdint.h>

#define MODE_COUNT			13

#define MSG_BUFFER_SIZE			41			// Message size in characters (+1 for \0)
#define WSPR_BUFFER_SIZE		163			// WSPR symbol buffer size, 162 symbols plus '\0'
#define GLYPH_SIZE				23			// Maximum size of custom glyph in columns (+1 for \0)
#define MAX_MSG_DELAY			30			// Maximum message delay time in minutes
#define MAX_WPM					50
#define MAX_NUM_GLYPH			2

enum MODE {MODE_DFCW3, MODE_DFCW6, MODE_DFCW10, MODE_DFCW120, MODE_QRSS3, MODE_QRSS6, MODE_QRSS10, MODE_QRSS120, MODE_CW, MODE_HELL, MODE_WSPR, MODE_GLYPHCODE, MODE_CAL};

#if OPENBEACON_CLIENT

// Array of mode names, indexed to enum MODE
const char mode_list[MODE_COUNT][10] = {"dfcw3", "dfcw6", "dfcw10", "dfcw120", "qrss3", "qrss6", "qrss10", "qrss120", "cw", "hell", "wspr", "glyphcode", "cal"};


const char mode_desc[MODE_COUNT][100] =	   {"Dual Frequency CW - 3 second dits",
											"Dual Frequency CW - 6 second dits",
											"Dual Frequency CW - 10 second dits",
											"Dual Frequency CW - 120 second dits",
											"QRSS - 3 second dits",
											"QRSS - 6 second dits",
											"QRSS - 10 second dits",
											"QRSS - 120 second dits",
											"CW",
											"Sequential Multi-tone Hell",
											"WSPR (experimental, see documentation)",
											"Glyphcode",
											"Calibration"};
#endif

// Array of speeds for the modes, indexed to enum MODE
// Speeds are in WPM * 1000
const uint16_t dit_speed[MODE_COUNT] = {400, 200, 120, 10, 400, 200, 120, 10, 5000, 5000, 1750, 5000, 10000};

#endif /* MODES_H_ */
