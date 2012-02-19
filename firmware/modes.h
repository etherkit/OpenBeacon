/*
 * modes.h
 *
 *  Created on: Jan 16, 2012
 *      Author: jason
 */

#ifndef MODES_H_
#define MODES_H_

#define MODE_COUNT			10

enum MODE {MODE_DFCW3, MODE_DFCW6, MODE_DFCW10, MODE_QRSS3, MODE_QRSS6, MODE_QRSS10, MODE_CW5, MODE_CW13, MODE_HELL, MODE_CAL};

// Array of mode names, indexed to enum MODE
const char mode_list[MODE_COUNT][10] = {"dfcw3", "dfcw6", "dfcw10", "qrss3", "qrss6", "qrss10", "cw5", "cw13", "hell", "cal"};

const char mode_desc[MODE_COUNT][100] =	   {"Dual Frequency CW - 3 second dits",
											"Dual Frequency CW - 6 second dits",
											"Dual Frequency CW - 10 second dits",
											"QRSS - 3 second dits",
											"QRSS - 6 second dits",
											"QRSS - 10 second dits",
											"CW - 5 WPM",
											"CW - 13 WPM",
											"Feld Hell",
											"Calibration"};

// Array of speeds for the modes, indexed to enum MODE
// Speeds are in WPM * 100
const uint16_t dit_speed[MODE_COUNT] = {40, 20, 12, 40, 20, 12, 500, 1300, 300, 2000};

#endif /* MODES_H_ */
