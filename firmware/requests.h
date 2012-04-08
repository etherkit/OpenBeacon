/*
 * requests.h
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

/* This header is shared between the firmware and the host software. It
 * defines the USB request numbers (and optionally data types) used to
 * communicate between the host and the device.
 */

#ifndef __REQUESTS_H_INCLUDED__
#define __REQUESTS_H_INCLUDED__

// Request that the device sends back wValue and wIndex. This is used with
// random data to test the reliability of the communication.
#define CUSTOM_RQ_ECHO				0

// Set and get operating mode based on enum in modes.h
#define CUSTOM_RQ_SET_MODE			1
#define CUSTOM_RQ_GET_MODE			2

// Set and get Message Buffer 1
#define CUSTOM_RQ_SET_MSG_1			3
#define CUSTOM_RQ_GET_MSG_1			4

// Set and get Message Buffer 2
#define CUSTOM_RQ_SET_MSG_2			5
#define CUSTOM_RQ_GET_MSG_2			6

// Set and get WPM
#define CUSTOM_RQ_SET_WPM			7
#define CUSTOM_RQ_GET_WPM			8

// Set and get message delay
#define CUSTOM_RQ_SET_MSG_DLY		9
#define CUSTOM_RQ_GET_MSG_DLY		10

// Set and get current message buffer
#define CUSTOM_RQ_SET_CUR_BUFFER	11
#define CUSTOM_RQ_GET_CUR_BUFFER	12

// Set and get DFCW offset
#define CUSTOM_RQ_SET_DFCW_OFFSET	13
#define CUSTOM_RQ_GET_DFCW_OFFSET	14

// Set custom glyph
#define CUSTOM_RQ_SET_GLYPH			15

// Set WSPR symbols
#define CUSTOM_RQ_SET_WSPR			16

// Initiate TX
#define CUSTOM_RQ_START_TX			17

#endif /* __REQUESTS_H_INCLUDED__ */
