/* Name: requests.h
 * Project: custom-class, a basic USB example
 * Author: Christian Starkjohann
 * Creation Date: 2008-04-09
 * Tabsize: 4
 * Copyright: (c) 2008 by OBJECTIVE DEVELOPMENT Software GmbH
 * License: GNU GPL v2 (see License.txt), GNU GPL v3 or proprietary (CommercialLicense.txt)
 * This Revision: $Id: requests.h 692 2008-11-07 15:07:40Z cs $
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

// Set and get Message Memory 1
#define CUSTOM_RQ_SET_MSG_1			3
#define CUSTOM_RQ_GET_MSG_1			4

// Set and get Message Memory 2
#define CUSTOM_RQ_SET_MSG_2			5
#define CUSTOM_RQ_GET_MSG_2			6

// Set and get WPM
#define CUSTOM_RQ_SET_WPM			7
#define CUSTOM_RQ_GET_WPM			8

#endif /* __REQUESTS_H_INCLUDED__ */
