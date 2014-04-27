/* -----------------------------------------------------------------------------
# Company name: CSSE3010 - Embedded Systems Design & Interfacing
# Project name: Practical 7 - FreeRTOS Networking
# Module name : TUIO Client
# Functionality: Extract TUIO /tuio/2dobj set messages on TCP Port 3000
# Hardware platform: NetduinoPlus (AT91SAM7)
#
# Author name: M. D'Souza
# Creation date: 060812
# Revision date (name): -
# Changes implemented (date): -
#(Comments): 
------------------------------------------------------------------------------*/

#ifndef __TUIOCLIENT_H__
#define __TUIOCLIENT_H__


/* Since this file will be included by uip.h, we cannot include uip.h
   here. But we might need to include uipopt.h if we need the u8_t and
   u16_t datatypes. */
#include "FreeRTOS.h"
#include "uipopt.h"
#include "queue.h"


//typedef struct tuioclient_state {
//	char inputbuffer[100];
//} uip_tcp_appstate_t;


/* My variables */
xQueueHandle xReactivisionQueue;
int tuio_debug;


/* Finally we define the application function. */
void tuioclient_appcall(void);


#endif /* __TUIOCLIENT_H__ */
/** @} */
/** @} */

