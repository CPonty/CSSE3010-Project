/*
 * Modified from an original work that is Copyright (c) 2001-2003, Adam Dunkels.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * This file is part of the uIP TCP/IP stack.
 *
 * $Id: main.c,v 1.10.2.4 2003/10/21 21:27:51 adam Exp $
 *
 */

/* Standard includes. */
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "USB-CDC.h"

/* uip includes. */
#include "uip.h"
#include "uip_arp.h"
#include "timer.h"
#include "clock-arch.h"
#include "debug_printf.h"

/* Additional includes. */
#include "SAM7_EMAC.h"
#include "partest.h"
#include "tuioclient.h"
#include "web_Task.h"
#include "json-data.h"

/* How long to wait before attempting to connect the MAC again. */
#define uipINIT_WAIT    ( 100 / portTICK_RATE_MS )

/* Shortcut to the header within the Rx buffer. */
#define xHeader ((struct uip_eth_hdr *) &uip_buf[ 0 ])

/* The semaphore used by the ISR to wake the uIP task. */
static xSemaphoreHandle xEMACSemaphore;

//Globals
struct uip_conn * tuio_conn;
//struct uip_conn * returned_conn;
uint16_t tuio_lport, tuio_rport;
uint8_t ack_wait = 0;

/* The incoming TCP packet buffer's header */
#define BUF ((struct uip_tcpip_hdr *)&uip_buf[UIP_LLH_LEN])

#define DEBUG 1

//Send functions
PT_THREAD(sendDemoPage(struct httpd_state *s));

/*-----------------------------------------------------------*/
/* NOTE: Do not run vuIP_Task, when running vuWeb_Task */
void vuWeb_Task( void *pvParameters )
{
	portBASE_TYPE i;
	uip_ipaddr_t xIPAddr;
	struct timer periodic_timer, arp_timer;
	u16_t ripaddr[2];

	/* Initialise the uIP stack. */
	timer_set( &periodic_timer, configTICK_RATE_HZ / 2 );
	timer_set( &arp_timer, configTICK_RATE_HZ * 10 );
	uip_init();
	uip_ipaddr( xIPAddr, uipIP_ADDR0, uipIP_ADDR1, uipIP_ADDR2, uipIP_ADDR3 );
	uip_sethostaddr( xIPAddr );
	httpd_init();
	
	//Connect to TUIO
	uip_ipaddr(ripaddr,192,168,0,3);
	tuio_conn = uip_connect(ripaddr, htons(3000));
	//Retrieve connection details
	tuio_lport = tuio_conn->lport;
	tuio_rport = tuio_conn->rport;

	web_debug = 0;

	/* Initialise the MAC. */
	do
    {
		vTaskDelay( uipINIT_WAIT );
		xEMACSemaphore = xEMACInit();        
    } while( xEMACSemaphore == NULL );

	for( ;; )
	{
		// Ensure TUIO connection stays up after serving web requests
		tuioConnectionHold();
	
		/* Is there received data ready to be processed? */
		uip_len = ulEMACPoll();
		
		if( uip_len > 0 )
		{
			/* Standard uIP loop taken from the uIP manual. */
			if( xHeader->type == htons( UIP_ETHTYPE_IP ) )
			{
				uip_arp_ipin();
				uip_input();

				if (web_debug) {debug_printf("*");}

				/* If the above function invocation resulted in data that 
				should be sent out on the network, the global variable 
				uip_len is set to a value > 0. */
				if( uip_len > 0 )
				{
					uip_arp_out();
					lEMACSend();
				}
			}
			else if( xHeader->type == htons( UIP_ETHTYPE_ARP ) )
			{
				uip_arp_arpin();

				/* If the above function invocation resulted in data that 
				should be sent out on the network, the global variable 
				uip_len is set to a value > 0. */
				if( uip_len > 0 )
				{
					lEMACSend();
				}
			}
		}
		else
		{
			if( timer_expired( &periodic_timer ) )
			{
				timer_reset( &periodic_timer );
				for( i = 0; i < UIP_CONNS; i++ )
				{
					uip_periodic( i );
	
					/* If the above function invocation resulted in data that 
					should be sent out on the network, the global variable 
					uip_len is set to a value > 0. */
					if( uip_len > 0 )
					{
						uip_arp_out();
						lEMACSend();
					}
				}	
	
				/* Call the ARP timer function every 10 seconds. */
				if( timer_expired( &arp_timer ) )
				{
					timer_reset( &arp_timer );
					uip_arp_timer();
				}
			}
			else
			{			
				/* We did not receive a packet, and there was no periodic
				processing to perform.  Block for a fixed period.  If a packet
				is received during this period we will be woken by the ISR
				giving us the Semaphore. */
				xSemaphoreTake( xEMACSemaphore, configTICK_RATE_HZ / 2 );			
			}
		}
		
		// Ensure TUIO connection stays up after serving web requests
		//tuioConnectionHold();
	}
}
/*-----------------------------------------------------------------------------------*/
/* UIP_APPCALL override. Decide what to do with accepted incoming packets */
void web_task_appcall( void )
{
	unsigned char *h;
	unsigned char *f;
	struct uip_conn *p;
	struct httpd_state *s;
	
	char urlBuff[HTTPVARQUEUE_WIDTH];
	uint8_t *pchar;
	uint8_t ppos, ppos2;
	uint8_t isAjaxCommand;
	int i, len;
	
	h = (unsigned char *) uip_appdata;
	p = (struct uip_conn *) uip_conn;
	/* (struct uip_conn *) tuio_conn; */
	s = (struct httpd_state *)(&(uip_conn->appstate));
	f = s->filename;
	
	// Ensure TUIO connection stays up after serving web requests
	tuioConnectionHold();

	//Filter packet types
	if (BUF->destport==tuio_lport && tuio_lport==uip_conn->lport) {
		
		if (web_debug) {
		//TUIO packet
		debug_printf("  => TUIO [%d][%d][%d]\r\n", 
			xTaskGetTickCount()/1000, tuio_conn->tcpstateflags, uip_flags);
		}
		
		//Proces tuio data
		tuioclient_appcall();

	} else if (BUF->srcport==tuio_lport && BUF->destport==tuio_rport) {
		
		if (web_debug) {
		//SYN/ACK packet
		debug_printf("  => ACK [%d][%d][%d]\r\n", 
			xTaskGetTickCount()/1000, tuio_conn->tcpstateflags, uip_flags);
		}
		
	} else {
	
			//other - assume HTTP
			
			/*	//don't make new psock connections. overhead; blocks tuio
				PSOCK_INIT(&s->sin, NULL, 0);
				PSOCK_BEGIN(&s->sin);
				PSOCK_SEND_STR(&s->sin,"HTTP/1.0 200 OK\r\n"\
										"Connection: close\r\n"\
										"Content-Type: text/html\r\n"\
										"\r\n"\
										"Hello World, From a simple httpd.");
				PSOCK_CLOSE(&s->sin);
				PSOCK_END(&s->sin);
			*/

			//Special cases: send custom strings instead of pages
			if (strstr(h, "helloworld")!=NULL ) {
				if (web_debug) {
					debug_printf("helloworld [%d]\r\n", uip_flags);
				}
  				if (uip_newdata() || uip_connected() || uip_poll() || \
  				    uip_rexmit()) {
  					len = snprintf(h, uip_mss()-1, "HTTP/1.0 200 OK\r\n"\
  									"Content-Type: text/html\r\n"\
  									"Connection: close\r\n"\
  									"\r\n"\
  									"Hello World, From a simple httpd packet server.");
  					uip_send(h,len);
  					ack_wait = 1;
  				}
			} else if (strstr(h, "jsonfetch")!=NULL) {
				if (web_debug) {
					debug_printf("jsonfetch [%d]\r\n", uip_flags);
				}
  				if (uip_newdata() || uip_connected() || uip_poll() || \
  				    uip_rexmit()) {
  					len = snprintf(h, uip_mss()-1, "HTTP/1.0 200 OK\r\n"\
  									"Content-Type: text/html\r\n"\
  									"Connection: close\r\n"\
  									"\r\n"\
  									"%s", jsonStr);
  					uip_send(h,len);
  					ack_wait = 1;
  				}
			} else {
			
				isAjaxCommand = 0;
			
				//determine if url contains GET data
				if (h[0]=='G'&&h[1]=='E'&&h[2]=='T'&&h[3]==' ') {
					// -find pos of first ?, searching htext
					ppos = 4;
					while (h[ppos] != '?' && ppos < 50) {
						ppos++;
					}
					// -find pos of first ' ', searching htext
					ppos2 =4;
					while (h[ppos2] != ' ' && ppos2 < 50) {
						ppos2++;
					}
					//decide if valid url formatting
					if (ppos<50 && ppos<ppos2) {
						isAjaxCommand = 1;
					} else {
						isAjaxCommand = 0;
					}
				}
		
				//if control signals found - push url into queue, back to main
				if (isAjaxCommand) {
					//copy to sending buffer
					for (i=0; i<HTTPVARQUEUE_WIDTH; i++) {
						urlBuff[i] = 0;
					}
					memcpy(urlBuff, &(h[ppos]), ppos2-ppos);
					//send
					xQueueSendToBack(xHttpVars_Queue, urlBuff, 0);
				}
		
				if (web_debug) {
					debug_printf(" => HTTP [%d][%d][%d] %.15s\r\n", 
					xTaskGetTickCount()/1000, tuio_conn->tcpstateflags, 
					uip_flags, h);
				}
		
				/*if (web_debug) {
					debug_printf(" => HTTP %.15s\r\n", h);
				}*/
		
				if (web_debug && isAjaxCommand) {
					debug_printf("%d > \"%.*s\"\r\n", ppos, 
						ppos2-ppos, &(h[ppos]) );
				 }
				 
				 
				 //Serve http request
				 if (uip_acked() && ack_wait) {
  					uip_close();
  					ack_wait = 0;
  				 } else {
  				 	httpd_appcall();
  				 }

				
				//httpd_appcall();
			
			}
	}
	
}
/*-----------------------------------------------------------*/

PT_THREAD(sendDemoPage(struct httpd_state *s)) {
	PSOCK_BEGIN(&s->sout);
	PSOCK_SEND_STR(&s->sout,"HTTP/1.0 200 OK\r\n"\
						  	"Connection: close\r\n"\
						  	"Content-Type: text/plain\r\n"\
						  	"\r\n"\
						  	"Hello World, From a simple httpd.");
	PSOCK_END(&s->sout);
}
		
/*-----------------------------------------------------------*/		

unsigned char tuioConnectionHold( void )
{
	/* Ensure the TUIO connection is still up. Return status */
	
	u16_t ripaddr[2];
	
	if (tuio_conn->tcpstateflags == UIP_TIME_WAIT){// || \
		//tuio_conn->tcpstateflags == UIP_SYN_SENT) {
		tuio_conn->tcpstateflags = UIP_ESTABLISHED;
	}
	
	if (tuio_conn->tcpstateflags == UIP_CLOSED) {
		uip_ipaddr(ripaddr,192,168,0,3);
		tuio_conn = uip_connect(ripaddr, htons(3000));
		tuio_lport = tuio_conn->lport;
		tuio_rport = tuio_conn->rport;
	}
	
	/*
	// If the http application has taken the current connection, give it back
	// to the tuio connection
	if (tuio_conn->tcpstateflags == UIP_TIME_WAIT) {
		tuio_conn->tcpstateflags = UIP_ESTABLISHED;
	}
	
	// If tuio has disconnected, reconnect
	if (tuio_conn->tcpstateflags == UIP_CLOSED) {
		uip_ipaddr(ripaddr,192,168,0,3);
		tuio_conn = uip_connect(ripaddr, htons(3000));
		//Retrieve connection details
		tuio_lport = tuio_conn->lport;
		tuio_rport = tuio_conn->rport;
	}
	*/
	
	return (unsigned char)(tuio_conn->tcpstateflags);
}

/*-----------------------------------------------------------*/

unsigned char tuioConnectionState( void )
{
	/* Return connectionstatus flag */
	
	/* The TCP states used in the uip_conn->tcpstateflags are:
	#define UIP_CLOSED      0
	#define UIP_SYN_RCVD    1
	#define UIP_SYN_SENT    2
	#define UIP_ESTABLISHED 3
	#define UIP_FIN_WAIT_1  4
	#define UIP_FIN_WAIT_2  5
	#define UIP_CLOSING     6
	#define UIP_TIME_WAIT   7
	#define UIP_LAST_ACK    8
	#define UIP_TS_MASK     15
	  
	#define UIP_STOPPED      16
	*/
	
	return (unsigned char)(tuio_conn->tcpstateflags);
}

/*-----------------------------------------------------------*/

void clock_init(void)
{
	/* This is done when the scheduler starts. */
}
/*-----------------------------------------------------------*/

clock_time_t clock_time( void )
{
	return xTaskGetTickCount();
}
/*-----------------------------------------------------------*/

void vProcessInput( char *pcInput )
{
char *c;

	/* Turn the LED on or off depending on the checkbox status. */

	c = strstr( pcInput, "?" );
	if( c )
	{
		if( strstr( c, "LED0=1" ) != NULL )
		{
			vParTestSetLED( 3, 0 );
		}
		else
		{
			vParTestSetLED( 3, 1 );
		}		
	}
}





