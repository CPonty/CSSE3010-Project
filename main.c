/* -----------------------------------------------------------------------------
# Company name: CSSE3010 - Embedded Systems Design & Interfacing
# Project name: Blimp radio example
# Module name : Main 
# Functionality: Provides basic structure for sending/receiving packets.
# Hardware platform: NetduinoPlus (AT91SAM7)
#
# Author name: Chris Rieger
# Creation date: 20/09/2012
# Revision date (name): -
# Changes implemented (date): -
#(Comments): 
------------------------------------------------------------------------------*/

/* Standard includes. */
#include <stdlib.h>
#include <stdio.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "board.h"
#include "queue.h"

/* Application includes. */
#include "USB-CDC.h"
//#include "uIP_Task.h"
#include "BlockQ.h"
#include "blocktim.h"
#include "flash.h"
#include "QPeek.h"
#include "dynamic.h"
#include "debug_printf.h"
#include "FreeRTOS_CLI.h"

/* Files */
#include "ff.h"
#include "diskio.h"

#define FNAME_SIZE  32
FATFS Fatfs;		/* File system object */
FIL Fil;			/* File object */
TCHAR * logFName;	/* File name-string */

/* Shared includes */
#include "json-data.h"
char *jsonStr = NULL;

/* Tasks */
#include "web_Task.h"

/* Libraries */
#include <aic/aic.h>
#include <pio/pio.h>
#include <pmc/pmc.h>
#include <pwmc/pwmc.h>
#include <utility/led.h>
#include <utility/trace.h>

/* Local Library Include */
#include "nrf24l01plus.h"

/* Math macros */
#define BLIMPSIDE(x) ((x)%12<6?0:1)
#define ABS(x) ((x)<0?(-(x)):(x))

/* Priorities for tasks. */
#define mainUSB_PRIORITY					( tskIDLE_PRIORITY + 1 )
#define mainUIP_PRIORITY					( tskIDLE_PRIORITY + 1 )
#define mainUSBCLI_PRIORITY					( tskIDLE_PRIORITY + 1 )
#define mainRadio_PRIORITY					( tskIDLE_PRIORITY + 1 )
#define mainWEB_PRIORITY					( tskIDLE_PRIORITY + 1 )
#define mainCONTROL_PRIORITY				( tskIDLE_PRIORITY + 1 )

/* Stack sizes for tasks. */
#define mainUSB_TASK_STACK					( 200 )
#define mainUSBCLI_TASK_STACK_SIZE			( 400 )
#define mainRadio_TASK_STACK_SIZE			( 400 )
#define mainWEB_TASK_STACK_SIZE				( 200 )//probably increase later
#define mainCONTROL_TASK_STACK_SIZE			( 400 )

/* The rate at which the idle hook sends data to the USB port. */
#define mainUSB_TX_FREQUENCY		( 200 / portTICK_RATE_MS )

/* The LED toggle by the tick hook should an error have been found in a task. */
#define mainERROR_LED						( 3 )

// Custom pins
#define PIN_RESET {1 << 30, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_OUTPUT_0, PIO_DEGLITCH | PIO_PULLUP}
#define PIN_LASER {1 << 19, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_OUTPUT_0, PIO_DEFAULT}

// pin instances
static const Pin pinLaser = PIN_LASER;
static const Pin resetPin = PIN_RESET;

// Pio pins to configure.
static const Pin pins[] = {
	PIN_LASER,
	PIN_PWMC_PWM2,    // PORTB 1<<21  IO9
    PIN_PWMC_PWM3,    // PORTB 1<<22  IO10
    PIN_RESET        // PORTA 1<<30 /RESET
};

// Radio pins
static const Pin pinsRF[] = {nrf24l01plus_MISO, nrf24l01plus_MOSI, nrf24l01plus_SCK, nrf24l01plus_CE, nrf24l01plus_CSN};

// SERVO PWM
#define CHANNEL_SERVO1   2    // X axis, digital i/o pin 9
#define CHANNEL_SERVO2   3    // Y axis, digital i/o pin 10

#define SERVO_PWM_FREQUENCY         50
#define SERVO_DUTY_CYCLE            2000
#define DEG_THRESHOLD               1 // deadzone, degrees
#define SERVO_X_CENTRE              160
#define SERVO_X_SPREAD              85
#define SERVO_Y_CENTRE              90
#define SERVO_Y_SPREAD              28
#define SERVO_X_LEFT 	(SERVO_X_CENTRE-SERVO_X_SPREAD)
#define SERVO_X_RIGHT 	(SERVO_X_CENTRE+SERVO_X_SPREAD)
#define SERVO_Y_LEFT 	(SERVO_Y_CENTRE-SERVO_Y_SPREAD)
#define SERVO_Y_RIGHT 	(SERVO_Y_CENTRE+SERVO_Y_SPREAD)
#define SERVO_X_RANGE	(SERVO_X_RIGHT-SERVO_X_LEFT)
#define SERVO_Y_RANGE	(SERVO_Y_RIGHT-SERVO_Y_LEFT)

// FIDUCIAL CONTROL
#define IMAGE_DEADZONE_THRESHOLD	10 // percentage of screen size fiducial can
									   // move off-centre before it is followed

// Radio structs
struct radio_tx {
	//
	uint8_t blimpid;
	uint8_t motor_active;
	uint8_t motor_power;
	uint8_t motor_dir;
	uint8_t altitude;
	uint8_t command;
	uint16_t command_data;
	uint8_t filler[24];
	//
}__attribute__((packed)); 

struct radio_rx {
	//
	uint8_t blimpid;
	uint8_t motor_active;
	uint8_t motor_power;
	uint8_t motor_dir;
	uint8_t altitude;
	uint8_t command;
	uint16_t command_data;
	uint16_t distance;
	uint16_t magnetometer;
	uint8_t filler[20];
	//
}__attribute__((packed));

unsigned char tx_payload[TX_PLOAD_WIDTH]; // TX Preparation payload
	unsigned char tx_buf[TX_PLOAD_WIDTH]; // TX queue->buffer->radio 
	unsigned char rx_buf[TX_PLOAD_WIDTH]; // RX radio->queue->buffer	

struct radio_tx *pkt_tx = (struct radio_tx*)tx_payload; //pkt_tx
struct radio_rx *pkt_rx = (struct radio_rx*)rx_buf;     //pkt_rx

#define RADIO_DELAY 1000 //1000

#define HTTP_MODE 1
#define  CLI_MODE 2

/* Utility declarations */
void pkt_tx_set_defaults(void);
void die(FRESULT rc);
DWORD get_fattime(void);
void init_Pwm(void);

/* Basic Declarations */
static void prvSetupHardware( void );
void vApplicationIdleHook( void );

// Tasks
void vRadio_task(void *pvParameters);
void vControl_task(void *pvParameters);

// CLI shared variables
long lParam_len; 
char *cCmd_string;
uint8_t noCommand;

// Globals
int rf_debug = 0;
int fiducial_debug = 0;
int control_mode = HTTP_MODE;
int logging = 0;
int pan, tilt;
int autopan = 0;

#define TASKTEXTBUFSIZE 1100
char * bigTextBuf;

/*-----------------------------------------------------------*/
// CLI
/*-----------------------------------------------------------*/
static portBASE_TYPE prvLaserCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE prvRadioCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE prvDebugCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE prvModeCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE prvBlimpCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE prvCommandCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE prvLsCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE prvTasksCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE prvLogCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE prvRemoveCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE prvPanTiltCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );

/* Structure that defines the "laser" command line command. */
static const xCommandLineInput xLaser =
{
	( char * ) "laser",
	( char * ) "laser: Turn the Laser on/off\r\n",
	prvLaserCommand,
	1
};

/* Structure that defines the "radio" command line command. */
static const xCommandLineInput xRadio =
{
	( char * ) "rf",
	( char * ) "rf: send a file over radio.\r\n",
	prvRadioCommand,
	1
};

/* Structure that defines the "debug" command line command. */
static const xCommandLineInput xDebug =
{
	( char * ) "debug",
	( char * ) "debug: rf web tuio fiducial\r\n",
	prvDebugCommand,
	1
};

/* Structure that defines the "mode" command line command. */
static const xCommandLineInput xMode =
{
	( char * ) "mode",
	( char * ) "mode: switch http/cli motor commands\r\n",
	prvModeCommand,
	0
};

/* Structure that defines the "blimp" command line command. */
static const xCommandLineInput xBlimp =
{
	( char * ) "blimp",
	( char * ) "blimp: set RF channel to 2*blimp id\r\n",
	prvBlimpCommand,
	1
};

/* Structure that defines the "command" command line command. */
static const xCommandLineInput xCommand =
{
	( char * ) "command",
	( char * ) "command: set special command for blimps\r\n",
	prvCommandCommand,
	1
};

/* Structure that defines the "ls" command line command. */
static const xCommandLineInput xLs =
{
	( char * ) "ls",
	( char * ) "ls: print dir structure\r\n",
	prvLsCommand,
	0
};

/* Structure that defines the "tasks" command line command. */
static const xCommandLineInput xTasks =
{
	( char * ) "tasks",
	( char * ) "tasks: print status of scheduler tasks\r\n",
	prvTasksCommand,
	0
};

/* Structure that defines the "log" command line command. */
static const xCommandLineInput xLog =
{
	( char * ) "log",
	( char * ) "log: start/stop writing to logfile\r\n",
	prvLogCommand,
	1
};

/* Structure that defines the "remove" command line command. */
static const xCommandLineInput xRemove =
{
	( char * ) "rm",
	( char * ) "rm: delete file\r\n",
	prvRemoveCommand,
	1
};

/* Structure that defines the "pan-tilt" command line command. */
static const xCommandLineInput xPantilt =
{
	( char * ) "pt",
	( char * ) "pt: pan tilt\r\n",
	prvPanTiltCommand,
	2
};

/* USB CLI receive task */
void vUSB_cli_receive_task(void *pvParameters);
/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*/
// Starts all the other tasks, then starts the scheduler.
/*-----------------------------------------------------------*/
int main( void )
{
	int i;

	/* Setup any hardware that has not already been configured by the low
	level init routines. */
	prvSetupHardware();
	
	/* Create the USB CDC Serial task - used for debug-printf. */
	xTaskCreate( vUSBCDCTask, ( signed char * ) "USB", mainUSB_TASK_STACK, NULL, mainUSB_PRIORITY, NULL );
	
	/* Register Echo and Test CLI commands */
	FreeRTOS_CLIRegisterCommand(&xLaser);
	FreeRTOS_CLIRegisterCommand(&xRadio);
	FreeRTOS_CLIRegisterCommand(&xDebug);
	FreeRTOS_CLIRegisterCommand(&xMode);
	FreeRTOS_CLIRegisterCommand(&xBlimp);
	FreeRTOS_CLIRegisterCommand(&xCommand);
	FreeRTOS_CLIRegisterCommand(&xLs);
	FreeRTOS_CLIRegisterCommand(&xTasks);
	FreeRTOS_CLIRegisterCommand(&xLog);
	FreeRTOS_CLIRegisterCommand(&xRemove);
	FreeRTOS_CLIRegisterCommand(&xPantilt);
	
	// Configure io pins
    PIO_Configure(pins, PIO_LISTSIZE(pins));
    PIO_Set(&pinLaser);
    init_Pwm();
    
    // Allocate heap string buffers
    logFName = (TCHAR *)pvPortMalloc(sizeof(TCHAR)*FNAME_SIZE);
    bigTextBuf = (char *)pvPortMalloc(sizeof(char)*TASKTEXTBUFSIZE);
    jsonStr = (char *)pvPortMalloc(sizeof(char)*JSON_BUFSIZE);
    jsonStr[0]='{'; jsonStr[1]='}'; jsonStr[2]='\0';
    
    // Ready the filesystem; create the logfile
    // mount SD under fatFS
    f_mount(0, &Fatfs);
	
	// Create Queues.
	xRadioRX_Queue = xQueueCreate(5, sizeof(char)*32);
	xRadioTX_Queue = xQueueCreate(5, sizeof(char)*32);
	xHttpVars_Queue = xQueueCreate(2, sizeof(char)*HTTPVARQUEUE_WIDTH);
	xReactivisionQueue = xQueueCreate(1, sizeof(int)*10);
	
	// USB+CLI
    xTaskCreate( vUSB_cli_receive_task, "USBCLI", mainUSBCLI_TASK_STACK_SIZE, NULL, mainUSBCLI_PRIORITY, NULL );	
	// Radio task
	xTaskCreate(vRadio_task, "Radio", mainRadio_TASK_STACK_SIZE, NULL, mainRadio_PRIORITY, NULL );
	// Web Server
	xTaskCreate( vuWeb_Task, "WEB", mainWEB_TASK_STACK_SIZE, NULL, mainWEB_PRIORITY, NULL );
	// Command+Control
	xTaskCreate( vControl_task, "CONTROL", mainCONTROL_TASK_STACK_SIZE, NULL, mainCONTROL_PRIORITY, NULL );
	
	/* Start the scheduler.
	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used here. */
	
	vTaskStartScheduler();
	
	/* We should never get here as control is now taken by the scheduler. */
  	return 0;
}

/* Only purpose is to interface with the radio */
void vRadio_task(void *pvParameters)
{
    int i, xTimestamp;
    uint8_t any_packet_sent;
	
    // Configure RF pins
    PIO_Configure(pinsRF, PIO_LISTSIZE(pinsRF));
	
	// Start Radio
	nrf24l01plus_init();
	
	// Clean the tx,rx packets
	for(i = 0; i < 32; i ++) {
		tx_payload[i] = 0; //clean
		rx_buf[i] = 0; //clean
	}
	
	// Default blimp
	global_blimpid = 9;
	
	// Set the tx packet with defaults
	pkt_tx_set_defaults();
	
	vTaskDelay(100 / portTICK_RATE_MS);
	
	debug_printf("\r\nWelcome to Project 3 \r\n");
	
	any_packet_sent = 0;
	xTimestamp = xTaskGetTickCount();
	
	for (;;) {
        nrf24l01plus_receive_packet();
		
        if (xQueueReceive(xRadioTX_Queue, tx_buf, 1) == pdPASS) {	 //Process received value
        	any_packet_sent = 1;
			nrf24l01plus_mode_tx_send(tx_buf);
			while(!nrf24l01plus_Send_Packet(tx_buf));
		    nrf24l01plus_mode_rx();
		    if (rf_debug) {
		    	debug_printf(">");
		    }
		}
		if (xQueueReceive(xRadioRX_Queue, rx_buf, 1) == pdPASS) {	 //Process received value 
		   if (pkt_rx->blimpid == global_blimpid) {
			   if(rf_debug) { // debug
					debug_printf("Rx: id=%u on=%u pwr=%u dir=%u alt=%u dist=%u mag=%u com=%x resp=%u\r\n",
						pkt_rx->blimpid, pkt_rx->motor_active, 
						pkt_rx->motor_power, pkt_rx->motor_dir, 
						pkt_rx->altitude, pkt_rx->distance,
						pkt_rx->magnetometer,
						pkt_rx->command,
						pkt_rx->command_data);
				}
				//
				// Deal with rx'd data directly here
				//
			}
		}
		
		// Regular resend
		if (xTaskGetTickCount()-xTimestamp > RADIO_DELAY) {
			xTimestamp = xTaskGetTickCount();
			if (any_packet_sent) {
				xQueueSendToBack(xRadioTX_Queue, tx_payload, 0);
			}
		}
		
		vTaskDelay(10 / portTICK_RATE_MS); 
	}
}
/*
// Radio structs
struct radio_tx {
	//
	uint8_t blimpid;
	uint8_t motor_active;
	uint8_t motor_power;
	uint8_t motor_dir;
	uint8_t altitude;
	uint8_t command;
	uint16_t command_data;
	uint8_t filler[24];
	//
}__attribute__((packed)); 

struct radio_rx {
	//
	uint8_t blimpid;
	uint8_t motor_active;
	uint8_t motor_power;
	uint8_t motor_dir;
	uint8_t altitude;
	uint8_t command;
	uint16_t command_data;
	uint16_t distance;
	uint16_t magnetometer;
	uint8_t filler[20];
	//
}__attribute__((packed));

unsigned char tx_payload[TX_PLOAD_WIDTH]; // TX Preparation payload
	unsigned char tx_buf[TX_PLOAD_WIDTH]; // TX queue->buffer->radio 
	unsigned char rx_buf[TX_PLOAD_WIDTH]; // RX radio->queue->buffer	

struct radio_tx *pkt_tx = (struct radio_tx*)tx_payload; //pkt_tx
struct radio_rx *pkt_rx = (struct radio_rx*)rx_buf;     //pkt_rx
*/

/*-----------------------------------------------------------*/
/* Utility functions */

void pkt_tx_set_defaults(void)
{
	// Set the contents of the tx packet to default values
	pkt_tx->blimpid = global_blimpid;
	pkt_tx->motor_active = 0;
	pkt_tx->motor_power = 127;
	pkt_tx->motor_dir = 127;
	pkt_tx->altitude = 127;
	pkt_tx->command = 0;
	pkt_tx->command_data = 5493; //studentID
}

// FatFS error notification / task blocker
void die(FRESULT rc)
{
	debug_printf("Failed with rc=%u.\n", rc);
	for (;;);
}


// User Provided Timestamp Function for FatFs
DWORD get_fattime(void)
{
	return	  ((DWORD)(2012 - 1980) << 25)	/* Year = 2012 */
			| ((DWORD)10 << 21)				/* Month = 10 */
			| ((DWORD)25 << 16)				/* Day_m = 25*/
			| ((DWORD)0 << 11)				/* Hour = 0 */
			| ((DWORD)0 << 5)				/* Min = 0 */
			| ((DWORD)0 >> 1);				/* Sec = 0 */
}

// Initialise PWM
void init_Pwm(void)
{
    // Enable PWMC peripheral clock
    AT91C_BASE_PMC->PMC_PCER = 1 << AT91C_ID_PWMC;

    // Set clock A for servo's at 50hz * Duty cycle (samples).
    PWMC_ConfigureClocks(2 * SERVO_PWM_FREQUENCY * SERVO_DUTY_CYCLE, 0, BOARD_MCK);

    // Configure PWMC channel for SERVO (center-aligned, inverted polarity)
    PWMC_ConfigureChannel(CHANNEL_SERVO1, AT91C_PWMC_CPRE_MCKA, AT91C_PWMC_CALG, AT91C_PWMC_CPOL);
    PWMC_SetPeriod(CHANNEL_SERVO1, SERVO_DUTY_CYCLE);
    PWMC_SetDutyCycle(CHANNEL_SERVO1, SERVO_X_CENTRE);
    
    PWMC_ConfigureChannel(CHANNEL_SERVO2, AT91C_PWMC_CPRE_MCKA, AT91C_PWMC_CALG, AT91C_PWMC_CPOL);
    PWMC_SetPeriod(CHANNEL_SERVO2, SERVO_DUTY_CYCLE);
    PWMC_SetDutyCycle(CHANNEL_SERVO2, SERVO_Y_CENTRE);
    
    // Enable channel #1 and #2
    PWMC_EnableChannel(CHANNEL_SERVO1);
    PWMC_EnableChannel(CHANNEL_SERVO2);
}

// Set PWM Channel duty cycle
void pwm_set_duty(unsigned char channel, unsigned short duty)
{
	// abstraction of PWMC_SetDutyCycle with channel disable/enable added
    PWMC_DisableChannel(channel);
    PWMC_SetDutyCycle(channel, duty);
    PWMC_EnableChannel(channel);
}

// Apply pan/tilt angles to servo
void servo_set_pan_tilt(short pan, short tilt)
{
	pwm_set_duty(CHANNEL_SERVO1, pan);
	pwm_set_duty(CHANNEL_SERVO2, tilt);
}

// Limit pan/tilt values
short panLimits(short pan)
{
	if (pan < SERVO_X_LEFT)  { return SERVO_X_LEFT; }
	if (pan > SERVO_X_RIGHT) { return SERVO_X_RIGHT; }
	return pan;
}
short tiltLimits(short tilt)
{
	if (tilt < SERVO_Y_LEFT)  { return SERVO_Y_LEFT; }
	if (tilt > SERVO_Y_RIGHT) { return SERVO_Y_RIGHT; }
	return tilt;
}

// Get the induced servo motion from a location seen by the camera (either axis)
short axisGetMotion(short position)
{
	return (position-50)/IMAGE_DEADZONE_THRESHOLD;
}

// String to int. Avoiding atoi, was causing problems.
int str2int(char * string, long maxChars)
{
	int x, i;
	if (!isdigit(string[0])) { return -1; }
	x = string[0]-'0';
	// process remaining characters after the first:
	for (i=1; i<maxChars; i++) {
		if (!isdigit(string[i])) { break; }
		// iterating left->right: x = 10x + digit
		x = (10*x) + (string[i]-'0'); 
	}
	return x;
}

/*-----------------------------------------------------------*/
/* Command & Control task */
void vControl_task(void *pvParameters)
{
	//Time
	portTickType xTimestamp = 0;
	portTickType xTimestamp2 = 0;
	int hours, minutes, seconds, mseconds;
	
	//http
	char cbuff[HTTPVARQUEUE_WIDTH];
	uint8_t urlcommand_valid;
	
	//file
	FRESULT rc;
	UINT bytesWritten;
	char rowData[100];
	char fileMode = 0;
	
	//json
	uint16_t nchars;
	
	//reactivision
	//portBASE_TYPE xStatus;
	int usable[4], usableCount;
	unsigned int tuioBuf[10];
	int i, pixelDistance;
	int idbuff, matchedId, oldestId, new1, new2, id[4];
	int distance;
	float floatx, floaty, floata; //range 0..1, 0..1, 0..2PI
	short x1, y1, x[4], y[4];
	short xcentre, ycentre;
	short abuff, rotation[4];
	portTickType xTime, xOldestTimestamp, xNewestTimestamp, xBlimpTime[4];
	
	//servos
	//int pan, tilt;
	short moveX, moveY;
	
	//reset stored fiducial data
	distance = 1000;
	xcentre = 0;
	ycentre = 0;
	for (i=0; i<4; i++) {
		id[i]=-1;
		rotation[i]=0;
		x[i]=0;
		y[i]=0;
		xBlimpTime[i] = 0;
	}
	
	//centre servos
	pan =  SERVO_X_CENTRE;
	tilt = SERVO_Y_CENTRE;
	servo_set_pan_tilt(pan, tilt);
	autopan = 0;
	
	for (;;) {
	
		//Read & process TUIO data
		if (xQueueReceive(xReactivisionQueue, tuioBuf, 1) == pdPASS) {
			/*
			//Set TUIO attributes. See TUIO 1.1 Specification, Table 1.
			session_id		= (int) tuio_attribute[0];			 
			class_id 		= (int) tuio_attribute[1];
			position_x 		= *(float *)&tuio_attribute[2]; 	//range 0...1
			position_y 		= *(float *)&tuio_attribute[3]; 	//range 0...1
			angle_a 		= *(float *)&tuio_attribute[4]; 	//range 0..2PI
        	*/
        	//extract properties
        	xTime = xTaskGetTickCount();
            idbuff = *(int *)(&tuioBuf[1]);
            floatx= *(float *)(&tuioBuf[2]);
            floaty= *(float *)(&tuioBuf[3]);
            floata= *(float *)(&tuioBuf[4]);
            x1 = 100-(int)(floatx*100.0f);
            y1 = 100-(int)(floaty*100.0f);
            abuff = (int)(floata*57.3f); // 180/pi, scaling correct to 2 places
            //find if blimp ID exists
            matchedId = -1;
            for (i=0; i<4; i++) {
            	if (id[i]==idbuff) {
            		matchedId = i;
            		break;
            	}
            }
            //find oldest listed fiducial
            oldestId = -1;
            xOldestTimestamp = xTime+1;
            for (i=0; i<4; i++) {
            	if (xBlimpTime[i]<xOldestTimestamp) {
            		xOldestTimestamp = xBlimpTime[i];
            		oldestId = i;
            	}
            }
            //find two newest fiducials
            new1=-1;
            new2=-1;
            xNewestTimestamp=0;
            for (i=0; i<4; i++) {
            	if (xBlimpTime[i]>xNewestTimestamp) {
            		xNewestTimestamp = xBlimpTime[i];
            		new1 = i;
            	}
            }
            xNewestTimestamp=0;
            for (i=0; i<4; i++) {
            	if (i==new1) continue;
            	if (xBlimpTime[i]>xNewestTimestamp) {
            		xNewestTimestamp = xBlimpTime[i];
            		new2 = i;
            	}
            }
            //if blimp id exist, replace it, else replace id of oldest fiducial.
            if (matchedId != -1) {
            	id[matchedId]=idbuff;
				rotation[matchedId]=abuff;
				x[matchedId]=x1;
				y[matchedId]=y1;
				xBlimpTime[matchedId] = xTime;
            } else {
				id[oldestId]=idbuff;
				rotation[oldestId]=abuff;
				x[oldestId]=x1;
				y[oldestId]=y1;
				xBlimpTime[oldestId] = xTime;
            }
            //decide which markers are usable
            usableCount=0;
        	for (i=0; i<4; i++) {
        		if (id[i]!=-1 && (xTime-xBlimpTime[i])<2000) {
        			usable[i]=1;	
        			usableCount++;
        		} else {
        			usable[i]=0;
        		}
        	}
            
            //recompute distance
            if (BLIMPSIDE(new1)==BLIMPSIDE(new2)) {
           		pixelDistance = ABS(x[new1]-x[new2])+ABS(y[new1]-y[new2]);
           		distance = (int)((0.7*(65.0/100.0)/(pixelDistance/100.0))*1000.0f);
           		distance /= ABS(id[new1]-id[new2]);
            }
            
            //recompute midpoint
            xcentre=0;
            ycentre=0;
            for (i=0; i<4; i++) {
            	if (usable[i]) {
            		xcentre += x[i];
            		ycentre += y[i];
            	}
            }
            xcentre /= usableCount;
            ycentre /= usableCount;
            
            if (autopan) {
		        //translate fiducial position to motion
		    	moveX = axisGetMotion(xcentre);
		    	moveY = axisGetMotion(ycentre);
		    	//translate motion to pan/tilt
		    	pan  = panLimits(pan  + moveX);
		    	tilt = tiltLimits(tilt + moveY);
		    	servo_set_pan_tilt(pan, tilt);
        	}
            
            if (fiducial_debug) {
            	//debug_printf("<fid %d>", idbuff);
            	//vUSBSendByte('x');
            }
		}
		
		//Read & process AJAX data
		if (xQueueReceive(xHttpVars_Queue, cbuff, 1) == pdPASS) {
			urlcommand_valid = 0;
			/* Expected format:
				?a=xxx&b=xxx&c=xxx&d=xxx
				a: motor_active
				b: motor_power
				c: motor_dir
				d: altitude
			*/
			if (control_mode==HTTP_MODE) {
				if (strlen(cbuff)==24) {
					// Test format matches expected
					if (cbuff[0]=='?' && cbuff[2]=='=' && cbuff[6]=='&' && \
						cbuff[8]=='=' && cbuff[12]=='&'&& cbuff[14]=='='&& \
						cbuff[18]=='&'&& cbuff[20]=='='&& \
						cbuff[3]=='0'      && cbuff[4]=='0'      && isdigit(cbuff[ 5]) && \
						isdigit(cbuff[ 9]) && isdigit(cbuff[10]) && isdigit(cbuff[11])&& \
						isdigit(cbuff[15]) && isdigit(cbuff[16]) && isdigit(cbuff[17])&& \
						isdigit(cbuff[21]) && isdigit(cbuff[22]) && isdigit(cbuff[23])) {
						
						// Valid command! Read values into pkt_tx
						urlcommand_valid = 1;
						pkt_tx->motor_active = cbuff[5]-'0';
						pkt_tx->motor_power = 100*(cbuff[9]-'0')+10*(cbuff[10]-'0')+(cbuff[11]-'0');
						pkt_tx->motor_dir = 100*(cbuff[15]-'0')+10*(cbuff[16]-'0')+(cbuff[17]-'0');
						pkt_tx->altitude = 100*(cbuff[21]-'0')+10*(cbuff[22]-'0')+(cbuff[23]-'0');
						xQueueSendToBack(xRadioTX_Queue, tx_payload, 0);
					}
				}
			}
			
			// HTTP-Debug printouts
			if (rf_debug) {
				if (urlcommand_valid && control_mode==HTTP_MODE) {
					debug_printf("TextRx: %s\r\n",cbuff);
				} else if (control_mode != HTTP_MODE) {
					debug_printf("TextRx: <denied, control mode is CLI>\r\n");
				} else {
					debug_printf("TextRx: <bad string>\r\n");
				}
			}
		}
	
		// 1sec ticker
		if (xTaskGetTickCount()-xTimestamp > 1000) {
			xTimestamp = xTaskGetTickCount();
			//debug_printf(":%d:\r\n",xTimestamp);
			//
			
			//Logfile is open and needs writing to
			if (logging==1) {
				fileMode = 1;
				//Format time
				mseconds = xTimestamp%1000;
				seconds = (xTimestamp/1000)%60;
				minutes = (xTimestamp/60000)%60;
				hours = (xTimestamp/3600000)%24;
				//Format the log entry into a 100-character fixed-width line
            	sprintf(rowData, "%2d:%2d:%2d:%3d  id%2u on%3u pwr%3u dir%3u alt%3u dist%4u mag%3u com%2x rx%4u\r\n", 
            		hours, minutes, seconds, mseconds,
            		pkt_rx->blimpid, pkt_rx->motor_active, 
					pkt_rx->motor_power, pkt_rx->motor_dir, pkt_rx->altitude, 
					pkt_rx->distance, pkt_rx->magnetometer,
					pkt_rx->command, pkt_rx->command_data
            	);
        		//Write and flush the file
        		rc = f_write(&Fil, rowData, strlen(rowData), &bytesWritten);
            	f_sync(&Fil);
            	//Check for errors
            	if (rc) {
					debug_printf("<Logfile Write Error> %u\n\r", rc);
					logging = -1; //logfile write error
					fileMode = 0;
				}
			}
			
			//Logfile is requested to close, but we haven't closed it
			if (logging==2 && fileMode!=2) {
				f_close(&Fil);
			}
			
			//Need to print distance to blimp
			if (fiducial_debug) {
				debug_printf("fid:%d|%d|%d|%d x:%dy:%d px:%dr:%d\r\n",
					id[0], id[1], id[2], id[3], xcentre, ycentre, 
					pixelDistance, distance);
			}
		}
		
		// 0.2sec ticker
		if (xTaskGetTickCount()-xTimestamp2 > 200) {
			xTimestamp2 = xTaskGetTickCount();
			//JSON script needs updating
			//x,y in screen units (0..100)
			//t in ms (large number)
			//pan in deg (0..359)
			//dist in mm (0..large)
			//alt in mm (0..2500+)
			//magdir in deg (0..359)
			//vel in mm/sec (0..1000+-)
			nchars = sprintf(jsonStr, "{\"fiducials\":["\
				"{\"id\":%3d,\"x\":%3u,\"y\":%3u,\"t\":%7d,\"rot\":%3u},"\
				"{\"id\":%3d,\"x\":%3u,\"y\":%3u,\"t\":%7d,\"rot\":%3u},"\
				"{\"id\":%3d,\"x\":%3u,\"y\":%3u,\"t\":%7d,\"rot\":%3u},"\
				"{\"id\":%3d,\"x\":%3u,\"y\":%3u,\"t\":%7d,\"rot\":%3u} "\
				"],\"t\":%7d,\"pan\":%3u,\"dist\":%5u,\"alt\":%4u,"\
				"\"magdir\":%3u}", 
				/*//demo values
				(uint16_t)id[0], (uint16_t)40, (uint16_t)20, (int)xTimestamp2-10,  (uint16_t)320,
				(uint16_t)id[1], (uint16_t)30, (uint16_t)30, (int)xTimestamp2-200, (uint16_t)220,
				(uint16_t)id[2], (uint16_t)60, (uint16_t)90, (int)xTimestamp2-300, (uint16_t)10,
				(uint16_t)id[3], (uint16_t)70, (uint16_t)50, (int)xTimestamp2-4000,(uint16_t)70,
				(int)xTimestamp2, (uint16_t)(xTimestamp2/200), (uint16_t)(1000+xTimestamp2%1000), (uint16_t)(500+xTimestamp2%1000),//pkt_rx->distance, 
				(uint16_t)pkt_rx->magnetometer
				*/
				//actual values
				id[0], x[0], y[0], xBlimpTime[0], rotation[0],
				id[1], x[1], y[1], xBlimpTime[1], rotation[1],
				id[2], x[2], y[2], xBlimpTime[2], rotation[2],
				id[3], x[3], y[3], xBlimpTime[3], rotation[3],
				(int)xTimestamp2, (uint16_t)pan, (uint16_t)distance, 
				pkt_rx->distance, pkt_rx->magnetometer
			);
		}
		
		vTaskDelay(10);
	}
}

/*-----------------------------------------------------------*/
/* CLI PanTilt Function */
static portBASE_TYPE prvPanTiltCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ) {
	
	long lParam_len1; 
	long lParam_len2; 
	char * cCmd_string1;
	char * cCmd_string2;	
	//
	// Purpose: demonstration of multiple arguments, str->int parsing,
	//			 variable-passing queues

	//Get parameters from command string
	cCmd_string1 = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len1);
	cCmd_string2 = FreeRTOS_CLIGetParameter(pcCommandString, 2, &lParam_len2);
	
	if (cCmd_string1[0]=='s') {
		//stop auto-tracking
		autopan = 0;
		xWriteBufferLen = sprintf(pcWriteBuffer, "Automated pan-tilt: OFF\r\n");
	} else if (cCmd_string1[0]=='g') {
		//stop auto-tracking
		autopan = 1;
		xWriteBufferLen = sprintf(pcWriteBuffer, "Automated pan-tilt: ACTIVE\r\n");
	} else {
		//Read and set pan and tilt
		pan  = panLimits (SERVO_X_LEFT+str2int(cCmd_string1, lParam_len1));
		tilt = tiltLimits(SERVO_Y_LEFT+str2int(cCmd_string2, lParam_len2));
		servo_set_pan_tilt(pan, tilt);
		xWriteBufferLen = sprintf(pcWriteBuffer, "\n\r pan: %d\n\r tilt: %d\n\r",
			pan, tilt);
	}
	
	return pdFALSE;
}

/*-----------------------------------------------------------*/
/* CLI Laser Function */
static portBASE_TYPE prvLaserCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ) {

	//Get parameters from command string
	cCmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);

	if(cCmd_string[0] == 'o' && cCmd_string[1] == 'n') {
	    PIO_Set(&pinLaser); //turn on
		xWriteBufferLen = sprintf(pcWriteBuffer, "\nOnn\n\r");
	} else if (cCmd_string[0] == 'o' && cCmd_string[1] == 'f' && cCmd_string[2] == 'f') {
	    PIO_Clear(&pinLaser); //turn off
		xWriteBufferLen = sprintf(pcWriteBuffer, "\nOff\n\r");
	}
	return pdTRUE;
}

/*-----------------------------------------------------------*/
/* CLI Blimp ID Function */
static portBASE_TYPE prvBlimpCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ) {

	//Get parameters from command string
	cCmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);

	noCommand = 0;
	if (lParam_len==1) {
		if (isdigit(cCmd_string[0])) {
			global_blimpid = cCmd_string[0]-'0';
		}
	} else if (lParam_len==2) {
		if (isdigit(cCmd_string[0]) && isdigit(cCmd_string[1])) {
			global_blimpid = 10*(cCmd_string[0]-'0') + (cCmd_string[1]-'0');
		}
	} else {
		noCommand = 1;
	}
	
	if (noCommand) {		
		xWriteBufferLen = sprintf(pcWriteBuffer, "Invalid blimp ID\r\n");
	} else {
		xWriteBufferLen = sprintf(pcWriteBuffer, "Blimp ID set: %d\r\n", 
			global_blimpid);
		pkt_tx->blimpid = global_blimpid;
		//restart radio on new freq.
		nrf24l01plus_init();
	}
	
	return pdTRUE;
}

/*-----------------------------------------------------------*/
/* CLI Radio Function */
static portBASE_TYPE prvRadioCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ) {

	//Get parameters from command string
	cCmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);

	//Decide which command to use
	noCommand = 0;
	if(cCmd_string[0] == 'o' && cCmd_string[1] == 'n') {
	    pkt_tx->motor_active = 1;
	} else if (cCmd_string[0] == 'o' && cCmd_string[1] == 'f' && \
		cCmd_string[2] == 'f') {
	    pkt_tx->motor_active = 0;
	} else if (cCmd_string[0] == 'q') {
		pkt_tx->motor_power = 255;
		pkt_tx->motor_dir = 0;
	} else if (cCmd_string[0] == 'w') {
		pkt_tx->motor_power = 255;
		pkt_tx->motor_dir = 127;
	} else if (cCmd_string[0] == 'e') {
		pkt_tx->motor_power = 255;
		pkt_tx->motor_dir = 255;
	} else if (cCmd_string[0] == 's') {
		pkt_tx->motor_power = 127;
		pkt_tx->motor_dir = 127;
	} else if (cCmd_string[0] == 'z') {
		pkt_tx->motor_power = 0;
		pkt_tx->motor_dir = 0;
	} else if (cCmd_string[0] == 'x') {
		pkt_tx->motor_power = 0;
		pkt_tx->motor_dir = 127;
	} else if (cCmd_string[0] == 'c') {
		pkt_tx->motor_power = 0;
		pkt_tx->motor_dir = 255;
	} else if (cCmd_string[0] == '1') {
		pkt_tx->altitude = 32;
	} else if (cCmd_string[0] == '2') {
		pkt_tx->altitude = 127;
	} else if (cCmd_string[0] == '3') {
		pkt_tx->altitude = 255;
	} else if (cCmd_string[0] == '-') {
		pkt_tx_set_defaults();
	} else {
		noCommand = 1;
	}
	
	if (noCommand) {		
		xWriteBufferLen = sprintf(pcWriteBuffer, "Invalid radio command\r\n");
	} else if (control_mode != CLI_MODE) {
		xWriteBufferLen = sprintf(pcWriteBuffer, "Tx radio: <denied, control mode is HTTP>\r\n");
	} else {
		xWriteBufferLen = sprintf(pcWriteBuffer, "Tx radio: %.*s\r\n", 
			lParam_len, cCmd_string);
		xQueueSendToBack(xRadioTX_Queue, tx_payload, 0);
	}
	
	return pdTRUE;
}

/*-----------------------------------------------------------*/
/* CLI Radio-Command Function */
static portBASE_TYPE prvCommandCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ) {

	//Get parameters from command string
	cCmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);

	//Decide which command to use
	noCommand = 0;
	if(cCmd_string[0] == 'o' && cCmd_string[1] == 'f' && cCmd_string[2] == 'f'){
	    pkt_tx->command = 0x00;
	} else if (cCmd_string[0] == 'i' && cCmd_string[1] == 'd') {
		//passkey
		pkt_tx->command = 0xC2;
	} else if (cCmd_string[0] == 'r' && cCmd_string[1] == 'a' && \
			   cCmd_string[2] == 'w' && cCmd_string[3] == 'x') {
		//magnotometer x-value (raw)
		pkt_tx->command = 0xC3;
	} else if (cCmd_string[0] == 'i' && cCmd_string[1] == 'r') {
		//infrared rangefinder (raw)
		pkt_tx->command = 0xC4;
	} else if (cCmd_string[0] == 'l' && cCmd_string[1] == 'e' && \
			   cCmd_string[2] == 'f' && cCmd_string[3] == 't') {
		//turn left 180
		pkt_tx->command = 0xC7;
	} else if (cCmd_string[0] == 'r' && cCmd_string[1] == 'i' && \
			   cCmd_string[2] == 'g' && cCmd_string[3] == 'h' && \
			   cCmd_string[4] == 't') {
		//turn right 180
		pkt_tx->command = 0xC8;
	} else {
		noCommand = 1;
	}
	
	if (noCommand) {		
		xWriteBufferLen = sprintf(pcWriteBuffer, "Invalid radio command\r\n");
	} else {
		xWriteBufferLen = sprintf(pcWriteBuffer, "Tx radioCommand: %.*s\r\n", 
			lParam_len, cCmd_string);
		xQueueSendToBack(xRadioTX_Queue, tx_payload, 0);
	}
	
	return pdTRUE;
}

/*-----------------------------------------------------------*/
/* CLI debug Function */
static portBASE_TYPE prvDebugCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ) {
    
    //Get parameters from command string
	cCmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);
	
	noCommand = 0;
	if(cCmd_string[0]=='r'&&cCmd_string[1]=='f') {
		rf_debug = !rf_debug;
		xWriteBufferLen = sprintf(pcWriteBuffer, "rf_debug:%d   \r\n", rf_debug);
	} else if (cCmd_string[0]=='w'&&cCmd_string[1]=='e'&&cCmd_string[2]=='b') {
		web_debug = !web_debug;
		xWriteBufferLen = sprintf(pcWriteBuffer, "web_debug:%d\r\n", web_debug);
	} else if (cCmd_string[0]=='t'&&cCmd_string[1]=='u'&&cCmd_string[2]=='i'&&\
			   cCmd_string[3]=='o'){
		tuio_debug = !tuio_debug;
		xWriteBufferLen = sprintf(pcWriteBuffer, "tuio_debug:%d\r\n", tuio_debug);
	} else if (cCmd_string[0]=='f'&&cCmd_string[1]=='i'&&cCmd_string[2]=='d') {
		fiducial_debug = !fiducial_debug;
		xWriteBufferLen = sprintf(pcWriteBuffer, "fiducial_debug:%d\r\n", fiducial_debug);
	} else {
		xWriteBufferLen = sprintf(pcWriteBuffer, "Invalid debug mode\r\n");
	}
	
	return pdTRUE;
}

/*-----------------------------------------------------------*/
/* CLI mode Function */
static portBASE_TYPE prvModeCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ) {
    
	if (control_mode == HTTP_MODE) {
		control_mode = CLI_MODE;
		xWriteBufferLen = sprintf(pcWriteBuffer, "control source: CLI\r\n");
	} else if (control_mode == CLI_MODE) {
		control_mode = HTTP_MODE;
		xWriteBufferLen = sprintf(pcWriteBuffer, "control source: HTTP\r\n");
	}
	
	return pdTRUE;
}

/*-----------------------------------------------------------*/
/* CLI Ls Function */
static portBASE_TYPE prvLsCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ) {

	//File properties
	FRESULT rc;	/* Result code */
	DIR dir;	/* Directory object */
	FILINFO fno;/* File information object */
	UINT bw, br, i;
	
	//Counting files/folders
	short nfiles, nfolders;
	nfiles = 0;

	nfolders = 0;
	
	//Open root directory, iterate through folders
	rc = f_opendir(&dir, "");
	if (rc) {
		debug_printf("\n\rCannot read root directory!\n\r");
		vTaskDelay(1000);
	}
	debug_printf("\n\r");
    for (;;) {
		rc = f_readdir(&dir, &fno);	/* Read a directory item */
		if (rc || !fno.fname[0]) 	/* Error or end of dir */
			{ break; }					
		if (fno.fattrib & AM_DIR) {
			debug_printf("D]        - %s\n\r", fno.fname);
			nfolders++;
		} else {
			debug_printf("f] %8lu %s\n\r", fno.fsize, fno.fname);
			nfiles++;	
		}
        vTaskDelay(1);//5
	}

	//Write command output string to buffer.
	xWriteBufferLen = sprintf(pcWriteBuffer, "\n\r total: %d files, %d folders\n\r",
		nfiles, nfolders);
	
	return pdFALSE;
}

/*-----------------------------------------------------------*/
/* CLI Task Usage Function */
static portBASE_TYPE prvTasksCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ) {
 	
 	short strPtr, charCount;
 	
 	//Read data into a clean text buffer
 	vTaskList(bigTextBuf);
 	charCount = strlen(bigTextBuf);
 	
 	//Loop to print chunks
	strPtr = 0;
	while (strPtr < charCount) {
		debug_printf("%.20s", &(bigTextBuf[strPtr]));
		strPtr += 20;
		vTaskDelay(5); 
	}
 	
 	//Write command output string to buffer.
	xWriteBufferLen = sprintf(pcWriteBuffer, "\n\r task total: %u \n\r", 
		uxTaskGetNumberOfTasks());
	
	return pdFALSE;
}

/*-----------------------------------------------------------*/
/* CLI Log Function */
static portBASE_TYPE prvLogCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ) {
    char i;
    FRESULT rc, rc2;

	//Get parameters from command string
	cCmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);

	//Decide which command to use
	noCommand = 0;
	if(cCmd_string[0] == 's' && cCmd_string[1] == 't' && \
	   cCmd_string[2] == 'a' && cCmd_string[3] == 'r' && cCmd_string[4] == 't'){
	    //Open Command
		if (logging==-2) {
			//logfile open error
			xWriteBufferLen = sprintf(pcWriteBuffer, "ERROR: Logfile creation\r\n");
			noCommand = 2;
		} else if (logging==-1) {
			//logfile write error
			xWriteBufferLen = sprintf(pcWriteBuffer, "ERROR: Logfile write\r\n");
			noCommand = 2;
		} else if (logging==0) {
			//logfile untouched
			for (i=0; i>=0; i++){
    			// loop through logfile names, starting at file 0
    			sprintf(logFName, "5493_L%d.TXT", i);
    			rc = f_open(&Fil, logFName, FA_READ);
    			// if file doesn't exist, it's the next logfile - create it
    			if (rc){
    				rc2 = f_open(&Fil, logFName, FA_WRITE | FA_CREATE_ALWAYS);
    				// if file couldn't be open, log the error flag
    				if (rc2) {
    					logging = -2;
    				} else {
    					logging = 1;
    				}
    				//f_close(&Fil); Don't close! It's used by the control task.
    				break;
    			}
   			}
		} else if (logging==1) {
			//logfile open
			xWriteBufferLen = sprintf(pcWriteBuffer, "Log start: already started\r\n");
			noCommand = 2;
		} else if (logging==2) {
			//logfile closed
			xWriteBufferLen = sprintf(pcWriteBuffer, "Log start: already stopped\r\n");
			noCommand = 2;
		} else {
			//invalid logfile state
			xWriteBufferLen = sprintf(pcWriteBuffer, "ERROR: Invalid logfile state!\r\n");
			noCommand = 2;
		}
	} else if (cCmd_string[0] == 's' && cCmd_string[1] == 't' && \
			   cCmd_string[2] == 'o' && cCmd_string[3] == 'p') {
		//Close Command
		if (logging==-2) {
			//logfile open error
			xWriteBufferLen = sprintf(pcWriteBuffer, "ERROR: Logfile creation\r\n");
			noCommand = 2;
		} else if (logging==-1) {
			//logfile write error
			xWriteBufferLen = sprintf(pcWriteBuffer, "ERROR: Logfile write\r\n");
			noCommand = 2;
		} else if (logging==0) {
			//logfile untouched
			xWriteBufferLen = sprintf(pcWriteBuffer, "Log stop: not started\r\n");
			noCommand = 2;
		} else if (logging==1) {
			//logfile open
			//
			logging = 2;
			//
		} else if (logging==2) {
			//logfile closed
			xWriteBufferLen = sprintf(pcWriteBuffer, "Log stop: already stopped\r\n");
		} else {
			//invalid logfile state
			xWriteBufferLen = sprintf(pcWriteBuffer, "ERROR: Invalid logfile state!\r\n");
			noCommand = 2;
		}
	} else {
		noCommand = 1;
	}
	
	if (noCommand==1) {		
		xWriteBufferLen = sprintf(pcWriteBuffer, "Invalid logfile command\r\n");
	} else if (noCommand==0) {
		xWriteBufferLen = sprintf(pcWriteBuffer, "Logfile: %.*s %s\r\n", 
			lParam_len, cCmd_string, logFName);
	}
	
	return pdTRUE;
}

/*-----------------------------------------------------------*/
/* CLI Remove Function */
static portBASE_TYPE prvRemoveCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ) {
    FRESULT rc;
    
    //Get parameters from command string
	cCmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);
    
    noCommand=0;
    //Delete the file
    rc = f_unlink (cCmd_string);
    if (rc) {
    	noCommand=1;
    }
	
	if (noCommand==1) {		
		xWriteBufferLen = sprintf(pcWriteBuffer, "REMOVE: file %.*s failed, code %d\r\n",
			lParam_len, cCmd_string, rc);
	} else {
		xWriteBufferLen = sprintf(pcWriteBuffer, "%.*s deleted.\r\n", 
			lParam_len, cCmd_string);
	}
	
	return pdTRUE;
}

/*-----------------------------------------------------------*/
/*-----------------------------------------------------------*/
/*-----------------------------------------------------------*/
/* USB CLI receive task */
void vUSB_cli_receive_task(void *pvParameters) {

	char cRxedChar;
	char cInputString[20];
	char cInputIndex = 0;
	int8_t *pcOutputString;
	portBASE_TYPE xReturned;

	//Initialise pointer to CLI output buffer.
	memset(cInputString, 0, sizeof(cInputString));
	pcOutputString = FreeRTOS_CLIGetOutputBuffer();

	for (;;) {
		//Receive character from USB receive
		cRxedChar = ucUSBReadByte();
		if ((cRxedChar != 0) && (cRxedChar != 5)) {

			//reflect byte
			vUSBSendByte(cRxedChar);

			//Process only if return is received.
			if (cRxedChar == '\r') {

				//Put null character in command input string.
				cInputString[cInputIndex] = '\0';

				//Process command input string.
				xReturned = FreeRTOS_CLIProcessCommand( cInputString, pcOutputString, configCOMMAND_INT_MAX_OUTPUT_SIZE );

				memset(cInputString, 0, sizeof(cInputString));
				cInputIndex = 0;
				
				//Display CLI output string
				debug_printf("%s\n\r",pcOutputString);
			
			} else {

				if( cRxedChar == '\r' ) {
					// Ignore the character.
				} else if( cRxedChar == '\b' ) {
					// Backspace was pressed.  Erase the last character in the
					//string - if any.
					if( cInputIndex > 0 ) {
						cInputIndex--;
						cInputString[ cInputIndex ] = '\0';
					}
				} else {
					// A character was entered.  Add it to the string
					// entered so far.  When a \n is entered the complete
					// string will be passed to the command interpreter.
					if( cInputIndex < 20 ) {
						cInputString[ cInputIndex ] = cRxedChar;
						cInputIndex++;
					}
				}
			}		
		}

		vTaskDelay(50);
	}
}

/*-----------------------------------------------------------*/
/* Hardware Initialisation */
static void prvSetupHardware( void )
{
	portDISABLE_INTERRUPTS();
	
	/* When using the JTAG debugger the hardware is not always initialised to
	the correct default state.  This line just ensures that this does not
	cause all interrupts to be masked at the start. */
	AT91C_BASE_AIC->AIC_EOICR = 0;
	
	/* Most setup is performed by the low level init function called from the
	startup asm file. */

	/* Enable the peripheral clock. */
    AT91C_BASE_PMC->PMC_PCER = 1 << AT91C_ID_PIOA;
    AT91C_BASE_PMC->PMC_PCER = 1 << AT91C_ID_PIOB;
	AT91C_BASE_PMC->PMC_PCER = 1 << AT91C_ID_EMAC;

	/* Initialise the LED outputs for use by the demo application tasks. */
	LED_Configure(0);
}

/*-----------------------------------------------------------*/
/* Idle Application Task */
void vApplicationIdleHook( void )
{
	static portTickType xLastTx = 0;

	/* The idle hook simply prints the idle tick count */
	if( ( xTaskGetTickCount() - xLastTx ) > mainUSB_TX_FREQUENCY )
	{
		xLastTx = xTaskGetTickCount();
		//debug_printf("IDLE Tick %d\n", xLastTx);		
		LED_Toggle(0);
	}
}
