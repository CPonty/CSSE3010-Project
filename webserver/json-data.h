/* -----------------------------------------------------------------------------
# Company name: CSSE3010 - Embedded Systems Design & Interfacing
# Project name: Project 3 (Final)
# Module name : JSON Data
# Functionality: Define shared data for JSON page generator
# Hardware platform: NetduinoPlus (AT91SAM7)
#
# Author name: C. Ponticello
# Creation date: 22102012
# Revision date (name): -
# Changes implemented (date): -
#(Comments): 
------------------------------------------------------------------------------*/

#ifndef __JSONDATA_H__
#define __JSONDATA_H__

/*
 * Data from the main application accessible to the json page builder.
 *  read-only in web_Task, read-write elsewhere
*/

#define JSON_BUFSIZE (512)

char *jsonStr;

#endif /* __JSONDATA_H__ */
/** @} */
/** @} */
