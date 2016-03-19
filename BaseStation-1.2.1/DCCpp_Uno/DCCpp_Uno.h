/**********************************************************************

DCCpp_Uno.h
COPYRIGHT (c) 2013-2015 Gregg E. Berman

Part of DCC++ BASE STATION for the Arduino

**********************************************************************/

#include "Config.h"

#ifndef DCCpp_Uno_h
#define DCCpp_Uno_h

/////////////////////////////////////////////////////////////////////////////////////
// AUTO-SELECT ARDUINO BOARD
/////////////////////////////////////////////////////////////////////////////////////

#ifdef ARDUINO_AVR_MEGA                   // is using Mega 1280, define as Mega 2560 (pinouts and functionality are identical)
  #define ARDUINO_AVR_MEGA2560
#endif

/*---------------------------------------------------------------
 *  PINS D3, D8, D9, D11, D12, D13, A0, A1 used by Motor Shield
 *  PINS D47, D48 used by Loconet Shield
 *  PINS D50, D51, D52, D53, D10, D4 used by Ethernet Shield
 ----------------------------------------------------------------*/
#if defined  ARDUINO_AVR_MEGA2560

  #define ARDUINO_TYPE    "MEGA"

  #define DCC_SIGNAL_PIN_MAIN 12          // Arduino Mega - uses OC1B
  #define DCC_SIGNAL_PIN_PROG 2           // Arduino Mega - uses OC3B
  

#else

  #error CANNOT COMPILE - DCC++ ONLY WORKS WITH AN ARDUINO MEGA 1280/2560

#endif

/////////////////////////////////////////////////////////////////////////////////////
// SELECT MOTOR SHIELD
/////////////////////////////////////////////////////////////////////////////////////

#if MOTOR_SHIELD_TYPE == 0

  #define MOTOR_SHIELD_NAME "ARDUINO MOTOR SHIELD"

  #define SIGNAL_ENABLE_PIN_MAIN 3
  #define SIGNAL_ENABLE_PIN_PROG 11

  #define CURRENT_MONITOR_PIN_MAIN A0
  #define CURRENT_MONITOR_PIN_PROG A1

  #define DIRECTION_MOTOR_CHANNEL_PIN_A 12
  #define DIRECTION_MOTOR_CHANNEL_PIN_B 13

#elif MOTOR_SHIELD_TYPE == 1

  #define MOTOR_SHIELD_NAME "POLOLU MC33926 MOTOR SHIELD"

  #define SIGNAL_ENABLE_PIN_MAIN 9
  #define SIGNAL_ENABLE_PIN_PROG 11

  #define CURRENT_MONITOR_PIN_MAIN A0
  #define CURRENT_MONITOR_PIN_PROG A1

  #define DIRECTION_MOTOR_CHANNEL_PIN_A 7
  #define DIRECTION_MOTOR_CHANNEL_PIN_B 8

#else

  #error CANNOT COMPILE - PLEASE SELECT A PROPER MOTOR SHIELD TYPE

#endif

/////////////////////////////////////////////////////////////////////////////////////
// SELECT COMMUNICATION INTERACE
/////////////////////////////////////////////////////////////////////////////////////

#if COMM_TYPE == 0

  #define INTERFACE Serial

#elif COMM_TYPE == 1

  #define INTERFACE eServer
  #define SDCARD_CS 4
  
#else

  #error CANNOT COMPILE - PLEASE SELECT A PROPER COMMUNICATIONS INTERFACE TYPE

#endif

/////////////////////////////////////////////////////////////////////////////////////
// SET WHETHER TO SHOW PACKETS - DIAGNOSTIC MODE ONLY
/////////////////////////////////////////////////////////////////////////////////////

// If SHOW_PACKETS is set to 1, then for select main operations track commands that modify an internal DCC packet register,
// if printFlag for that command is also set to 1, DCC++ BASE STATION will additionally return the 
// DCC packet contents of the modified register in the following format:

//    <* REG: B1 B2 ... Bn CSUM / REPEAT>
//
//    REG: the number of the main operations track packet register that was modified
//    B1: the first hexidecimal byte of the DCC packet
//    B2: the second hexidecimal byte of the DCC packet
//    Bn: the nth hexidecimal byte of the DCC packet
//    CSUM: a checksum byte that is required to be the final byte in any DCC packet
//    REPEAT: the number of times the DCC packet was re-transmitted to the tracks after its iniital transmission
 
#define SHOW_PACKETS  0       // set to zero to disable printing of every packet for select main operations track commands

/////////////////////////////////////////////////////////////////////////////////////

#endif

 
