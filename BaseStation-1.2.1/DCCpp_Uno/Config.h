/**********************************************************************

Config.h
COPYRIGHT (c) 2013-2015 Gregg E. Berman

Part of DCC++ BASE STATION for the Arduino

**********************************************************************/

/////////////////////////////////////////////////////////////////////////////////////
//
// DEFINE MOTOR_SHIELD_TYPE ACCORDING TO THE FOLLOWING TABLE:
//
//  0 = ARDUINO MOTOR SHIELD          (MAX 18V/2A PER CHANNEL)
//  1 = POLOLU MC33926 MOTOR SHIELD   (MAX 28V/3A PER CHANNEL)

#define MOTOR_SHIELD_TYPE   0

// SET THIS TO 1 IF THE MOTOR SHIELD HAS CURRENT FEEDBACK
// SET THIS TO 0 IF THE MOTOR SHIELD DOES NOT HAVE CURRENT FEEDBACK
#define MOTOR_SHIELD_SUPPORTS_FEEDBACK 0

// SET TO ANYTHING OTHER THAN 0 TO RUN WITHOUT BUTTONS
#define NO_BUTTONS          0         

/////////////////////////////////////////////////////////////////////////////////////
//
// DEFINE NUMBER OF MAIN TRACK REGISTER

#define MAX_MAIN_REGISTERS 50

