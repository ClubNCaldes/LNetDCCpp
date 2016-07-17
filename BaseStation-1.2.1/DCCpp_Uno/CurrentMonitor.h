/**********************************************************************

CurrentMonitor.h
COPYRIGHT (c) 2013-2015 Gregg E. Berman

Part of DCC++ BASE STATION for the Arduino

**********************************************************************/

#ifndef CurrentMonitor_h
#define CurrentMonitor_h

#include "Arduino.h"

#define  CURRENT_SAMPLE_SMOOTHING   0.01
#define  CURRENT_SAMPLE_MAX         300

#define  CURRENT_SAMPLE_TIME        1

// defines for GlobalPowerON
#define OFF 0
#define ON 1
#define EMERGENCY 2

#define PWON_BUTTON_PIN 30               // power on push button
#define PWOFF_BUTTON_PIN 31              // power off push button
#define EMERGENCY_STOP_PIN 32        // external emergency stop
#define PWON_LED_PIN 33                  // green led for POWER ON
#define PWOFF_LED_PIN 34                 // red led for POWER OFF
#define EMERGENCY_LED_PIN 35

struct CurrentMonitor{  
  static long int sampleTime;
  static byte globalPowerON;
  int pin;
  float current;
  char *msg;
  CurrentMonitor(int, char *);
  static boolean checkTime();
  void check();
  void setGlobalPower(uint8_t);
};

#endif

