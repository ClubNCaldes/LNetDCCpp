/**********************************************************************

LNetCmdStation.h
COPYRIGHT (c) 2016 Dani Guisado

Command Station Loconet implementation

**********************************************************************/

#ifndef LNetCmdStation_h
#define LNetCmdStation_h


#include "PacketRegister.h"
#include "CurrentMonitor.h"
#include <LiquidCrystal.h>
#include <LocoNet.h>

  
struct LNetCmdStation{  
  lnMsg *LnPacket;
  //DGS Loconet Slot table for locomotives
  rwSlotDataMsg locoNetSlots[50]; //TODO use a define
  static volatile RegisterList *mRegs, *pRegs;
  static CurrentMonitor *mMonitor;
  void init(volatile RegisterList *, volatile RegisterList *, CurrentMonitor *, LiquidCrystal *);  
  void checkPacket();
  void sendOPC_GP(byte);
  void processIncomingLoconetCommand();
};

#endif

