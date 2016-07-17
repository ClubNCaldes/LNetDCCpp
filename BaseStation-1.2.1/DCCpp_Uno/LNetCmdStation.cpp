/**********************************************************************

CurrentMonitor.cpp
COPYRIGHT (c) 2013-2015 Gregg E. Berman

Part of DCC++ BASE STATION for the Arduino

**********************************************************************/

#include "LNetCmdStation.h"
#include "DCCpp_Uno.h"

///////////////////////////////////////////////////////////////////////////////

volatile RegisterList *LNetCmdStation::mRegs;
volatile RegisterList *LNetCmdStation::pRegs;
CurrentMonitor *LNetCmdStation::mMonitor;

void LNetCmdStation::init(volatile RegisterList *_mRegs, volatile RegisterList *_pRegs, CurrentMonitor *_mMonitor)
{  
  mRegs=_mRegs;
  pRegs=_pRegs;
  mMonitor=_mMonitor;
  
  //DGS initialize slots to FREE
  int n;
  for (n=0;n<MAX_MAIN_REGISTERS;n++)
  {
    locoNetSlots[n].command=0xE7;
    locoNetSlots[n].mesg_size=0x0E;
    locoNetSlots[n].slot=n+1;
    locoNetSlots[n].stat=LOCO_FREE | DEC_MODE_128;
    
    locoNetSlots[n].adr=0;
    locoNetSlots[n].spd=0;
    locoNetSlots[n].dirf=0;
    locoNetSlots[n].trk = GTRK_POWER & GTRK_MLOK1; // POWER ON & Loconet 1.1 by default
    locoNetSlots[n].ss2=0;
    locoNetSlots[n].adr2=0;
    locoNetSlots[n].snd=0;
    locoNetSlots[n].id1=0;
    locoNetSlots[n].id2=0;
  }

} // LNetCmdStation::init

void LNetCmdStation::checkPacket()
{
  // Check for any received LocoNet packets
  LnPacket = LocoNet.receive();
  if (LnPacket)
  {
    Serial.println("# Loconet incomming #");
    processIncomingLoconetCommand();
  }
}

void LNetCmdStation::sendOPC_GP(byte on) 
{
        lnMsg SendPacket;
        if (on==ON) {
            SendPacket.data[ 0 ] = OPC_GPON;  
        } else {
            SendPacket.data[ 0 ] = OPC_GPOFF;  
        }
        LocoNet.send( &SendPacket ) ;
}

void LNetCmdStation::processIncomingLoconetCommand() 
{  
  int n; int freeslot=MAX_MAIN_REGISTERS;
  unsigned char opcode = (int)LnPacket->sz.command;
  char s[20];
  byte myByte;
  int myAddress;
  
  switch (opcode)
  {
    case OPC_GPON: //Global ON command
      #ifdef DEBUG
        Serial.println("# GLOBAL ON #");
      #endif
      mMonitor->setGlobalPower(ON);
      //TODO send GPON
      break;
      
    case OPC_GPOFF: // Global OFF command
      #ifdef DEBUG
        Serial.println("# GLOBAL OFF #");
      #endif            
      mMonitor->setGlobalPower(OFF);
      break;
      
    case OPC_LOCO_ADR: // Request of Loco
      #ifdef DEBUG
        Serial.println("# OPC_LOCO_ADR Request of Loco #");
      #endif
      //Check if it is in slot already and also gets the first possible free slot
      //Slot 0 is not examined as it is used as BT2 slot (TODO not implemented)
      
      for (n=1;n<MAX_MAIN_REGISTERS;n++)
      {
        if (((locoNetSlots[n].stat & LOCOSTAT_MASK) == LOCO_FREE) && n<MAX_MAIN_REGISTERS) 
          freeslot=n;
        else if (locoNetSlots[n].adr==LnPacket->la.adr_lo && locoNetSlots[n].adr2==LnPacket->la.adr_hi) 
          break;
      }
  
      //Loco not found and no free slots
      if (n==MAX_MAIN_REGISTERS && freeslot==MAX_MAIN_REGISTERS)
      {
        #ifdef DEBUG
        Serial.println("# !LONGACK! No free slots for loco #");
        #endif 
        LocoNet.sendLongAck(0);
        break;
      }
      //Loco not found, add to the first free slot speed 0, direction front, F0 ON
      if (n==MAX_MAIN_REGISTERS) 
      {
        n=freeslot;
        locoNetSlots[n].command=0xE7;
        locoNetSlots[n].mesg_size=0x0E;
        locoNetSlots[n].slot=n;
        locoNetSlots[n].stat=LOCO_IDLE | DEC_MODE_128;
        locoNetSlots[n].adr=LnPacket->la.adr_lo;
        locoNetSlots[n].spd=0;
        locoNetSlots[n].dirf=DIRF_F0;
        locoNetSlots[n].trk &= GTRK_POWER & GTRK_MLOK1; // POWER ON & Loconet 1.1 by default
        locoNetSlots[n].ss2=0;
        locoNetSlots[n].adr2=LnPacket->la.adr_hi;
        locoNetSlots[n].snd=0;
        locoNetSlots[n].id1=0;
        locoNetSlots[n].id2=0;
      }    
      #ifdef DEBUG
      Serial.println("# SLOT SENT #");
      #endif 
      LocoNet.send ((lnMsg*)&locoNetSlots[n]);
      break;
      
    case OPC_MOVE_SLOTS:
      #ifdef DEBUG
        Serial.println("# OPC_MOVE_SLOTS #");
      #endif
      //Check slot range (0 DISPATCH NOT SUPPORTED, DIFFERENT NOT SUPPORTED)
      if (LnPacket->sm.dest>=MAX_MAIN_REGISTERS || LnPacket->sm.src>=MAX_MAIN_REGISTERS || LnPacket->sm.dest!=LnPacket->sm.src || LnPacket->sm.dest<1 || LnPacket->sm.src<1)
      {
        LocoNet.sendLongAck(0);
        return;
      }
      
      locoNetSlots[LnPacket->sm.dest].stat|=LOCO_IN_USE;    
      LocoNet.send ((lnMsg*)&locoNetSlots[LnPacket->sm.dest]);
      //<t REGISTER CAB SPEED DIRECTION>
      myAddress=locoNetSlots[LnPacket->sm.dest].adr+(locoNetSlots[LnPacket->sm.dest].adr2<<7);
      sprintf(s,"%d %d %d %d",LnPacket->sm.dest,myAddress,locoNetSlots[LnPacket->sm.dest].spd,bitRead(locoNetSlots[LnPacket->sm.dest].dirf,5));
      Serial.print("====>> ");Serial.println(s);
      mRegs->setThrottle(s);
      // <f CAB BYTE1 [BYTE2]>
      //   To set functions F0-F4 on (=1) or off (=0):
      //   BYTE1:  128 + F1*1 + F2*2 + F3*4 + F4*8 + F0*16
      //   BYTE2:  omitted
      myByte=128;
      bitWrite(myByte,4,bitRead(locoNetSlots[LnPacket->sm.dest].dirf,4)); //F0
      bitWrite(myByte,0,bitRead(locoNetSlots[LnPacket->sm.dest].dirf,0)); //F1
      bitWrite(myByte,1,bitRead(locoNetSlots[LnPacket->sm.dest].dirf,1)); //F2
      bitWrite(myByte,2,bitRead(locoNetSlots[LnPacket->sm.dest].dirf,2)); //F3
      bitWrite(myByte,3,bitRead(locoNetSlots[LnPacket->sm.dest].dirf,3)); //F4
      sprintf(s,"%d %d",myAddress,myByte);      
      mRegs->setFunction(s);
      break;
      
    case OPC_SLOT_STAT1:
      #ifdef DEBUG
        Serial.println("# OPC_SLOT_STAT1 #");
      #endif
      locoNetSlots[LnPacket->ss.slot].stat = LnPacket->ss.stat;
      //<t REGISTER CAB SPEED DIRECTION>
      //char s[20];
      //sprintf(s,"%d %d %d %d",LnPacket->ss.slot,locoNetSlots[LnPacket->ss.slot].adr,locoNetSlots[LnPacket->ss.slot].spd,bitRead(locoNetSlots[LnPacket->ldf.slot].dirf,5));
      //mainRegs.setThrottle(s);
      break;
      
    case OPC_LOCO_SPD:
      #ifdef DEBUG
        Serial.println("# OPC_LOCO_SPD #");
      #endif
      locoNetSlots[LnPacket->lsp.slot].spd = LnPacket->lsp.spd;
      //<t REGISTER CAB SPEED DIRECTION>
      myAddress=locoNetSlots[LnPacket->lsp.slot].adr+(locoNetSlots[LnPacket->lsp.slot].adr2<<7);
      sprintf(s,"%d %d %d %d",LnPacket->lsp.slot,myAddress,locoNetSlots[LnPacket->lsp.slot].spd,bitRead(locoNetSlots[LnPacket->ldf.slot].dirf,5));
      mRegs->setThrottle(s);
      break;
      
    case OPC_LOCO_DIRF:
      #ifdef DEBUG
        Serial.print("# OPC_LOCO_DIRF # Diretion and F0 - F4 # ");
        Serial.println(locoNetSlots[LnPacket->ldf.slot].dirf,BIN);
      #endif
      locoNetSlots[LnPacket->ldf.slot].dirf = LnPacket->ldf.dirf;
      //<t REGISTER CAB SPEED DIRECTION>
      myAddress=locoNetSlots[LnPacket->lsp.slot].adr+(locoNetSlots[LnPacket->lsp.slot].adr2<<7);
      sprintf(s,"%d %d %d %d",LnPacket->ldf.slot,myAddress,locoNetSlots[LnPacket->ldf.slot].spd,bitRead(locoNetSlots[LnPacket->ldf.slot].dirf,5));      
      mRegs->setThrottle(s);
      // <f CAB BYTE1 [BYTE2]>
      //   To set functions F0-F4 on (=1) or off (=0):
      //   BYTE1:  128 + F1*1 + F2*2 + F3*4 + F4*8 + F0*16
      //   BYTE2:  omitted
      myByte=128;      
      bitWrite(myByte,0,bitRead(locoNetSlots[LnPacket->ldf.slot].dirf,0)); //F1
      bitWrite(myByte,1,bitRead(locoNetSlots[LnPacket->ldf.slot].dirf,1)); //F2
      bitWrite(myByte,2,bitRead(locoNetSlots[LnPacket->ldf.slot].dirf,2)); //F3
      bitWrite(myByte,3,bitRead(locoNetSlots[LnPacket->ldf.slot].dirf,3)); //F4
      bitWrite(myByte,4,bitRead(locoNetSlots[LnPacket->ldf.slot].dirf,4)); //F0
      sprintf(s,"%d %d",myAddress,myByte);      
      mRegs->setFunction(s);
      break;
      
    case OPC_LOCO_SND:
      #ifdef DEBUG
        Serial.println("# OPC_LOCO_SND #");
      #endif
      locoNetSlots[LnPacket->ls.slot].snd = LnPacket->ls.snd;

      //*    To set functions F5-F8 on (=1) or off (=0):
      //*   
      //*    BYTE1:  176 + F5*1 + F6*2 + F7*4 + F8*8
      //*    BYTE2:  omitted
      myAddress=locoNetSlots[LnPacket->lsp.slot].adr+(locoNetSlots[LnPacket->lsp.slot].adr2<<7);
      myByte=176;      
      bitWrite(myByte,0,bitRead(locoNetSlots[LnPacket->ls.slot].snd,0)); //F5
      bitWrite(myByte,1,bitRead(locoNetSlots[LnPacket->ls.slot].snd,1)); //F6
      bitWrite(myByte,2,bitRead(locoNetSlots[LnPacket->ls.slot].snd,2)); //F7
      bitWrite(myByte,3,bitRead(locoNetSlots[LnPacket->ls.slot].snd,3)); //F8      
      sprintf(s,"%d %d",myAddress,myByte);      
      mRegs->setFunction(s);
      
      break;

      /*case OPC_IMM_PACKET:
      #ifdef DEBUG
        Serial.println("# OPC_IMM_PACKET # F9 - F12 STILL UNKNOWN!! ");
      #endif
      
      break;*/

    default:
      // ignore the message...
      
        Serial.println("# !! IGNORE MESSAGE !! #");      
        Serial.print("RX: ");
        uint8_t msgLen = getLnMsgSize(LnPacket); 
        for (uint8_t x = 0; x < msgLen; x++)
        {
          uint8_t val = LnPacket->data[x];
            // Print a leading 0 if less than 16 to make 2 HEX digits
          if(val < 16)
            Serial.print('0');
            
          Serial.print(val, HEX);
          Serial.print(' ');
        }
        Serial.println(" <");
      
  }
} // LNetCmdStation::processIncomingLoconetCommand

