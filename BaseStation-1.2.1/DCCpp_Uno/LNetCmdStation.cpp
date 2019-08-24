/**********************************************************************

CurrentMonitor.cpp
COPYRIGHT (c) 2013-2015 Gregg E. Berman

Part of DCC++ BASE STATION for the Arduino

**********************************************************************/

#include "LNetCmdStation.h"
#include "DCCpp_Uno.h"

#define DEBUG

///////////////////////////////////////////////////////////////////////////////

volatile RegisterList *LNetCmdStation::mRegs;
volatile RegisterList *LNetCmdStation::pRegs;
CurrentMonitor *LNetCmdStation::mMonitor;
LiquidCrystal *mLcd;

void LNetCmdStation::init(volatile RegisterList *_mRegs, volatile RegisterList *_pRegs, CurrentMonitor *_mMonitor, LiquidCrystal *_mLcd)
{  
  mRegs=_mRegs;
  pRegs=_pRegs;
  mMonitor=_mMonitor;
  mLcd=_mLcd;
  
  // DGS initialize slots to FREE
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
    processIncomingLoconetCommand();
  }
}

void LNetCmdStation::sendOPC_GP(byte on) 
{
        lnMsg SendPacket;
        if (on==ON) 
        {
            SendPacket.data[ 0 ] = OPC_GPON;            
        } 
        else if (on==OFF)
        {
            SendPacket.data[ 0 ] = OPC_GPOFF;             
        }
        else
        {
            SendPacket.data[ 0 ] = OPC_IDLE;  
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
  int cvnum=0;
  int cvvalue=0;

  #ifdef DEBUG 
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
  #endif
  
  switch (opcode)
  {
    case OPC_GPON: // Global ON command
      #ifdef DEBUG
        Serial.println("# GLOBAL ON #");        
      #endif
      mLcd->setCursor(0,0);             // set the LCD cursor   position 
      mLcd->print("Status: ON      ");
      mMonitor->setGlobalPower(ON);      
      break;
      
    case OPC_GPOFF: // Global OFF command
      #ifdef DEBUG
        Serial.println("# GLOBAL OFF #");
      #endif            
      mMonitor->setGlobalPower(OFF);
      mLcd->setCursor(0,0);             // set the LCD cursor   position 
      mLcd->print("Status: OFF     ");   
      break;

    case OPC_IDLE: // Stop emergency
      #ifdef DEBUG
        Serial.println("# EMERGENCY STOP #");
      #endif            
      mMonitor->setGlobalPower(EMERGENCY);
      mLcd->setCursor(0,0);             // set the LCD cursor   position 
      mLcd->print("!! EMERGENCY !!");   
      break;
      
    case OPC_LOCO_ADR: // Request of Loco
      #ifdef DEBUG
        Serial.println("# OPC_LOCO_ADR Request of Loco #");
      #endif
      // Check if it is in slot already and also gets the first possible free slot
      // Slot 0 is not examined as it is used as BT2 slot (TODO not implemented)
      
      for (n=1;n<MAX_MAIN_REGISTERS;n++)
      {
        if (((locoNetSlots[n].stat & LOCOSTAT_MASK) == LOCO_FREE) && n<MAX_MAIN_REGISTERS) 
          freeslot=n;
        else if (locoNetSlots[n].adr==LnPacket->la.adr_lo && locoNetSlots[n].adr2==LnPacket->la.adr_hi) 
          break;
      }
  
      // Loco not found and no free slots
      if (n==MAX_MAIN_REGISTERS && freeslot==MAX_MAIN_REGISTERS)
      {
        #ifdef DEBUG
        Serial.println("# !LONGACK! No free slots for loco #");
        #endif 
        LocoNet.sendLongAck(0);
        break;
      }
      // Loco not found, add to the first free slot speed 0, direction front, F0 ON
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
      // Check slot range (0 DISPATCH NOT SUPPORTED, DIFFERENT NOT SUPPORTED)
      if (LnPacket->sm.dest>=MAX_MAIN_REGISTERS || LnPacket->sm.src>=MAX_MAIN_REGISTERS || LnPacket->sm.dest!=LnPacket->sm.src || LnPacket->sm.dest<1 || LnPacket->sm.src<1)
      {
        LocoNet.sendLongAck(0);
        return;
      }
      
      locoNetSlots[LnPacket->sm.dest].stat|=LOCO_IN_USE;    
      LocoNet.send ((lnMsg*)&locoNetSlots[LnPacket->sm.dest]);
      // <t REGISTER CAB SPEED DIRECTION>
      myAddress=locoNetSlots[LnPacket->sm.dest].adr+(locoNetSlots[LnPacket->sm.dest].adr2<<7);
      sprintf(s,"%d %d %d %d",LnPacket->sm.dest,myAddress,locoNetSlots[LnPacket->sm.dest].spd,bitRead(locoNetSlots[LnPacket->sm.dest].dirf,5));

      mRegs->setThrottle(s);
      // <f CAB BYTE1 [BYTE2]>
      // To set functions F0-F4 on (=1) or off (=0):
      // BYTE1:  128 + F1*1 + F2*2 + F3*4 + F4*8 + F0*16
      // BYTE2:  omitted
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
      // <t REGISTER CAB SPEED DIRECTION>
      // char s[20];
      // sprintf(s,"%d %d %d %d",LnPacket->ss.slot,locoNetSlots[LnPacket->ss.slot].adr,locoNetSlots[LnPacket->ss.slot].spd,bitRead(locoNetSlots[LnPacket->ldf.slot].dirf,5));
      // mainRegs.setThrottle(s);
      break;
      
    case OPC_LOCO_SPD:
      #ifdef DEBUG
        Serial.println("# OPC_LOCO_SPD #");
      #endif
      locoNetSlots[LnPacket->lsp.slot].spd = LnPacket->lsp.spd;
      // <t REGISTER CAB SPEED DIRECTION>
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
      // <t REGISTER CAB SPEED DIRECTION>
      myAddress=locoNetSlots[LnPacket->lsp.slot].adr+(locoNetSlots[LnPacket->lsp.slot].adr2<<7);
      sprintf(s,"%d %d %d %d",LnPacket->ldf.slot,myAddress,locoNetSlots[LnPacket->ldf.slot].spd,bitRead(locoNetSlots[LnPacket->ldf.slot].dirf,5));      
      mRegs->setThrottle(s);
      // <f CAB BYTE1 [BYTE2]>
      // To set functions F0-F4 on (=1) or off (=0):
      // BYTE1:  128 + F1*1 + F2*2 + F3*4 + F4*8 + F0*16
      // BYTE2:  omitted
      myByte=128;      
      bitWrite(myByte,0,bitRead(locoNetSlots[LnPacket->ldf.slot].dirf,0)); // F1
      bitWrite(myByte,1,bitRead(locoNetSlots[LnPacket->ldf.slot].dirf,1)); // F2
      bitWrite(myByte,2,bitRead(locoNetSlots[LnPacket->ldf.slot].dirf,2)); // F3
      bitWrite(myByte,3,bitRead(locoNetSlots[LnPacket->ldf.slot].dirf,3)); // F4
      bitWrite(myByte,4,bitRead(locoNetSlots[LnPacket->ldf.slot].dirf,4)); // F0
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
      bitWrite(myByte,0,bitRead(locoNetSlots[LnPacket->ls.slot].snd,0)); // F5
      bitWrite(myByte,1,bitRead(locoNetSlots[LnPacket->ls.slot].snd,1)); // F6
      bitWrite(myByte,2,bitRead(locoNetSlots[LnPacket->ls.slot].snd,2)); // F7
      bitWrite(myByte,3,bitRead(locoNetSlots[LnPacket->ls.slot].snd,3)); // F8      
      sprintf(s,"%d %d",myAddress,myByte);      
      mRegs->setFunction(s);
      
      break;

    case OPC_WR_SL_DATA: //Programming track
      /*------------------------------------------------------------------------
      This OPC leads to immediate LACK codes:
      <B4>,<7F>,<7F>,<chk> Function NOT implemented, no reply.
      <B4>,<7F>,<0>,<chk> Programmer BUSY , task aborted, no reply.
      <B4>,<7F>,<1>,<chk> Task accepted , <E7> reply at completion.
      <B4>,<7F>,<0x40>,<chk> Task accepted blind NO <E7> reply at completion.

      typedef struct progtask_t {
      uint8_t command;
      uint8_t mesg_size;      ummmmm, size of the message in bytes?                    
      uint8_t slot;           slot number for this request - slot 124 is programmer    
      uint8_t pcmd;           programmer command                                       
      uint8_t pstat;          programmer status error flags in reply message           
      uint8_t hopsa;          Ops mode - 7 high address bits of loco to program        
      uint8_t lopsa;          Ops mode - 7 low  address bits of loco to program        
      uint8_t trk;            track status. Note: bit 3 shows if prog track is busy    
      uint8_t cvh;            hi 3 bits of CV# and msb of data7                        
      uint8_t cvl;            lo 7 bits of CV#                                         
      uint8_t data7;          7 bits of data to program, msb is in cvh above           
      uint8_t pad2;
      uint8_t pad3;
      uint8_t chksum;         exclusive-or checksum for the message                
      } progTaskMsg;

      #define PRG_SLOT          0x7c      This slot communicates with the programming track    

      values and macros to decode programming messages 
      #define PCMD_RW           0x40       1 = write, 0 = read                                  
      #define PCMD_BYTE_MODE    0x20       1 = byte operation, 0 = bit operation (if possible)  
      #define PCMD_TY1          0x10       TY1 Programming type select bit                      
      #define PCMD_TY0          0x08       TY0 Programming type select bit                      
      #define PCMD_OPS_MODE     0x04       1 = Ops mode, 0 = Service Mode                       
      #define PCMD_RSVRD1       0x02       reserved                                             
      #define PCMD_RSVRD0       0x01       reserved                                             
      
      programming mode mask 
      #define PCMD_MODE_MASK      (PCMD_BYTE_MODE | PCMD_OPS_MODE | PCMD_TY1 | PCMD_TY0)

      programming modes
      -----------------
      Paged mode  byte R/W on Service Track 
      #define PAGED_ON_SRVC_TRK       (PCMD_BYTE_MODE)
      Direct mode byte R/W on Service Track 
      #define DIR_BYTE_ON_SRVC_TRK    (PCMD_BYTE_MODE | PCMD_TY0)
      Direct mode bit  R/W on Service Track
      #define DIR_BIT_ON_SRVC_TRK     (PCMD_TY0)
      Physical Register byte R/W on Service Track
      #define REG_BYTE_RW_ON_SRVC_TRK (PCMD_TY1)      
      Service Track Reserved function
      #define SRVC_TRK_RESERVED       (PCMD_TY1 | PCMD_TY0)      
      Ops mode byte program - no feedback
      #define OPS_BYTE_NO_FEEDBACK    (PCMD_BYTE_MODE | PCMD_OPS_MODE)      
      Ops mode byte program - feedback 
      #define OPS_BYTE_FEEDBACK       (OPS_BYTE_NO_FEEDBACK | PCMD_TY0)      
      Ops mode bit program - no feedback
      #define OPS_BIT_NO_FEEDBACK     (PCMD_OPS_MODE)      
      Ops mode bit program - feedback 
      #define OPS_BIT_FEEDBACK        (OPS_BIT_NO_FEEDBACK | PCMD_TY0)
      
      Programmer Status error flags
      #define PSTAT_USER_ABORTED  0x08    /* User aborted this command 
      #define PSTAT_READ_FAIL     0x04    /* Failed to detect Read Compare Acknowledge from decoder 
      #define PSTAT_WRITE_FAIL    0x02    /* No Write acknowledge from decoder                      
      #define PSTAT_NO_DECODER    0x01    /* Service mode programming track empty                   
      
      bit masks for CVH
      #define CVH_CV8_CV9         0x30    /* mask for CV# bits 8 and 9    
      #define CVH_CV7             0x01    /* mask for CV# bit 7           
      #define CVH_D7              0x02    /* MSbit for data value         
      
      build data byte from programmer message
      #define PROG_DATA(ptr)      (((ptr->cvh & CVH_D7) << 6) | (ptr->data7 & 0x7f))
      
      build CV # from programmer message
      #define PROG_CV_NUM(ptr)    (((((ptr->cvh & CVH_CV8_CV9) >> 3) | (ptr->cvh & CVH_CV7)) * 128) + (ptr->cvl & 0x7f))
      ------------------------------------------------------------------------*/

      // Check for programming slot    
      if (LnPacket->pt.slot!=PRG_SLOT)
        break;

      // PCMD value of 00 aborts current SERVICE mode programming and echo <E6>RD
      if (LnPacket->pt.pcmd==0x00)
        break;

      // Bit operations not implemented
      if (((LnPacket->pt.pcmd&PCMD_BYTE_MODE)==0) || ((LnPacket->pt.pcmd&PCMD_RW)>0 && (LnPacket->pt.pcmd&PCMD_OPS_MODE)>0))
      {
        LocoNet.send(OPC_LONG_ACK,0x7F,0x7F);
        break;
      }
      
      digitalWrite(PROG_RELAY1,HIGH);
      digitalWrite(PROG_RELAY2,HIGH);
      digitalWrite(PWOFF_LED_PIN, HIGH);
      
      cvnum=(((((LnPacket->pt.cvh & CVH_CV8_CV9) >> 3) | (LnPacket->pt.cvh & CVH_CV7)) * 128) + (LnPacket->pt.cvl & 0x7f));
      cvnum++;
      cvvalue=(((LnPacket->pt.cvh & CVH_D7) << 6) | (LnPacket->pt.data7 & 0x7f));
      
      // READ ON PROGRAMMING TRACK      
      if ((LnPacket->pt.pcmd&PCMD_RW) == 0 && (LnPacket->pt.pcmd&PCMD_OPS_MODE)==0)
      {
        LocoNet.send(OPC_LONG_ACK,0x7F,1);
        #ifdef DEBUG
        Serial.print("Read on programming track CV "); Serial.println(cvnum);        
        #endif
        /* <R CV CALLBACKNUM CALLBACKSUB>
        *    CV: the number of the Configuration Variable memory location in the decoder to read from (1-1024)
        *    CALLBACKNUM: an arbitrary integer (0-32767) that is ignored by the Base Station and is simply echoed back in the output - useful for external programs that call this function
        *    CALLBACKSUB: a second arbitrary integer (0-32767) that is ignored by the Base Station and is simply echoed back in the output - useful for external programs (e.g. DCC++ Interface) that call this function
        *    
        *    returns: <r CALLBACKNUM|CALLBACKSUB|CV VALUE)
        *    where VALUE is a number from 0-255 as read from the requested CV, or -1 if read could not be verified
        */    
        sprintf(s,"%d 11 12",cvnum);
        
        cvvalue=pRegs->readCV(s);
        // <0xEF>,<0E>,<7C>,<PCMD>,<0>    ,<HOPSA>,<LOPSA>,<TRK>,<CVH>,<CVL>,<DATA7>,<0>,<0>,<CHK>
        // <0xE7>,<0E>,<7C>,<PCMD>,<PSTAT>,<HOPSA>,<LOPSA>,<TRK>,<CVH>,<CVL>,<DATA7>,<0>,<0>,<CHK>
        LnPacket->pt.command=OPC_SL_RD_DATA;
        LnPacket->pt.pstat=0;
        LnPacket->pt.data7=cvvalue;        
        LocoNet.send(LnPacket);
        
        mLcd->setCursor(0,0);             // set the LCD cursor   position 
        mLcd->print("GET CV"); mLcd->print(cvnum); mLcd->print(" = "); mLcd->print(cvvalue);
      }      
      //WRITE ON PROGRAMMING TRACK
      if ((LnPacket->pt.pcmd&PCMD_RW)>0 && (LnPacket->pt.pcmd&PCMD_OPS_MODE)==0)
      {
        LocoNet.send(OPC_LONG_ACK,0x7F,1);
        #ifdef DEBUG
        Serial.print("Write on programming track CV ");Serial.print(cvnum);Serial.print(" VALUE ");Serial.println(cvvalue);                
        #endif
        /* <W CV VALUE CALLBACKNUM CALLBACKSUB>
        *    CV: the number of the Configuration Variable memory location in the decoder to write to (1-1024)
        *    VALUE: the value to be written to the Configuration Variable memory location (0-255) 
        *    CALLBACKNUM: an arbitrary integer (0-32767) that is ignored by the Base Station and is simply echoed back in the output - useful for external programs that call this function
        *    CALLBACKSUB: a second arbitrary integer (0-32767) that is ignored by the Base Station and is simply echoed back in the output - useful for external programs (e.g. DCC++ Interface) that call this function
        *    
        *    returns: <r CALLBACKNUM|CALLBACKSUB|CV Value)
        *    where VALUE is a number from 0-255 as read from the requested CV, or -1 if verificaiton read fails
        */
        sprintf(s,"%d %d 11 11",cvnum,cvvalue);        
        pRegs->writeCVByte(s);
        cvvalue=pRegs->readCV(s);
        LnPacket->pt.command=OPC_SL_RD_DATA;
        LnPacket->pt.pstat=0;
        LnPacket->pt.data7=cvvalue;        
        LocoNet.send(LnPacket);    
        mLcd->setCursor(0,0);             // set the LCD cursor   position 
        mLcd->print("SET CV"); mLcd->print(cvnum); mLcd->print(" = "); mLcd->print(cvvalue);            
      }            
      
      digitalWrite(PROG_RELAY1,LOW);
      digitalWrite(PROG_RELAY2,LOW);
      digitalWrite(PWOFF_LED_PIN, LOW);
      
      break;      

    default:
      // ignore the message...
        #ifdef DEBUG
        Serial.println("# !! IGNORE MESSAGE !! #");            
        #endif
  }

} // LNetCmdStation::processIncomingLoconetCommand

