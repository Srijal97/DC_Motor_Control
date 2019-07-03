/* 
 * File:   MODBUS.h
 * Author: Eshu
 *
 * Created on May 12, 2019, 1:08 PM
 */

//    
//------------------------------------------------------------------------------     
// MODBUS Protocol - Read Holding Registers
//------------------------------------------------------------------------------
//                  ADDRESS - Fun Code - Start Address - Num of Reg  - CRC
// Master->Slave    0x80    -  0x03    - HByte-LByte   - HByte-LByte - CRC
// -----------------------------------------------------------------------------    
//                  ADDRESS - Fun Code - DataBytes - Data (n/2) - CRC    
// Slave->Master    0x80    -  0x03    - 2*NumReg  - Data       - CRC 
//------------------------------------------------------------------------------
    
//------------------------------------------------------------------------------    
// MODBUS Protocol - Preset Multiple Registers    
// -----------------------------------------------------------------------------
//                  ADDRESS - Fun Code - Start Address - Num of Reg  - NumBytes - Data  - CRC
// Master->Slave    0x80    -  0x10    - HByte-LByte   - HByte-LByte - Bytes    - Bytes - CRC
// -----------------------------------------------------------------------------    
//                  ADDRESS - Fun Code - Start Address - Num of Reg  - CRC 
// Slave->Master    0x80    -  0x10    - HByte-LByte   - HByte-LByte - CRC
//------------------------------------------------------------------------------
//

#ifndef MODBUS_H
#define	MODBUS_H

#ifdef	__cplusplus
extern "C" {
#endif

#include "Macros.h"
#include "projMacros.h"


#ifdef	__cplusplus
}
#endif

    
#define ENABLE_RX_ISR               IEC0bits.U1RXIE = 1;  
#define DISBLE_RX_ISR               IEC0bits.U1RXIE = 0;  

// Variables
extern uCHAR messageBuffer[MAX_TXRX_BUFF_LENGHT]; // 100bytes 
extern uCHAR receiveByteLen;
extern uINT  modbusTimeoutCounter;

extern uCHAR flagSlaveAdd; //= NO;


//functions
void createTableForMODBUS   (void);
uINT ModBus_CRC16           (uCHAR * Buffer, uINT Length);

void stateMachineForReceiveFrame    (uCHAR recByte);
void decodeRecieveMessage           (void);

void Timer2CallBack (void);
void clearBuffer(uCHAR *buffer, uCHAR index, uCHAR bufLen);

void transmitBuffer (uCHAR *buf, uCHAR bufLen);


#endif	/* MODBUS_H */

