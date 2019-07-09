//******************************************************************************
// Code developed for SPIT Mumbai
// This file includes all functions related to MODBUS Protocol
//******************************************************************************

//    
//------------------------------------------------------------------------------     
// MODBUS Protocol - Read Holding Registers
//------------------------------------------------------------------------------
//                  ADDRESS - Fun Code - Start Address - Num of Reg  - CRC
// Master->Slave    0x80    -  0x03    - HByte-LByte   - HByte-LByte - CRC
// -----------------------------------------------------------------------------    
//                  ADDRESS - Fun Code - DataBytes - Data (n/2) - CRC    
// Slave->Master    0x80    -  0x03    - 2*NumReg  - Data       - crcLo-crcHi 
//------------------------------------------------------------------------------
    
//------------------------------------------------------------------------------    
// MODBUS Protocol - Preset Single Registers    
// -----------------------------------------------------------------------------
//                  ADDRESS - Fun Code - Start Address - Num of Reg  - NumBytes - Data  - CRC
// Master->Slave    0x80    -  0x06    - HByte-LByte   - HByte-LByte - Bytes    - Bytes - CRC
// -----------------------------------------------------------------------------    
//                  ADDRESS - Fun Code - Start Address - Num of Reg  - CRC 
// Slave->Master    0x80    -  0x06    - HByte-LByte   - HByte-LByte - CRC
//------------------------------------------------------------------------------
//

#include "MODBUS.h"
#include "Tasks.h"
#include "string.h"
#include "BizLogic.h"
#include "mcc_generated_files/pin_manager.h"
#include "mcc_generated_files/uart1.h"
#include "mcc_generated_files/tmr2.h"

#define HEADER_BYTE     (uCHAR)1
#define FUNC_BYTE       (uCHAR)1
#define ADDR_BYTES      (uCHAR)2
#define REG_BYTES       (uCHAR)2
#define NO_BYTE         (uCHAR)1
#define CRC_BYTES       (uCHAR)2

#define NO_HANDLE       (uCHAR)0
#define MODBUS_HANDLE   (uCHAR)1
#define CANBUS_HANDLE   (uCHAR)2

#define READ_ONLY_REGS  (uCHAR)15    // only 15 varibales are read only
#define READ_WRITE_REGS (uCHAR)15    // only 15 varibales are read-Write only

#define READ_ONLY_VAR_OFFSET    (uINT)0
#define READ_WRITE_VAR_OFFSET   (uINT)20

uINT  memAddressROvars[15];     // Read only Variables
uINT  memAddressRWvars[15];     // Read/Write Variables

uCHAR sizeOfROVar[15];          // size RO vars 
uCHAR sizeOfRWVar[15];          // size RO vars

uCHAR handleTimeOut = NO_HANDLE;

uCHAR delay;



//******************************************************************************
// decodeRecieveMessage
//******************************************************************************
void decodeRecieveMessage (void)
{
    volatile uINT datalen = 0;
    volatile uINT msgCrc  = 0;
    volatile uINT calcCRC = 0;
    volatile uINT address = 0;
    volatile uINT noOfReg = 0;
    volatile sINT AddrOffset    = 0;    
    volatile uINT transmitBytes = 0;
    volatile uINT varAddress = 0;
    volatile uINT varSize    = 0;
    //volatile 
    
    volatile uCHAR i = 0;
  
    // Stop Timeout Timer
    TMR2_Stop(); 
    
    datalen = (MAX_TXRX_BUFF_LENGHT - UART1_ReceiveBufferSizeGet());    
    UART1_ReadBuffer(&messageBuffer[0], datalen);
    
    // Create CRC from Message - [(crcHi << 8) | crcLo]
    msgCrc  = (uINT)((messageBuffer[datalen-1]<<8) | ((uINT)messageBuffer[datalen-2]));     
    calcCRC =  ModBus_CRC16(&messageBuffer[0], (datalen-2));
           
    if((calcCRC == msgCrc) && (messageBuffer[0] == THIS_DEVICE_ADDRESS))
    {      
        address = (uINT)((messageBuffer[2]<<8) | ((uINT)messageBuffer[3]));
        noOfReg = (uINT)((messageBuffer[4]<<8) | ((uINT)messageBuffer[5]));                
        
        if(messageBuffer[1] == READ_HOLDING_REGISTERS)
        {          
            // Keep Slave Address and Function ID and Clear all 
            clearBuffer(&messageBuffer[0], 2, MAX_TXRX_BUFF_LENGHT); 
        
            if(address <= (READ_ONLY_VAR_OFFSET+READ_ONLY_REGS))
            {
                AddrOffset = address;
                
                if(AddrOffset < -1) 
                   AddrOffset = 0;
                
                if(AddrOffset > READ_ONLY_REGS)
                   AddrOffset = READ_ONLY_REGS;
                
                if(noOfReg >= READ_ONLY_REGS)
                    noOfReg = READ_ONLY_REGS;                       
                                                  
                transmitBytes = 0;
                for(i=0; i<noOfReg; i++)
                {
                    varAddress = memAddressROvars[AddrOffset+i]; 
                    varSize    = sizeOfROVar[i];
                    transmitBytes += varSize;
                            
                    memcpy( (uCHAR *)      &messageBuffer[3+i*varSize], 
                            (const uCHAR *)varAddress, 
                            (size_t)       varSize);  
                }
            }
            else
            {
                if(address <= (READ_WRITE_VAR_OFFSET+READ_WRITE_REGS))
                {
                    // Read/Write Only
                    AddrOffset = (sINT)(address - READ_WRITE_VAR_OFFSET);
                    
                    if(AddrOffset < -1) 
                       AddrOffset = 0;

                    if(AddrOffset > READ_WRITE_REGS)
                       AddrOffset = READ_WRITE_REGS;
                    
                    if(noOfReg >= READ_WRITE_VAR_OFFSET)
                        transmitBytes = (uINT)(READ_WRITE_VAR_OFFSET<<1);
                    
                    if(noOfReg >= READ_WRITE_VAR_OFFSET)
                        noOfReg = READ_WRITE_VAR_OFFSET;

                    transmitBytes = 0;
                    for(i=0; i<noOfReg; i++)
                    {
                        varAddress = memAddressRWvars[AddrOffset+i]; 
                        varSize    = sizeOfRWVar[i];
                        transmitBytes += varSize;

                        memcpy( (uCHAR *)      &messageBuffer[3+i*varSize], 
                                (const uCHAR *)varAddress, 
                                (size_t)       varSize);  
                    }                    
                }
            }

            messageBuffer[2]  = transmitBytes; 

            // Add CRC to the String
            calcCRC = ModBus_CRC16(&messageBuffer[0], (transmitBytes+3)); 

            messageBuffer[transmitBytes+3] = (uCHAR)calcCRC;
            messageBuffer[transmitBytes+4] = (uCHAR)(calcCRC>>8);                               

            // Transmit data
            transmitBuffer (&messageBuffer[0], (transmitBytes+5));
            //PIN_RS485_DIR  = TRANSMIT;
            /*IO_RA4_RS485_DIR_SetHigh();    
            for(i=0; i<=(transmitBytes+4); i++)
            {
                UART1_Write((const uCHAR) messageBuffer[i]); 
                Delay_us(1000);
            }                
            IO_RA4_RS485_DIR_SetLow();*/
            
        }
        else if(messageBuffer[1] == PRESET_SINGLE_REGISTER)
        {
            if(address <= (READ_WRITE_VAR_OFFSET+READ_WRITE_REGS))
            {
                // Read Only
                AddrOffset = (sINT)(address - READ_WRITE_VAR_OFFSET);
                if(AddrOffset < -1) 
                   AddrOffset = 0;

                if(AddrOffset > READ_WRITE_REGS)
                   AddrOffset = READ_WRITE_REGS;                              
                
                memcpy( (uCHAR *) memAddressRWvars[AddrOffset], 
                        (const uCHAR *)&messageBuffer[4] , 
                        (size_t)sizeOfRWVar[AddrOffset]);                
                    
                transmitBuffer (&messageBuffer[0], datalen);
                
                //PIN_RS485_DIR  = TRANSMIT;                 
                //for(i=0; i<datalen; i++)
                //{
                //    UART1_Write((const uCHAR) messageBuffer[i]); 
                //    Delay_us(1000);
                //}                
                //PIN_RS485_DIR  = RECEIVE;             
            }           
        }
        else
        {
            clearBuffer(&messageBuffer[0], 0, MAX_TXRX_BUFF_LENGHT);
        }
    }
    else
    {
        clearBuffer(&messageBuffer[0], 0, MAX_TXRX_BUFF_LENGHT);
    }
    
    clearBuffer(&messageBuffer[0], 0, MAX_TXRX_BUFF_LENGHT);      
    ENABLE_RX_ISR;
}
//******************************************************************************


//******************************************************************************
// transmitBuffer 
// Input - Buffer Pointer and Send Length
// This function is used to transmit the desired buffer to the PC
//******************************************************************************
void transmitBuffer (uCHAR *buf, uCHAR bufLen)
{
    volatile uINT bytesWritten   = 0;
    UART1_TRANSFER_STATUS status = 0;
    
    static uINT tmCounter = 0;
    
    IO_RA4_RS485_DIR_SetHigh();    
    bytesWritten = UART1_WriteBuffer( (const uint8_t *)&buf[0] , bufLen );
    
    tmCounter = 0;    
    while(1)
    {
        status = UART1_TransferStatusGet();
        
        if(status &= UART1_TRANSFER_STATUS_TX_EMPTY)
            break;
        
        if(++tmCounter > 50000) break;  // For Timeout only 
    }    
    IO_RA4_RS485_DIR_SetLow();
}
//******************************************************************************



//******************************************************************************
// stateMachineForReceiveFrame
//******************************************************************************
void stateMachineForReceiveFrame (uCHAR recByte)
{
    static uCHAR byteRcvCounter = 0;
    static uCHAR reqPresetRegs  = NO;

    // Start Timer here
    TMR2_Start();
    handleTimeOut = MODBUS_HANDLE;         
    
    //byteRcvCounter = 0
    if((byteRcvCounter == 1))
    {
        if((recByte == READ_HOLDING_REGISTERS) || 
           (recByte == PRESET_SINGLE_REGISTER))  
        {
            reqPresetRegs = NO;
            receiveByteLen = HEADER_BYTE+FUNC_BYTE+ADDR_BYTES+REG_BYTES+CRC_BYTES;   // Total 8 bytes
        }
        else if (recByte == PRESET_MULTIPLE_REGISTERS)
        {
            reqPresetRegs  = YES;
            receiveByteLen = HEADER_BYTE+FUNC_BYTE+ADDR_BYTES+REG_BYTES+NO_BYTE+CRC_BYTES;
        }            
        else
        {
            reqPresetRegs = NO;
            receiveByteLen = 5; // Total 
        }            
    }
    
    if((reqPresetRegs == YES) && (byteRcvCounter == 6)) 
    {
        reqPresetRegs = NO;
        receiveByteLen += recByte;  // Align the receiveByteLen according data
    }
    
    if(++byteRcvCounter >= receiveByteLen)
    {
        byteRcvCounter = 0;            
        reqPresetRegs  = NO;
        eventRegister = EVENT_MODBUS_TASK; 
        
        flagSlaveAdd = NO;
        
        // Stop Serial ISR to get further byte
        DISBLE_RX_ISR;
    }        
}
//******************************************************************************

//******************************************************************************
// stateMachineForReceiveFrame
//******************************************************************************
void Timer2CallBack (void)
{
    volatile uCHAR i = 0;
    
    TMR2_Stop(); 
    
    if(handleTimeOut == MODBUS_HANDLE)
    {
        receiveByteLen = 8;
        for(i=0; i<MAX_TXRX_BUFF_LENGHT; i++)
        {
            messageBuffer[i] = 0;             
        }
        
        ENABLE_RX_ISR;
    }
}
//******************************************************************************


//******************************************************************************
// stateMachineForReceiveFrame
//******************************************************************************
void clearBuffer(uCHAR *buffer, uCHAR index, uCHAR bufLen)
{
    volatile uCHAR i = 0;
    for(i=index; i<bufLen; i++)
    {
        buffer[i] = 0;
    }
}
//******************************************************************************


//******************************************************************************
// Modbus CRC Generation Code
//******************************************************************************
uINT ModBus_CRC16(uCHAR * Buffer, uINT Length)
{
    volatile uINT cur_crc = 0, i;
    
    cur_crc = 0xFFFF;

    do
    {
        i  = 8;
        cur_crc = (cur_crc ^ *Buffer++);
        do
        {
            if (0x0001 & cur_crc)
            {
                cur_crc >>= 1;
                cur_crc ^= 0xA001;
            }
            else
            {
                cur_crc >>= 1;
            }
        }
        while (--i);
    }
    while (--Length);
    
    return cur_crc;
}
//******************************************************************************



//******************************************************************************
// Modbus Table Creation for the Communication
//******************************************************************************
void createTableForMODBUS (void)
{
    volatile uCHAR i = 0;
    
    //
    // Read Only Variables
    //
    i = 0;
    
    memAddressROvars[i] = (uINT) ((uINT *)&motorActualRPM);    
    sizeOfROVar[i]      = sizeof(motorActualRPM);
    i++;      
    
    memAddressROvars[i] = (uINT) ((uINT *)&adcInputVoltage);    
    sizeOfROVar[i]      = sizeof(adcInputVoltage);    
    i++; 
    
    memAddressROvars[i] = (uINT) ((uINT *)&dcBusCurrent);    
    sizeOfROVar[i]      = sizeof(dcBusCurrent);    
    i++; 
    
    memAddressROvars[i] = (uINT) ((uINT *)&adcMotorVoltage);    
    sizeOfROVar[i]      = sizeof(adcMotorVoltage);    
    i++; 

    memAddressROvars[i] = (uINT) ((uINT *)&adcPLCinputVoltage);    
    sizeOfROVar[i]      = sizeof(adcPLCinputVoltage);    
    i++; 

    memAddressROvars[i] = (uINT) ((sINT *)&adcInternalTemp);    
    sizeOfROVar[i]      = sizeof(adcInternalTemp);    
    i++; 

    memAddressROvars[i] = (uINT) ((uINT *)&adcTachoInput);    
    sizeOfROVar[i]      = sizeof(adcTachoInput);    
    i++; 
    
    // adc_externalPOT
    memAddressROvars[i] = (uINT) ((uINT *)&adcPotInput);    
    sizeOfROVar[i]      = sizeof(adcPotInput);
    i++;  
        
    
    //
    // RW Variables
    //
    i = 0;
    
    memAddressRWvars[i] = (uINT) ((uINT *)&motorSetRPM);    
    sizeOfRWVar[i]      = sizeof(motorSetRPM);
    i++; 
    
    memAddressRWvars[i] = (uINT) ((uINT *)&speed_Kp);    
    sizeOfRWVar[i]      = sizeof(speed_Kp);
    i++; 
    
    memAddressRWvars[i] = (uINT) ((uINT *)&speed_Ki);    
    sizeOfRWVar[i]      = sizeof(speed_Ki);
    i++; 

    memAddressRWvars[i] = (uINT) ((uINT *)&Torque_Kp);    
    sizeOfRWVar[i]      = sizeof(Torque_Kp);
    i++; 

    memAddressRWvars[i] = (uINT) ((uINT *)&Torque_Ki);    
    sizeOfRWVar[i]      = sizeof(Torque_Ki);
    i++;  
    
    // controlMode
    memAddressRWvars[i] = (uINT) ((uINT *)&motorControlMode);    
    sizeOfRWVar[i]      = sizeof(motorControlMode);
    i++; 
}
//******************************************************************************


