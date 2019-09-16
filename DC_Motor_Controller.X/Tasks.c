//******************************************************************************
// Code developed for SPIT Mumbai
// This file includes all functions related to mutliTasking kernel
//******************************************************************************
#include "Tasks.h"
#include "mcc_generated_files/pin_manager.h"
#include "BizLogic.h"
#include "motorFun.h"
#include "MODBUS.h"

uCHAR cnt1ms  = 0;
uCHAR cnt10ms = 0;
uCHAR cnt25ms = 0;
uCHAR cnt100ms = 0;


//******************************************************************************
// TimerCallBack 
// Input - None
// This function is the callback of timer1 and called after every 1ms duration 
//******************************************************************************
void TimerCallBack (void)  
{
    volatile uINT lclEventReg = 0;

    // Read ADC is the 100us default Task
    readAllAnalogVariables();
    
    
    
    if(++cnt1ms > 9)
    {
        cnt1ms = 0;
        eventRegister |= EVENT_TIMER_1ms_TASK;
    }
        
    if(++cnt10ms > 99)
    {        
        cnt10ms = 0;
        eventRegister |= EVENT_TIMER_10ms_TASK;
    }
        
    if(++cnt25ms > 249)
    {
        cnt25ms = 0;
        eventRegister |= EVENT_TIMER_25ms_TASK;
    }    
   
    lclEventReg = eventRegister;    // Must be atomic 
    
    if(lclEventReg & EVENT_MODBUS_TASK)
    {
        lclEventReg &= ~EVENT_MODBUS_TASK; // Reset the Event Bit
        MODBUSDecodeTask();
    }
    
    if(lclEventReg & EVENT_CAN_COM_TASK)
    {
        lclEventReg &= ~EVENT_CAN_COM_TASK; // Reset the Event Bit
        CANBUSDecodeTask();
    } 
    
    if(lclEventReg & EVENT_ETHERNATE_TASK)
    {
        lclEventReg &= ~EVENT_ETHERNATE_TASK; // Reset the Event Bit
        EthernetDecodeTask();
    } 
    
    if(lclEventReg & EVENT_TIMER_1ms_TASK)
    {
        lclEventReg &= ~EVENT_TIMER_1ms_TASK; // Reset the Event Bit
        Timer1msTask();
    } 
    
    if(lclEventReg & EVENT_TIMER_10ms_TASK)
    {
        lclEventReg &= ~EVENT_TIMER_10ms_TASK; // Reset the Event Bit
        Timer10msTask();
    }  
    
    if(lclEventReg & EVENT_TIMER_25ms_TASK)
    {
        lclEventReg &= ~EVENT_TIMER_25ms_TASK; // Reset the Event Bit
        Timer25msTask();
    }     
    eventRegister = lclEventReg;    // Must be atomic
}
//******************************************************************************


//******************************************************************************
// Timer1msTask 
// Input - None
// This function will execute the functions which requires 1ms duration 
//******************************************************************************
void Timer1msTask (void)
{
    if(ss_duty_count == 600){
        runMotorWithControl();
    }
    
}
//******************************************************************************


//******************************************************************************
// Timer10msTask 
// Input - None
// This function will execute the functions which requires 10ms duration 
//******************************************************************************
void Timer10msTask (void)
{
    //IO_RD5_HB_LED_Toggle();   
    if(ss_duty_count < 600) {
        runMotor(motorDirection, ss_duty_count);
        ss_duty_count += 2;
    }
    
}
//******************************************************************************


//******************************************************************************
// Timer25msTask 
// Input - None
// This function will execute the functions which requires 25ms duration 
//******************************************************************************
void Timer25msTask (void)
{
   //IO_RD5_HB_LED_Toggle();   
    cnt100ms++;
    if(cnt100ms == 4){
        read_encoder_velocity();
        
        cnt100ms = 0;
    }
    
}
//******************************************************************************


//******************************************************************************
// Timer1msTask 
// Input - None
// This function is event based function 
//******************************************************************************
void MODBUSDecodeTask (void)
{
    decodeRecieveMessage();
} 
//******************************************************************************


//******************************************************************************
// CANBUSDecodeTask 
// Input - None
// This function is event based function 
//******************************************************************************
void CANBUSDecodeTask (void)
{
    
}
//******************************************************************************


//******************************************************************************
// CANBUSDecodeTask 
// Input - None
// This function is event based function 
//******************************************************************************
void EthernetDecodeTask (void)
{
    
}
//******************************************************************************
