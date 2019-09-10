
//******************************************************************************
// Code developed for SPIT Mumbai
// This file includes all ISR functions
//******************************************************************************
#include <p33EP64MC506.h>

#include "ISRs.h"
#include "mcc_generated_files/pin_manager.h"
#include "motorFun.h"
 

void enableInterrupts(void)
{
    /* Enable level 1-7 interrupts */
    /* No restoring of previous CPU IPL state performed here */
    INTCON2bits.GIE = 1;
    return;
}

void disableInterrupts(void)
{
    /* Disable level 1-7 interrupts */
    /* No saving of current CPU IPL setting performed here */
    INTCON2bits.GIE = 0;
    return;
}

void initInterrupts(void)
{

    /* Enable CN interrupts */
    IEC1bits.CNIE = 1;
    
    IFS1bits.CNIF = 0; // Reset CN interrupt
    
    return;

    
}

void __attribute__((__interrupt__,no_auto_psv)) _CNInterrupt(void)
{
/* Insert ISR Code Here*/
    if(IO_RG6_SW4_GetValue() == 0){  //If switch SW1 ON
        IO_RD5_HB_LED_SetLow();
        motorDirection = MOTOR_DIR_FORWARD;
    }
    else {
        IO_RD5_HB_LED_SetHigh();
        motorDirection = MOTOR_DIR_REVERSE;
    }
    
    motorControlMode = (IO_RG9_SW1_GetValue() << 2) 
                     + (IO_RA11_SW2_GetValue() << 1) 
                     + (IO_RA12_SW3_GetValue());

/* Clear CN interrupt */
    IFS1bits.CNIF = 0;
}