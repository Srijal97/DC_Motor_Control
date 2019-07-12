/**
  Generated main.c file from MPLAB Code Configurator

  @Company
    Microchip Technology Inc.

  @File Name
    main.c

  @Summary
    This is the generated main.c using PIC24 / dsPIC33 / PIC32MM MCUs.

  @Description
    This source file provides main entry point for system intialization and application code development.
    Generation Information :
        Product Revision  :  PIC24 / dsPIC33 / PIC32MM MCUs - 1.95-b-SNAPSHOT
        Device            :  dsPIC33EP64MC506
    The generated drivers are tested against the following:
        Compiler          :  XC16 v1.36
        MPLAB 	          :  MPLAB X v5.10
*/

/*
    (c) 2016 Microchip Technology Inc. and its subsidiaries. You may use this
    software and any derivatives exclusively with Microchip products.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
    WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
    PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION
    WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
    BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
    FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
    ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
    THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.

    MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE
    TERMS.
*/

/**
  Section: Included Files
*/

// 70.0416 MHz Running
#include "mcc_generated_files/system.h"
#include "mcc_generated_files/pin_manager.h"
#include "mcc_generated_files/pwm.h"
#include "mcc_generated_files/tmr1.h"
#include "Macros.h"
#include "projMacros.h" 
#include "motorFun.h"
#include "globals.h"
#include "Tasks.h"
#include "BizLogic.h"
#include "CANProtocol.h"
#include "MODBUS.h"


/*
                         Main application
 */


int main(void)
{
    // initialize the device
    SYSTEM_Initialize();
    PWM_ModuleEnable();
    
    PWM_DutyCycleSet(PWM_GENERATOR_1, 0);
    PWM_DutyCycleSet(PWM_GENERATOR_2, 0);
    
    createTableForMODBUS();
    
    IO_RA4_RS485_DIR_SetLow();
    
    TMR1_Start();   // Kernel Timer Start
    
    //int a = sizeof(uLONG);

    while (1)
    {
        
        if(IO_RG9_SW1_GetValue() == 0){  //If switch SW1 ON
            IO_RD5_HB_LED_SetLow();
            motorDirection = MOTOR_DIR_FORWARD;
        }
        else {
            IO_RD5_HB_LED_SetHigh();
            motorDirection = MOTOR_DIR_REVERSE;
        }
        
        
        TMR1_Tasks_16BitOperation();    // 100uS Timer function
        
        // Add your application code
        //IO_RD5_HB_LED_Toggle();
        //IO_RC9_ETH_RST_Toggle();
        //IO_RB0_ETH_CS_Toggle();
        //IO_RA10_CAN_SI_Toggle();
        //IO_RA4_RS485_DIR_Toggle();
        //IO_RB11_DBU_PIN_Toggle();
        
        //if(IO_RA12_SW3_GetValue() == 0)
            //IO_RD5_HB_LED_SetHigh();
        //else
            //IO_RD5_HB_LED_SetLow();   
        
        
        //for(i=0; i<6000; i++);
    }
    return 1; 
}


/**
 End of File
*/

