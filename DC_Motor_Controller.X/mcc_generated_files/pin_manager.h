/**
  PIN MANAGER Generated Driver File

  @Company:
    Microchip Technology Inc.

  @File Name:
    pin_manager.h

  @Summary:
    This is the generated manager file for the PIC24 / dsPIC33 / PIC32MM MCUs device.  This manager
    configures the pins direction, initial state, analog setting.
    The peripheral pin select, PPS, configuration is also handled by this manager.

  @Description:
    This source file provides implementations for PIN MANAGER.
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

#ifndef _PIN_MANAGER_H
#define _PIN_MANAGER_H
/**
    Section: Includes
*/
#include <xc.h>

/**
    Section: Device Pin Macros
*/
/**
  @Summary
    Sets the GPIO pin, RA0, high using LATA0.

  @Description
    Sets the GPIO pin, RA0, high using LATA0.

  @Preconditions
    The RA0 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RA0 high (1)
    channel_AN0_SetHigh();
    </code>

*/
#define channel_AN0_SetHigh()          _LATA0 = 1
/**
  @Summary
    Sets the GPIO pin, RA0, low using LATA0.

  @Description
    Sets the GPIO pin, RA0, low using LATA0.

  @Preconditions
    The RA0 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RA0 low (0)
    channel_AN0_SetLow();
    </code>

*/
#define channel_AN0_SetLow()           _LATA0 = 0
/**
  @Summary
    Toggles the GPIO pin, RA0, using LATA0.

  @Description
    Toggles the GPIO pin, RA0, using LATA0.

  @Preconditions
    The RA0 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RA0
    channel_AN0_Toggle();
    </code>

*/
#define channel_AN0_Toggle()           _LATA0 ^= 1
/**
  @Summary
    Reads the value of the GPIO pin, RA0.

  @Description
    Reads the value of the GPIO pin, RA0.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RA0
    postValue = channel_AN0_GetValue();
    </code>

*/
#define channel_AN0_GetValue()         _RA0
/**
  @Summary
    Configures the GPIO pin, RA0, as an input.

  @Description
    Configures the GPIO pin, RA0, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RA0 as an input
    channel_AN0_SetDigitalInput();
    </code>

*/
#define channel_AN0_SetDigitalInput()  _TRISA0 = 1
/**
  @Summary
    Configures the GPIO pin, RA0, as an output.

  @Description
    Configures the GPIO pin, RA0, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RA0 as an output
    channel_AN0_SetDigitalOutput();
    </code>

*/
#define channel_AN0_SetDigitalOutput() _TRISA0 = 0
/**
  @Summary
    Sets the GPIO pin, RA1, high using LATA1.

  @Description
    Sets the GPIO pin, RA1, high using LATA1.

  @Preconditions
    The RA1 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RA1 high (1)
    channel_AN1_SetHigh();
    </code>

*/
#define channel_AN1_SetHigh()          _LATA1 = 1
/**
  @Summary
    Sets the GPIO pin, RA1, low using LATA1.

  @Description
    Sets the GPIO pin, RA1, low using LATA1.

  @Preconditions
    The RA1 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RA1 low (0)
    channel_AN1_SetLow();
    </code>

*/
#define channel_AN1_SetLow()           _LATA1 = 0
/**
  @Summary
    Toggles the GPIO pin, RA1, using LATA1.

  @Description
    Toggles the GPIO pin, RA1, using LATA1.

  @Preconditions
    The RA1 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RA1
    channel_AN1_Toggle();
    </code>

*/
#define channel_AN1_Toggle()           _LATA1 ^= 1
/**
  @Summary
    Reads the value of the GPIO pin, RA1.

  @Description
    Reads the value of the GPIO pin, RA1.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RA1
    postValue = channel_AN1_GetValue();
    </code>

*/
#define channel_AN1_GetValue()         _RA1
/**
  @Summary
    Configures the GPIO pin, RA1, as an input.

  @Description
    Configures the GPIO pin, RA1, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RA1 as an input
    channel_AN1_SetDigitalInput();
    </code>

*/
#define channel_AN1_SetDigitalInput()  _TRISA1 = 1
/**
  @Summary
    Configures the GPIO pin, RA1, as an output.

  @Description
    Configures the GPIO pin, RA1, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RA1 as an output
    channel_AN1_SetDigitalOutput();
    </code>

*/
#define channel_AN1_SetDigitalOutput() _TRISA1 = 0
/**
  @Summary
    Sets the GPIO pin, RA10, high using LATA10.

  @Description
    Sets the GPIO pin, RA10, high using LATA10.

  @Preconditions
    The RA10 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RA10 high (1)
    IO_RA10_CAN_SI_SetHigh();
    </code>

*/
#define IO_RA10_CAN_SI_SetHigh()          _LATA10 = 1
/**
  @Summary
    Sets the GPIO pin, RA10, low using LATA10.

  @Description
    Sets the GPIO pin, RA10, low using LATA10.

  @Preconditions
    The RA10 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RA10 low (0)
    IO_RA10_CAN_SI_SetLow();
    </code>

*/
#define IO_RA10_CAN_SI_SetLow()           _LATA10 = 0
/**
  @Summary
    Toggles the GPIO pin, RA10, using LATA10.

  @Description
    Toggles the GPIO pin, RA10, using LATA10.

  @Preconditions
    The RA10 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RA10
    IO_RA10_CAN_SI_Toggle();
    </code>

*/
#define IO_RA10_CAN_SI_Toggle()           _LATA10 ^= 1
/**
  @Summary
    Reads the value of the GPIO pin, RA10.

  @Description
    Reads the value of the GPIO pin, RA10.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RA10
    postValue = IO_RA10_CAN_SI_GetValue();
    </code>

*/
#define IO_RA10_CAN_SI_GetValue()         _RA10
/**
  @Summary
    Configures the GPIO pin, RA10, as an input.

  @Description
    Configures the GPIO pin, RA10, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RA10 as an input
    IO_RA10_CAN_SI_SetDigitalInput();
    </code>

*/
#define IO_RA10_CAN_SI_SetDigitalInput()  _TRISA10 = 1
/**
  @Summary
    Configures the GPIO pin, RA10, as an output.

  @Description
    Configures the GPIO pin, RA10, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RA10 as an output
    IO_RA10_CAN_SI_SetDigitalOutput();
    </code>

*/
#define IO_RA10_CAN_SI_SetDigitalOutput() _TRISA10 = 0
/**
  @Summary
    Sets the GPIO pin, RA11, high using LATA11.

  @Description
    Sets the GPIO pin, RA11, high using LATA11.

  @Preconditions
    The RA11 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RA11 high (1)
    IO_RA11_SW2_SetHigh();
    </code>

*/
#define IO_RA11_SW2_SetHigh()          _LATA11 = 1
/**
  @Summary
    Sets the GPIO pin, RA11, low using LATA11.

  @Description
    Sets the GPIO pin, RA11, low using LATA11.

  @Preconditions
    The RA11 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RA11 low (0)
    IO_RA11_SW2_SetLow();
    </code>

*/
#define IO_RA11_SW2_SetLow()           _LATA11 = 0
/**
  @Summary
    Toggles the GPIO pin, RA11, using LATA11.

  @Description
    Toggles the GPIO pin, RA11, using LATA11.

  @Preconditions
    The RA11 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RA11
    IO_RA11_SW2_Toggle();
    </code>

*/
#define IO_RA11_SW2_Toggle()           _LATA11 ^= 1
/**
  @Summary
    Reads the value of the GPIO pin, RA11.

  @Description
    Reads the value of the GPIO pin, RA11.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RA11
    postValue = IO_RA11_SW2_GetValue();
    </code>

*/
#define IO_RA11_SW2_GetValue()         _RA11
/**
  @Summary
    Configures the GPIO pin, RA11, as an input.

  @Description
    Configures the GPIO pin, RA11, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RA11 as an input
    IO_RA11_SW2_SetDigitalInput();
    </code>

*/
#define IO_RA11_SW2_SetDigitalInput()  _TRISA11 = 1
/**
  @Summary
    Configures the GPIO pin, RA11, as an output.

  @Description
    Configures the GPIO pin, RA11, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RA11 as an output
    IO_RA11_SW2_SetDigitalOutput();
    </code>

*/
#define IO_RA11_SW2_SetDigitalOutput() _TRISA11 = 0
/**
  @Summary
    Sets the GPIO pin, RA12, high using LATA12.

  @Description
    Sets the GPIO pin, RA12, high using LATA12.

  @Preconditions
    The RA12 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RA12 high (1)
    IO_RA12_SW3_SetHigh();
    </code>

*/
#define IO_RA12_SW3_SetHigh()          _LATA12 = 1
/**
  @Summary
    Sets the GPIO pin, RA12, low using LATA12.

  @Description
    Sets the GPIO pin, RA12, low using LATA12.

  @Preconditions
    The RA12 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RA12 low (0)
    IO_RA12_SW3_SetLow();
    </code>

*/
#define IO_RA12_SW3_SetLow()           _LATA12 = 0
/**
  @Summary
    Toggles the GPIO pin, RA12, using LATA12.

  @Description
    Toggles the GPIO pin, RA12, using LATA12.

  @Preconditions
    The RA12 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RA12
    IO_RA12_SW3_Toggle();
    </code>

*/
#define IO_RA12_SW3_Toggle()           _LATA12 ^= 1
/**
  @Summary
    Reads the value of the GPIO pin, RA12.

  @Description
    Reads the value of the GPIO pin, RA12.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RA12
    postValue = IO_RA12_SW3_GetValue();
    </code>

*/
#define IO_RA12_SW3_GetValue()         _RA12
/**
  @Summary
    Configures the GPIO pin, RA12, as an input.

  @Description
    Configures the GPIO pin, RA12, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RA12 as an input
    IO_RA12_SW3_SetDigitalInput();
    </code>

*/
#define IO_RA12_SW3_SetDigitalInput()  _TRISA12 = 1
/**
  @Summary
    Configures the GPIO pin, RA12, as an output.

  @Description
    Configures the GPIO pin, RA12, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RA12 as an output
    IO_RA12_SW3_SetDigitalOutput();
    </code>

*/
#define IO_RA12_SW3_SetDigitalOutput() _TRISA12 = 0
/**
  @Summary
    Sets the GPIO pin, RA4, high using LATA4.

  @Description
    Sets the GPIO pin, RA4, high using LATA4.

  @Preconditions
    The RA4 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RA4 high (1)
    IO_RA4_RS485_DIR_SetHigh();
    </code>

*/
#define IO_RA4_RS485_DIR_SetHigh()          _LATA4 = 1
/**
  @Summary
    Sets the GPIO pin, RA4, low using LATA4.

  @Description
    Sets the GPIO pin, RA4, low using LATA4.

  @Preconditions
    The RA4 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RA4 low (0)
    IO_RA4_RS485_DIR_SetLow();
    </code>

*/
#define IO_RA4_RS485_DIR_SetLow()           _LATA4 = 0
/**
  @Summary
    Toggles the GPIO pin, RA4, using LATA4.

  @Description
    Toggles the GPIO pin, RA4, using LATA4.

  @Preconditions
    The RA4 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RA4
    IO_RA4_RS485_DIR_Toggle();
    </code>

*/
#define IO_RA4_RS485_DIR_Toggle()           _LATA4 ^= 1
/**
  @Summary
    Reads the value of the GPIO pin, RA4.

  @Description
    Reads the value of the GPIO pin, RA4.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RA4
    postValue = IO_RA4_RS485_DIR_GetValue();
    </code>

*/
#define IO_RA4_RS485_DIR_GetValue()         _RA4
/**
  @Summary
    Configures the GPIO pin, RA4, as an input.

  @Description
    Configures the GPIO pin, RA4, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RA4 as an input
    IO_RA4_RS485_DIR_SetDigitalInput();
    </code>

*/
#define IO_RA4_RS485_DIR_SetDigitalInput()  _TRISA4 = 1
/**
  @Summary
    Configures the GPIO pin, RA4, as an output.

  @Description
    Configures the GPIO pin, RA4, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RA4 as an output
    IO_RA4_RS485_DIR_SetDigitalOutput();
    </code>

*/
#define IO_RA4_RS485_DIR_SetDigitalOutput() _TRISA4 = 0
/**
  @Summary
    Sets the GPIO pin, RB0, high using LATB0.

  @Description
    Sets the GPIO pin, RB0, high using LATB0.

  @Preconditions
    The RB0 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RB0 high (1)
    IO_RB0_ETH_CS_SetHigh();
    </code>

*/
#define IO_RB0_ETH_CS_SetHigh()          _LATB0 = 1
/**
  @Summary
    Sets the GPIO pin, RB0, low using LATB0.

  @Description
    Sets the GPIO pin, RB0, low using LATB0.

  @Preconditions
    The RB0 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RB0 low (0)
    IO_RB0_ETH_CS_SetLow();
    </code>

*/
#define IO_RB0_ETH_CS_SetLow()           _LATB0 = 0
/**
  @Summary
    Toggles the GPIO pin, RB0, using LATB0.

  @Description
    Toggles the GPIO pin, RB0, using LATB0.

  @Preconditions
    The RB0 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RB0
    IO_RB0_ETH_CS_Toggle();
    </code>

*/
#define IO_RB0_ETH_CS_Toggle()           _LATB0 ^= 1
/**
  @Summary
    Reads the value of the GPIO pin, RB0.

  @Description
    Reads the value of the GPIO pin, RB0.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RB0
    postValue = IO_RB0_ETH_CS_GetValue();
    </code>

*/
#define IO_RB0_ETH_CS_GetValue()         _RB0
/**
  @Summary
    Configures the GPIO pin, RB0, as an input.

  @Description
    Configures the GPIO pin, RB0, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RB0 as an input
    IO_RB0_ETH_CS_SetDigitalInput();
    </code>

*/
#define IO_RB0_ETH_CS_SetDigitalInput()  _TRISB0 = 1
/**
  @Summary
    Configures the GPIO pin, RB0, as an output.

  @Description
    Configures the GPIO pin, RB0, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RB0 as an output
    IO_RB0_ETH_CS_SetDigitalOutput();
    </code>

*/
#define IO_RB0_ETH_CS_SetDigitalOutput() _TRISB0 = 0
/**
  @Summary
    Sets the GPIO pin, RB1, high using LATB1.

  @Description
    Sets the GPIO pin, RB1, high using LATB1.

  @Preconditions
    The RB1 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RB1 high (1)
    channel_AN3_SetHigh();
    </code>

*/
#define channel_AN3_SetHigh()          _LATB1 = 1
/**
  @Summary
    Sets the GPIO pin, RB1, low using LATB1.

  @Description
    Sets the GPIO pin, RB1, low using LATB1.

  @Preconditions
    The RB1 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RB1 low (0)
    channel_AN3_SetLow();
    </code>

*/
#define channel_AN3_SetLow()           _LATB1 = 0
/**
  @Summary
    Toggles the GPIO pin, RB1, using LATB1.

  @Description
    Toggles the GPIO pin, RB1, using LATB1.

  @Preconditions
    The RB1 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RB1
    channel_AN3_Toggle();
    </code>

*/
#define channel_AN3_Toggle()           _LATB1 ^= 1
/**
  @Summary
    Reads the value of the GPIO pin, RB1.

  @Description
    Reads the value of the GPIO pin, RB1.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RB1
    postValue = channel_AN3_GetValue();
    </code>

*/
#define channel_AN3_GetValue()         _RB1
/**
  @Summary
    Configures the GPIO pin, RB1, as an input.

  @Description
    Configures the GPIO pin, RB1, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RB1 as an input
    channel_AN3_SetDigitalInput();
    </code>

*/
#define channel_AN3_SetDigitalInput()  _TRISB1 = 1
/**
  @Summary
    Configures the GPIO pin, RB1, as an output.

  @Description
    Configures the GPIO pin, RB1, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RB1 as an output
    channel_AN3_SetDigitalOutput();
    </code>

*/
#define channel_AN3_SetDigitalOutput() _TRISB1 = 0
/**
  @Summary
    Sets the GPIO pin, RB11, high using LATB11.

  @Description
    Sets the GPIO pin, RB11, high using LATB11.

  @Preconditions
    The RB11 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RB11 high (1)
    IO_RB11_DBU_PIN_SetHigh();
    </code>

*/
#define IO_RB11_DBU_PIN_SetHigh()          _LATB11 = 1
/**
  @Summary
    Sets the GPIO pin, RB11, low using LATB11.

  @Description
    Sets the GPIO pin, RB11, low using LATB11.

  @Preconditions
    The RB11 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RB11 low (0)
    IO_RB11_DBU_PIN_SetLow();
    </code>

*/
#define IO_RB11_DBU_PIN_SetLow()           _LATB11 = 0
/**
  @Summary
    Toggles the GPIO pin, RB11, using LATB11.

  @Description
    Toggles the GPIO pin, RB11, using LATB11.

  @Preconditions
    The RB11 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RB11
    IO_RB11_DBU_PIN_Toggle();
    </code>

*/
#define IO_RB11_DBU_PIN_Toggle()           _LATB11 ^= 1
/**
  @Summary
    Reads the value of the GPIO pin, RB11.

  @Description
    Reads the value of the GPIO pin, RB11.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RB11
    postValue = IO_RB11_DBU_PIN_GetValue();
    </code>

*/
#define IO_RB11_DBU_PIN_GetValue()         _RB11
/**
  @Summary
    Configures the GPIO pin, RB11, as an input.

  @Description
    Configures the GPIO pin, RB11, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RB11 as an input
    IO_RB11_DBU_PIN_SetDigitalInput();
    </code>

*/
#define IO_RB11_DBU_PIN_SetDigitalInput()  _TRISB11 = 1
/**
  @Summary
    Configures the GPIO pin, RB11, as an output.

  @Description
    Configures the GPIO pin, RB11, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RB11 as an output
    IO_RB11_DBU_PIN_SetDigitalOutput();
    </code>

*/
#define IO_RB11_DBU_PIN_SetDigitalOutput() _TRISB11 = 0
/**
  @Summary
    Sets the GPIO pin, RB3, high using LATB3.

  @Description
    Sets the GPIO pin, RB3, high using LATB3.

  @Preconditions
    The RB3 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RB3 high (1)
    channel_AN5_SetHigh();
    </code>

*/
#define channel_AN5_SetHigh()          _LATB3 = 1
/**
  @Summary
    Sets the GPIO pin, RB3, low using LATB3.

  @Description
    Sets the GPIO pin, RB3, low using LATB3.

  @Preconditions
    The RB3 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RB3 low (0)
    channel_AN5_SetLow();
    </code>

*/
#define channel_AN5_SetLow()           _LATB3 = 0
/**
  @Summary
    Toggles the GPIO pin, RB3, using LATB3.

  @Description
    Toggles the GPIO pin, RB3, using LATB3.

  @Preconditions
    The RB3 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RB3
    channel_AN5_Toggle();
    </code>

*/
#define channel_AN5_Toggle()           _LATB3 ^= 1
/**
  @Summary
    Reads the value of the GPIO pin, RB3.

  @Description
    Reads the value of the GPIO pin, RB3.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RB3
    postValue = channel_AN5_GetValue();
    </code>

*/
#define channel_AN5_GetValue()         _RB3
/**
  @Summary
    Configures the GPIO pin, RB3, as an input.

  @Description
    Configures the GPIO pin, RB3, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RB3 as an input
    channel_AN5_SetDigitalInput();
    </code>

*/
#define channel_AN5_SetDigitalInput()  _TRISB3 = 1
/**
  @Summary
    Configures the GPIO pin, RB3, as an output.

  @Description
    Configures the GPIO pin, RB3, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RB3 as an output
    channel_AN5_SetDigitalOutput();
    </code>

*/
#define channel_AN5_SetDigitalOutput() _TRISB3 = 0
/**
  @Summary
    Sets the GPIO pin, RC0, high using LATC0.

  @Description
    Sets the GPIO pin, RC0, high using LATC0.

  @Preconditions
    The RC0 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RC0 high (1)
    channel_AN6_SetHigh();
    </code>

*/
#define channel_AN6_SetHigh()          _LATC0 = 1
/**
  @Summary
    Sets the GPIO pin, RC0, low using LATC0.

  @Description
    Sets the GPIO pin, RC0, low using LATC0.

  @Preconditions
    The RC0 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RC0 low (0)
    channel_AN6_SetLow();
    </code>

*/
#define channel_AN6_SetLow()           _LATC0 = 0
/**
  @Summary
    Toggles the GPIO pin, RC0, using LATC0.

  @Description
    Toggles the GPIO pin, RC0, using LATC0.

  @Preconditions
    The RC0 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RC0
    channel_AN6_Toggle();
    </code>

*/
#define channel_AN6_Toggle()           _LATC0 ^= 1
/**
  @Summary
    Reads the value of the GPIO pin, RC0.

  @Description
    Reads the value of the GPIO pin, RC0.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RC0
    postValue = channel_AN6_GetValue();
    </code>

*/
#define channel_AN6_GetValue()         _RC0
/**
  @Summary
    Configures the GPIO pin, RC0, as an input.

  @Description
    Configures the GPIO pin, RC0, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RC0 as an input
    channel_AN6_SetDigitalInput();
    </code>

*/
#define channel_AN6_SetDigitalInput()  _TRISC0 = 1
/**
  @Summary
    Configures the GPIO pin, RC0, as an output.

  @Description
    Configures the GPIO pin, RC0, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RC0 as an output
    channel_AN6_SetDigitalOutput();
    </code>

*/
#define channel_AN6_SetDigitalOutput() _TRISC0 = 0
/**
  @Summary
    Sets the GPIO pin, RC1, high using LATC1.

  @Description
    Sets the GPIO pin, RC1, high using LATC1.

  @Preconditions
    The RC1 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RC1 high (1)
    channel_AN7_SetHigh();
    </code>

*/
#define channel_AN7_SetHigh()          _LATC1 = 1
/**
  @Summary
    Sets the GPIO pin, RC1, low using LATC1.

  @Description
    Sets the GPIO pin, RC1, low using LATC1.

  @Preconditions
    The RC1 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RC1 low (0)
    channel_AN7_SetLow();
    </code>

*/
#define channel_AN7_SetLow()           _LATC1 = 0
/**
  @Summary
    Toggles the GPIO pin, RC1, using LATC1.

  @Description
    Toggles the GPIO pin, RC1, using LATC1.

  @Preconditions
    The RC1 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RC1
    channel_AN7_Toggle();
    </code>

*/
#define channel_AN7_Toggle()           _LATC1 ^= 1
/**
  @Summary
    Reads the value of the GPIO pin, RC1.

  @Description
    Reads the value of the GPIO pin, RC1.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RC1
    postValue = channel_AN7_GetValue();
    </code>

*/
#define channel_AN7_GetValue()         _RC1
/**
  @Summary
    Configures the GPIO pin, RC1, as an input.

  @Description
    Configures the GPIO pin, RC1, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RC1 as an input
    channel_AN7_SetDigitalInput();
    </code>

*/
#define channel_AN7_SetDigitalInput()  _TRISC1 = 1
/**
  @Summary
    Configures the GPIO pin, RC1, as an output.

  @Description
    Configures the GPIO pin, RC1, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RC1 as an output
    channel_AN7_SetDigitalOutput();
    </code>

*/
#define channel_AN7_SetDigitalOutput() _TRISC1 = 0
/**
  @Summary
    Sets the GPIO pin, RC2, high using LATC2.

  @Description
    Sets the GPIO pin, RC2, high using LATC2.

  @Preconditions
    The RC2 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RC2 high (1)
    channel_AN8_SetHigh();
    </code>

*/
#define channel_AN8_SetHigh()          _LATC2 = 1
/**
  @Summary
    Sets the GPIO pin, RC2, low using LATC2.

  @Description
    Sets the GPIO pin, RC2, low using LATC2.

  @Preconditions
    The RC2 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RC2 low (0)
    channel_AN8_SetLow();
    </code>

*/
#define channel_AN8_SetLow()           _LATC2 = 0
/**
  @Summary
    Toggles the GPIO pin, RC2, using LATC2.

  @Description
    Toggles the GPIO pin, RC2, using LATC2.

  @Preconditions
    The RC2 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RC2
    channel_AN8_Toggle();
    </code>

*/
#define channel_AN8_Toggle()           _LATC2 ^= 1
/**
  @Summary
    Reads the value of the GPIO pin, RC2.

  @Description
    Reads the value of the GPIO pin, RC2.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RC2
    postValue = channel_AN8_GetValue();
    </code>

*/
#define channel_AN8_GetValue()         _RC2
/**
  @Summary
    Configures the GPIO pin, RC2, as an input.

  @Description
    Configures the GPIO pin, RC2, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RC2 as an input
    channel_AN8_SetDigitalInput();
    </code>

*/
#define channel_AN8_SetDigitalInput()  _TRISC2 = 1
/**
  @Summary
    Configures the GPIO pin, RC2, as an output.

  @Description
    Configures the GPIO pin, RC2, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RC2 as an output
    channel_AN8_SetDigitalOutput();
    </code>

*/
#define channel_AN8_SetDigitalOutput() _TRISC2 = 0
/**
  @Summary
    Sets the GPIO pin, RC9, high using LATC9.

  @Description
    Sets the GPIO pin, RC9, high using LATC9.

  @Preconditions
    The RC9 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RC9 high (1)
    IO_RC9_ETH_RST_SetHigh();
    </code>

*/
#define IO_RC9_ETH_RST_SetHigh()          _LATC9 = 1
/**
  @Summary
    Sets the GPIO pin, RC9, low using LATC9.

  @Description
    Sets the GPIO pin, RC9, low using LATC9.

  @Preconditions
    The RC9 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RC9 low (0)
    IO_RC9_ETH_RST_SetLow();
    </code>

*/
#define IO_RC9_ETH_RST_SetLow()           _LATC9 = 0
/**
  @Summary
    Toggles the GPIO pin, RC9, using LATC9.

  @Description
    Toggles the GPIO pin, RC9, using LATC9.

  @Preconditions
    The RC9 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RC9
    IO_RC9_ETH_RST_Toggle();
    </code>

*/
#define IO_RC9_ETH_RST_Toggle()           _LATC9 ^= 1
/**
  @Summary
    Reads the value of the GPIO pin, RC9.

  @Description
    Reads the value of the GPIO pin, RC9.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RC9
    postValue = IO_RC9_ETH_RST_GetValue();
    </code>

*/
#define IO_RC9_ETH_RST_GetValue()         _RC9
/**
  @Summary
    Configures the GPIO pin, RC9, as an input.

  @Description
    Configures the GPIO pin, RC9, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RC9 as an input
    IO_RC9_ETH_RST_SetDigitalInput();
    </code>

*/
#define IO_RC9_ETH_RST_SetDigitalInput()  _TRISC9 = 1
/**
  @Summary
    Configures the GPIO pin, RC9, as an output.

  @Description
    Configures the GPIO pin, RC9, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RC9 as an output
    IO_RC9_ETH_RST_SetDigitalOutput();
    </code>

*/
#define IO_RC9_ETH_RST_SetDigitalOutput() _TRISC9 = 0
/**
  @Summary
    Sets the GPIO pin, RD5, high using LATD5.

  @Description
    Sets the GPIO pin, RD5, high using LATD5.

  @Preconditions
    The RD5 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RD5 high (1)
    IO_RD5_HB_LED_SetHigh();
    </code>

*/
#define IO_RD5_HB_LED_SetHigh()          _LATD5 = 1
/**
  @Summary
    Sets the GPIO pin, RD5, low using LATD5.

  @Description
    Sets the GPIO pin, RD5, low using LATD5.

  @Preconditions
    The RD5 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RD5 low (0)
    IO_RD5_HB_LED_SetLow();
    </code>

*/
#define IO_RD5_HB_LED_SetLow()           _LATD5 = 0
/**
  @Summary
    Toggles the GPIO pin, RD5, using LATD5.

  @Description
    Toggles the GPIO pin, RD5, using LATD5.

  @Preconditions
    The RD5 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RD5
    IO_RD5_HB_LED_Toggle();
    </code>

*/
#define IO_RD5_HB_LED_Toggle()           _LATD5 ^= 1
/**
  @Summary
    Reads the value of the GPIO pin, RD5.

  @Description
    Reads the value of the GPIO pin, RD5.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RD5
    postValue = IO_RD5_HB_LED_GetValue();
    </code>

*/
#define IO_RD5_HB_LED_GetValue()         _RD5
/**
  @Summary
    Configures the GPIO pin, RD5, as an input.

  @Description
    Configures the GPIO pin, RD5, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RD5 as an input
    IO_RD5_HB_LED_SetDigitalInput();
    </code>

*/
#define IO_RD5_HB_LED_SetDigitalInput()  _TRISD5 = 1
/**
  @Summary
    Configures the GPIO pin, RD5, as an output.

  @Description
    Configures the GPIO pin, RD5, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RD5 as an output
    IO_RD5_HB_LED_SetDigitalOutput();
    </code>

*/
#define IO_RD5_HB_LED_SetDigitalOutput() _TRISD5 = 0
/**
  @Summary
    Sets the GPIO pin, RG6, high using LATG6.

  @Description
    Sets the GPIO pin, RG6, high using LATG6.

  @Preconditions
    The RG6 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RG6 high (1)
    IO_RG6_SW4_SetHigh();
    </code>

*/
#define IO_RG6_SW4_SetHigh()          _LATG6 = 1
/**
  @Summary
    Sets the GPIO pin, RG6, low using LATG6.

  @Description
    Sets the GPIO pin, RG6, low using LATG6.

  @Preconditions
    The RG6 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RG6 low (0)
    IO_RG6_SW4_SetLow();
    </code>

*/
#define IO_RG6_SW4_SetLow()           _LATG6 = 0
/**
  @Summary
    Toggles the GPIO pin, RG6, using LATG6.

  @Description
    Toggles the GPIO pin, RG6, using LATG6.

  @Preconditions
    The RG6 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RG6
    IO_RG6_SW4_Toggle();
    </code>

*/
#define IO_RG6_SW4_Toggle()           _LATG6 ^= 1
/**
  @Summary
    Reads the value of the GPIO pin, RG6.

  @Description
    Reads the value of the GPIO pin, RG6.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RG6
    postValue = IO_RG6_SW4_GetValue();
    </code>

*/
#define IO_RG6_SW4_GetValue()         _RG6
/**
  @Summary
    Configures the GPIO pin, RG6, as an input.

  @Description
    Configures the GPIO pin, RG6, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RG6 as an input
    IO_RG6_SW4_SetDigitalInput();
    </code>

*/
#define IO_RG6_SW4_SetDigitalInput()  _TRISG6 = 1
/**
  @Summary
    Configures the GPIO pin, RG6, as an output.

  @Description
    Configures the GPIO pin, RG6, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RG6 as an output
    IO_RG6_SW4_SetDigitalOutput();
    </code>

*/
#define IO_RG6_SW4_SetDigitalOutput() _TRISG6 = 0
/**
  @Summary
    Sets the GPIO pin, RG7, high using LATG7.

  @Description
    Sets the GPIO pin, RG7, high using LATG7.

  @Preconditions
    The RG7 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RG7 high (1)
    IO_RG7_SW5_SetHigh();
    </code>

*/
#define IO_RG7_SW5_SetHigh()          _LATG7 = 1
/**
  @Summary
    Sets the GPIO pin, RG7, low using LATG7.

  @Description
    Sets the GPIO pin, RG7, low using LATG7.

  @Preconditions
    The RG7 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RG7 low (0)
    IO_RG7_SW5_SetLow();
    </code>

*/
#define IO_RG7_SW5_SetLow()           _LATG7 = 0
/**
  @Summary
    Toggles the GPIO pin, RG7, using LATG7.

  @Description
    Toggles the GPIO pin, RG7, using LATG7.

  @Preconditions
    The RG7 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RG7
    IO_RG7_SW5_Toggle();
    </code>

*/
#define IO_RG7_SW5_Toggle()           _LATG7 ^= 1
/**
  @Summary
    Reads the value of the GPIO pin, RG7.

  @Description
    Reads the value of the GPIO pin, RG7.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RG7
    postValue = IO_RG7_SW5_GetValue();
    </code>

*/
#define IO_RG7_SW5_GetValue()         _RG7
/**
  @Summary
    Configures the GPIO pin, RG7, as an input.

  @Description
    Configures the GPIO pin, RG7, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RG7 as an input
    IO_RG7_SW5_SetDigitalInput();
    </code>

*/
#define IO_RG7_SW5_SetDigitalInput()  _TRISG7 = 1
/**
  @Summary
    Configures the GPIO pin, RG7, as an output.

  @Description
    Configures the GPIO pin, RG7, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RG7 as an output
    IO_RG7_SW5_SetDigitalOutput();
    </code>

*/
#define IO_RG7_SW5_SetDigitalOutput() _TRISG7 = 0
/**
  @Summary
    Sets the GPIO pin, RG8, high using LATG8.

  @Description
    Sets the GPIO pin, RG8, high using LATG8.

  @Preconditions
    The RG8 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RG8 high (1)
    IO_RG8_SW6_SetHigh();
    </code>

*/
#define IO_RG8_SW6_SetHigh()          _LATG8 = 1
/**
  @Summary
    Sets the GPIO pin, RG8, low using LATG8.

  @Description
    Sets the GPIO pin, RG8, low using LATG8.

  @Preconditions
    The RG8 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RG8 low (0)
    IO_RG8_SW6_SetLow();
    </code>

*/
#define IO_RG8_SW6_SetLow()           _LATG8 = 0
/**
  @Summary
    Toggles the GPIO pin, RG8, using LATG8.

  @Description
    Toggles the GPIO pin, RG8, using LATG8.

  @Preconditions
    The RG8 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RG8
    IO_RG8_SW6_Toggle();
    </code>

*/
#define IO_RG8_SW6_Toggle()           _LATG8 ^= 1
/**
  @Summary
    Reads the value of the GPIO pin, RG8.

  @Description
    Reads the value of the GPIO pin, RG8.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RG8
    postValue = IO_RG8_SW6_GetValue();
    </code>

*/
#define IO_RG8_SW6_GetValue()         _RG8
/**
  @Summary
    Configures the GPIO pin, RG8, as an input.

  @Description
    Configures the GPIO pin, RG8, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RG8 as an input
    IO_RG8_SW6_SetDigitalInput();
    </code>

*/
#define IO_RG8_SW6_SetDigitalInput()  _TRISG8 = 1
/**
  @Summary
    Configures the GPIO pin, RG8, as an output.

  @Description
    Configures the GPIO pin, RG8, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RG8 as an output
    IO_RG8_SW6_SetDigitalOutput();
    </code>

*/
#define IO_RG8_SW6_SetDigitalOutput() _TRISG8 = 0
/**
  @Summary
    Sets the GPIO pin, RG9, high using LATG9.

  @Description
    Sets the GPIO pin, RG9, high using LATG9.

  @Preconditions
    The RG9 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RG9 high (1)
    IO_RG9_SW1_SetHigh();
    </code>

*/
#define IO_RG9_SW1_SetHigh()          _LATG9 = 1
/**
  @Summary
    Sets the GPIO pin, RG9, low using LATG9.

  @Description
    Sets the GPIO pin, RG9, low using LATG9.

  @Preconditions
    The RG9 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RG9 low (0)
    IO_RG9_SW1_SetLow();
    </code>

*/
#define IO_RG9_SW1_SetLow()           _LATG9 = 0
/**
  @Summary
    Toggles the GPIO pin, RG9, using LATG9.

  @Description
    Toggles the GPIO pin, RG9, using LATG9.

  @Preconditions
    The RG9 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RG9
    IO_RG9_SW1_Toggle();
    </code>

*/
#define IO_RG9_SW1_Toggle()           _LATG9 ^= 1
/**
  @Summary
    Reads the value of the GPIO pin, RG9.

  @Description
    Reads the value of the GPIO pin, RG9.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RG9
    postValue = IO_RG9_SW1_GetValue();
    </code>

*/
#define IO_RG9_SW1_GetValue()         _RG9
/**
  @Summary
    Configures the GPIO pin, RG9, as an input.

  @Description
    Configures the GPIO pin, RG9, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RG9 as an input
    IO_RG9_SW1_SetDigitalInput();
    </code>

*/
#define IO_RG9_SW1_SetDigitalInput()  _TRISG9 = 1
/**
  @Summary
    Configures the GPIO pin, RG9, as an output.

  @Description
    Configures the GPIO pin, RG9, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RG9 as an output
    IO_RG9_SW1_SetDigitalOutput();
    </code>

*/
#define IO_RG9_SW1_SetDigitalOutput() _TRISG9 = 0

/**
    Section: Function Prototypes
*/
/**
  @Summary
    Configures the pin settings of the dsPIC33EP64MC506
    The peripheral pin select, PPS, configuration is also handled by this manager.

  @Description
    This is the generated manager file for the PIC24 / dsPIC33 / PIC32MM MCUs device.  This manager
    configures the pins direction, initial state, analog setting.
    The peripheral pin select, PPS, configuration is also handled by this manager.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    void SYSTEM_Initialize(void)
    {
        // Other initializers are called from this function
        PIN_MANAGER_Initialize();
    }
    </code>

*/
void PIN_MANAGER_Initialize (void);


#endif
