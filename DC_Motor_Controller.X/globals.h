/* 
 * File:   globals.h
 * Author: Eshu
 *
 * Created on May 12, 2019, 12:52 PM
 */

#ifndef GLOBALS_H
#define	GLOBALS_H

#include "Macros.h"
#include "projMacros.h"

#ifdef	__cplusplus
extern "C" {
#endif

// Task Vars
uINT eventRegister = 0x0000;

// BizLogic Vars
uINT adcBusCurrent      = 0;
uINT adcInputVoltage    = 0;
uINT adcPLCinputVoltage = 0;
uINT adcMotorVoltage    = 0;
uINT adcInternalTemp    = 0;
uINT adcTachoInput      = 0;
uINT adcPotInput        = 0;

uCHAR motorControlMode = CONTROL_POT_MODE; 
uCHAR motorDirection = MOTOR_DIR_FORWARD;

uINT SetSpeed = 2000;
uINT motorRPM = 0;
uINT Speed_Kp = 1500;
uINT Speed_Ki = 10;

uINT dcBusCurrent = 0; 
uINT Torque_Kp = 1200;
uINT Torque_Ki = 5;

uINT MotorPWMDuty = 0; 

uCHAR messageBuffer[MAX_TXRX_BUFF_LENGHT]; // 50 bytes 
uCHAR receiveByteLen = 50;
uINT  modbusTimeoutCounter;
uCHAR flagSlaveAdd = NO;


#ifdef	__cplusplus
}
#endif

#endif	/* GLOBALS_H */
