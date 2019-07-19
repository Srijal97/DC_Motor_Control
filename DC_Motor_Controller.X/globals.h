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

uINT motorSetRPM = 0;
sINT motorActualRPM = 0;
uINT Eb = 0;
double speed_Kp = 1.3;//4000;
double speed_Ki = 1;//100;
double speed_Kd = 0.01;

uINT dcBusCurrent = 0; 
uINT dcBusVoltage = 0; 
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

