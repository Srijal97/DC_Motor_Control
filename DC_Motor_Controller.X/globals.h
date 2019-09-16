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

uCHAR motorControlMode = CONTROL_ENCODER_MODE; 
uCHAR motorDirection = MOTOR_DIR_FORWARD;

uINT motorSetRPM = 0;
sINT motorActualRPM = 0;
uINT Eb = 0;

//double speed_Kp = 1.3;//4000;
//double speed_Ki = 1;//100;
//double speed_Kd = 0.01;



uINT dcBusCurrent = 0; 
uINT dcBusVoltage = 0; 

uINT enc_speed_Kp = 4000;
uINT enc_speed_Ki = 600;

uINT bemf_speed_Kp = 1500;
uINT bemf_speed_Ki = 900;

uINT speed_Kp = 1100;
uINT speed_Ki = 500;

uINT torque_Kp = 2000;
uINT torque_Ki = 500;

uINT MotorPWMDuty = 0; 

uCHAR messageBuffer[MAX_TXRX_BUFF_LENGHT]; // 50 bytes 
uCHAR receiveByteLen = 50;
uINT  modbusTimeoutCounter;
uCHAR flagSlaveAdd = NO;

uint16_t encoder_vel = 0;

uINT ss_duty_count = 0;


#ifdef	__cplusplus
}
#endif

#endif	/* GLOBALS_H */

