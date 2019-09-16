/* 
 * File:   BizLogic.h
 * Author: Eshu
 *
 * Created on May 12, 2019, 1:05 PM
 */

#ifndef BIZLOGIC_H
#define	BIZLOGIC_H

#ifdef	__cplusplus
extern "C" {
#endif

#include <xc.h>
#include "Macros.h"
#include "projMacros.h"

// Variables
extern uINT adcBusCurrent;
extern uINT adcInputVoltage;
extern uINT adcPLCinputVoltage;
extern uINT adcMotorVoltage;
extern uINT adcInternalTemp;
extern uINT adcTachoInput;
extern uINT adcPotInput;
extern uINT dcBusCurrent;
extern uINT dcBusVoltage;

extern uCHAR motorControlMode;
extern uCHAR motorDirection;

extern uINT motorSetRPM;
extern sINT motorActualRPM;
extern uINT Eb;

extern double enc_speed_Kp;
extern double enc_speed_Ki;
//extern double enc_speed_Kd;

extern double bemf_speed_Kp;
extern double bemf_speed_Ki;

//extern uINT enc_speed_Kp;
//extern uINT enc_speed_Ki;

//extern uINT bemf_speed_Kp;
//extern uINT bemf_speed_Ki;

extern uINT speed_Kp;
extern uINT speed_Ki;

extern uINT torque_Kp;
extern uINT torque_Ki;

extern uINT MotorPWMDuty;

extern uint16_t encoder_vel;
    
    
// Functions
void readAllAnalogVariables (void);
uINT sampleReadADC          (uCHAR channelNo);

void runMotorWithControl (void);

    
#ifdef	__cplusplus
}
#endif

#endif	/* BIZLOGIC_H */

