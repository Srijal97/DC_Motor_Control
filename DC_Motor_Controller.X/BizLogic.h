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

extern uCHAR motorControlMode;
extern uCHAR motorDirection;

extern uINT SetSpeed;
extern uINT motorRPM;
extern uINT Speed_Kp;
extern uINT Speed_Ki;


extern uINT Torque_Kp;
extern uINT Torque_Ki;

extern uINT MotorPWMDuty;
    
    
// Functions
void readAllAnalogVariables (void);
uINT sampleReadADC          (uCHAR channelNo);

void runMotorWithControl (void);

    
#ifdef	__cplusplus
}
#endif

#endif	/* BIZLOGIC_H */

