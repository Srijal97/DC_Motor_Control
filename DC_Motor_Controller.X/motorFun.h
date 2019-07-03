/* 
 * File:   motorFun.h
 * Author: Eshu
 *
 * Created on May 12, 2019, 12:42 PM
 */

#ifndef MOTORFUN_H
#define	MOTORFUN_H

#ifdef	__cplusplus
extern "C" {
#endif

#include "Macros.h"
#include "projMacros.h"
#include "mcc_generated_files/pwm.h"
    
extern uINT SetSpeed;
extern uINT motorRPM;
extern uINT Speed_Kp;
extern uINT Speed_Ki;

extern uINT dcBusCurrent;
extern uINT Torque_Kp;
extern uINT Torque_Ki;

extern uINT MotorPWMDuty;
    
// Function Prototypes
void runMotor (uCHAR direction, uINT pwmDuty);
inline  void PWM_Override_Enable (PWM_GENERATOR genNum, uINT invLegState);
inline  void PWM_Override_Disable(PWM_GENERATOR genNum); 

// PI Controllers
uINT PIcontroller_Speed (uINT Setpoint, uINT PV, uINT Kpd, uINT Kid);
uINT PIcontroller_Torque(uINT Setpoint, uINT PV, uINT Kpd, uINT Kid);



#ifdef	__cplusplus
}
#endif

#endif	/* MOTORFUN_H */
