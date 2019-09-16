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
    
extern uINT motorSetRPM;
extern sINT motorActualRPM;
extern uINT Eb;

extern uCHAR motorDirection;

extern double enc_speed_Kp;
extern double enc_speed_Ki;

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

extern uINT dcBusCurrent;
extern uINT dcBusVoltage;


extern uINT MotorPWMDuty;

extern uint16_t encoder_vel;
    
// Function Prototypes
void runMotor (uCHAR direction, uINT pwmDuty);
inline  void PWM_Override_Enable (PWM_GENERATOR genNum, uINT invLegState);
inline  void PWM_Override_Disable(PWM_GENERATOR genNum); 

// PI Controllers
uINT PI_torque_discrete(uINT Setpoint, uINT PV, uINT Kpd, uINT Kid);
uINT PI_speed_discrete(uINT Setpoint, uINT PV, uINT Kpd, uINT Kid);

uINT PI_speed_cont(double setpoint, double processVariable, double Kp, double Ki);

void read_encoder_velocity();

#ifdef	__cplusplus
}
#endif

#endif	/* MOTORFUN_H */

