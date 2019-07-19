//******************************************************************************
// Code developed for SVIT Mumbai
// This file includes all functions related to Motor control
//******************************************************************************
#include "motorFun.h"


  double input;
  double Setpoint;
//******************************************************************************
// runMotor 
// Input - Direction and PWM Duty Count
// This function update PWM duty cycle and Direction of the motor
//******************************************************************************
void runMotor (uCHAR direction, uINT pwmDuty)
{
    volatile uINT  invLegStatus  = 0x0000;    
    static   uCHAR prevDirection = 1;
    
    // if direction = FORWARD then Bridge A will be in complementary PWM while Bridge B
    // in Override Mode.  
    if(direction == MOTOR_DIR_FORWARD)
    {
        if(prevDirection != direction)
        {
            PWM_ModuleDisable();
            
            PWM_Override_Disable(PWM_GENERATOR_1);
                                                          
            invLegStatus = (MASK_OVERRIDE_HIGH_SIDE_LOW) | (MASK_OVERRIDE_LOW_SIDE_HIGH);
            
            // PWM Gen2 - Half Bridge 2 in Override Mode
            PWM_Override_Enable(PWM_GENERATOR_2, invLegStatus);
            
            PWM_ModuleEnable();
        }
        PWM_DutyCycleSet(PWM_GENERATOR_1, pwmDuty);        
    }
    else
    {
         if(prevDirection != direction)
        {
            PWM_ModuleDisable();
            
            PWM_Override_Disable(PWM_GENERATOR_2);            
            
            invLegStatus = (MASK_OVERRIDE_HIGH_SIDE_LOW) | (MASK_OVERRIDE_LOW_SIDE_HIGH);
            
            // PWM Gen2 - Half Bridge 2 in Override Mode
            PWM_Override_Enable(PWM_GENERATOR_1, invLegStatus);
            
            PWM_ModuleEnable();
        }
        PWM_DutyCycleSet(PWM_GENERATOR_2, pwmDuty);       
    }
    
    prevDirection = direction;
}
//******************************************************************************


//******************************************************************************
// PWM_Override_Enable 
// Input - PWM Half Bridge and Inverter Leg Status
// This function used to override the PWM with OVD Status
//******************************************************************************
inline void PWM_Override_Enable(PWM_GENERATOR genNum, uINT invLegState)
{
    volatile uINT IOCON_Data = 0;
    
    IOCON_Data = (MASK_OVERRIDE_HIGH_LOW_PWM | invLegState);
    
    switch(genNum) { 
        case PWM_GENERATOR_1:
                __builtin_write_PWMSFR(&IOCON1, (IOCON1 | IOCON_Data), &PWMKEY);                
                break;       
        case PWM_GENERATOR_2:
                __builtin_write_PWMSFR(&IOCON2, (IOCON2 | IOCON_Data), &PWMKEY);                
                break;       
        case PWM_GENERATOR_3:
                __builtin_write_PWMSFR(&IOCON3, (IOCON3 | IOCON_Data), &PWMKEY);                
                break;       
        default:break;  
    }
}
//******************************************************************************


//******************************************************************************
// PWM_Override_Disable
// Input - PWM Half Bridge 
// This function used to disable override function
//******************************************************************************
inline void PWM_Override_Disable(PWM_GENERATOR genNum)
{
    switch(genNum) { 
        case PWM_GENERATOR_1:   
                __builtin_write_PWMSFR(&IOCON1, (IOCON1 & 0xFCFF), &PWMKEY);
                break;       
        case PWM_GENERATOR_2:   
                __builtin_write_PWMSFR(&IOCON2, (IOCON2 & 0xFCFF), &PWMKEY);
                break;       
        case PWM_GENERATOR_3:   
                __builtin_write_PWMSFR(&IOCON3, (IOCON3 & 0xFCFF), &PWMKEY);
                break;       
        default:break;  
    }
}
//******************************************************************************


//******************************************************************************
// Speed PI Controller
//******************************************************************************
uINT PIcontroller_Speed (double setpoint, double processVariable, double Kp, double Ki, double Kd)
{
    /*
    volatile float error  = 0;
    
    volatile float P_Term  = 0;
    volatile float I_Term  = 0;
    
    volatile uINT PID_out = 0;
    volatile float elapsedTime = 0.001;
        
    static float cumulativeError  = 0;
    
    error = (float)(sINT)(setpoint - processVariable); 
    SATURATE(error, MIN_PI_OUT, MAX_PI_OUT);  // between -2047 to 2048
  
    cumulativeError += error * elapsedTime; // Apply the History  
    
    P_Term = Kp * error;   
    I_Term = Ki * cumulativeError;   

    PID_out = (uINT)(float)(2047.0 + P_Term + I_Term);
    
    SATURATE(PID_out, 0, 4095); 
    
    return PID_out;  // The return value will be 0 to 4095
    */
    
    
    static double lastInput = 0;
    static double cumulative_error = 0;
 
    double SampleTimeInSec = ((double)100)/1000;

    Ki = Ki * SampleTimeInSec;
    Kd = Kd / SampleTimeInSec;
    input = processVariable;
    Setpoint=setpoint;
    
    double error = setpoint - input;
    double dInput = (input - lastInput);
    double output = 0;
    
    cumulative_error += error;
  

      /*Add Proportional on Measurement, if P_ON_M is specified*/
     // if(!pOnE) outputSum -= Kp * dInput;

    //if(cumulative_error > 4095) cumulative_error = 4095;
    //else if(cumulative_error < 0) cumulative_error = 0;

      /*Add Proportional on Error, if P_ON_E is specified*/
	

      /*Compute Rest of PID Output*/
    output = (Kp * error) + (Ki * cumulative_error) - (Kd * dInput);
      
       /*Remember some variables for next time*/
    lastInput = input;

	if(output > 4095) {
        output = 4095;
    }
    else if(output < 0) { 
        output = 0;
    }
    
	return (uINT)output;
}
//******************************************************************************


//******************************************************************************
// Torque PI Controller
//******************************************************************************
uINT PIcontroller_Torque(uINT Setpoint, uINT PV, uINT Kpd, uINT Kid)
{
    volatile sINT Ek  = 0;
    
    volatile sINT P_Term  = 0;
    volatile sINT I_Term  = 0;
    volatile sINT delPV   = 0;
    volatile uINT PID_out = 0;
        
    static sINT Ck     = 0;
    static uINT PVk_1  = 0;
    
    Ek     = (sINT) (Setpoint - PV); 
    SATURATE(Ek, MIN_PI_OUT, MAX_PI_OUT); 
    
    delPV  = (sINT) (PV - PVk_1);    
    P_Term = (sINT) ((Kpd * (sLONG)delPV) >> 12);   // Kp = 4096  then Kpd = Kp/4096 
    I_Term = (sINT) ((Kid * (sLONG)Ek   ) >> 16);   // Ki = 65535 then Kid = Ki/65535 
    
    Ck = (sINT) (Ck - P_Term + I_Term);
    SATURATE(Ck, MIN_PI_OUT, MAX_PI_OUT);    
    
    PVk_1   = PV; // Apply the History   
    
    PID_out = (uINT) (MAX_PI_OUT + Ck);
    
    return PID_out;  // The return value will be -2048 to 2048  
}
//******************************************************************************
