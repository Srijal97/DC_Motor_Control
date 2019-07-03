//******************************************************************************
// Code developed for SPIT Mumbai
// This file includes all functions related to Control system logic
//******************************************************************************
#include "BizLogic.h"
#include "mcc_generated_files/pin_manager.h"
#include "mcc_generated_files/uart1.h"
#include "motorFun.h" 

uINT lpfGain        = 10000;   // Approx. 0.16667 * 0xFFFF
uINT cnt20msSample  = 0;
uINT cnt50msSample  = 0;

sINT speedPIout = 0;
uINT currSetpoint = 0;
sINT torquePIout = 0; 

uCHAR messageTx[50] = {0x00, 0xAA, 0x55};



//******************************************************************************
// runMotorBasedOnPOT 
// Input - None
// This function used to run the motor from POT - Min - 52 and Max - 4083 
//******************************************************************************
void runMotorWithControl (void)
{
    if(motorControlMode == CONTROL_POT_MODE)
    {
        MotorPWMDuty = (uINT) (adcPotInput>>1);
                
        SATURATE(MotorPWMDuty, MIN_PWM_COUNT, MAX_PWM_COUNT);   
    }
    
    if(motorControlMode == CONTROL_SPEED_MODE)
    {
        // Current Setpoint will vary from 0 to 4096
        speedPIout   = PIcontroller_Speed (SetSpeed, motorRPM, Speed_Kp, Speed_Ki);
        
        currSetpoint = (uINT) (((uLONG)speedPIout * MAX_CUR_COUNT) >> 12); // range 0 to 4096 only        
        
        torquePIout  = PIcontroller_Torque ( currSetpoint, 
                                             dcBusCurrent, 
                                             Torque_Kp, 
                                             Torque_Ki );  
          
        MotorPWMDuty = (uINT) (((uLONG)torquePIout * MAX_PWM_COUNT) >> 12); // range 0 to 2048 only 
        
        SATURATE(MotorPWMDuty, MIN_PWM_COUNT, MAX_PWM_COUNT); 
    }
    
    // Send PWM Duty 
    runMotor(motorDirection, MotorPWMDuty);   //  
}
//******************************************************************************



//******************************************************************************
// readAllAnalogVariables 
// Input - None
// This function filters ADC data and make ADC data Ready for further processing
//******************************************************************************
void readAllAnalogVariables (void)
{
    volatile uINT adcBusCurrent_raw      = 0;
    volatile uINT adcInpVoltage_raw      = 0; 
    volatile uINT adcPLCinputVoltage_raw = 0;
    volatile uINT adcMotorVoltage_raw    = 0;
    volatile uINT adcInternalTemp_raw    = 0;
    volatile uINT adcTachoInput_raw      = 0;    
    volatile uINT adcPotInput_raw        = 0;

    // 100us Sampling
    adcBusCurrent_raw      = sampleReadADC(ADC_CHN0_BUS_CURRENT); 
    adcInpVoltage_raw      = sampleReadADC(ADC_CHN1_INPUT_VOLT);
    adcBusCurrent   = LPF(adcBusCurrent_raw, adcBusCurrent,   lpfGain);
    adcInputVoltage = LPF(adcInpVoltage_raw, adcInputVoltage, lpfGain);
    
    // Convert ADC current count to real Value - Formula can be applied later
    dcBusCurrent   = adcBusCurrent; 
    
    // 20ms Sampling
    if(++cnt20msSample > 199)
    {
        cnt20msSample = 0;

        adcMotorVoltage_raw    = sampleReadADC(ADC_CHN5_BACK_EMF); 
        adcPLCinputVoltage_raw = sampleReadADC(ADC_CHN3_PLC_INPUT); 
        adcTachoInput_raw      = sampleReadADC(ADC_CHN7_TACHO_INPUT); 
        adcPotInput_raw        = sampleReadADC(ADC_CHN8_POT_INPUT);

        adcMotorVoltage     = LPF(adcMotorVoltage_raw, adcMotorVoltage, lpfGain);
        adcPLCinputVoltage  = LPF(adcPLCinputVoltage_raw, adcPLCinputVoltage, lpfGain);    
        adcTachoInput       = LPF(adcTachoInput_raw, adcTachoInput, lpfGain);
        adcPotInput         = LPF(adcPotInput_raw, adcPotInput, lpfGain);
    }
    
    // 50ms Sampling
    if(++cnt50msSample > 499)
    {
        cnt50msSample = 0;
        
        adcInternalTemp_raw = sampleReadADC(ADC_CHN6_INTERANL_TEMP); 
        adcInternalTemp = LPF(adcInternalTemp_raw, adcInternalTemp, lpfGain);
    }
}
//******************************************************************************

    
//******************************************************************************
// sampleReadADC 
// Input - ADC channel No.
// This function samples ADC and convert ADC. returns ADC data
//******************************************************************************
uINT sampleReadADC (uCHAR channelNo)
{
    AD1CHS0 = channelNo;
    
    AD1CON1bits.SAMP = 0;       // Start the conversions

    while (!AD1CON1bits.DONE);  // Wait for conversion to complete
    
    return (ADC1BUF0);
}
//******************************************************************************