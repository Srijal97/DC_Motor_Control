//******************************************************************************
// Code developed for SPIT Mumbai
// This file includes all functions related to Control system logic
//******************************************************************************
#include "BizLogic.h"
#include "mcc_generated_files/pin_manager.h"
#include "mcc_generated_files/uart1.h"
#include "motorFun.h"
#include "encoder.h" 

uINT lpfGain        = 10000;   // 10000 = Approx. 0.16667 * 0xFFFF
uINT cnt20msSample  = 0;
uINT cnt50msSample  = 0;

uINT speedPIout = 0;
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
        
        uint16_t velocity = QEI_velocity_read() / 2;
        
        
        if ( motorDirection == MOTOR_DIR_FORWARD && velocity != 0) {
            velocity = (65535 - velocity) / 2;
        }
        else {
            velocity = velocity / 2;
        }
          
          
        SATURATE(velocity, 0, 4095);
        
        SATURATE(adcPotInput, 300, 3800);  // 3000 ---> Eb = 15V
        
        if(((float)dcBusVoltage - (float)dcBusCurrent*0.6) > 0.0f) {
            Eb = (uINT)((float)dcBusVoltage - (float)dcBusCurrent*0.6);
        }
        else {
            Eb = 0;
        }
        
        
        SATURATE(Eb, 0, 4095);
        
        speedPIout =  PIcontroller_Speed ((double)adcPotInput, (double)velocity, speed_Kp, speed_Ki, speed_Kd);
        // value from 0 to 4095
        
        MotorPWMDuty = (uINT) (((uLONG)speedPIout * MAX_PWM_COUNT)/4095); 
        
        SATURATE(MotorPWMDuty, MIN_PWM_COUNT, MAX_PWM_COUNT); 
        
        
        //----------------------------------------------------------//
        /*
        motorSetRPM = (uINT)(adcPotInput >> 2);  // range 0 to 4096/4
        
        SATURATE(motorSetRPM, 200, 1024);
        
        //motorActualRPM = (uINT)(adcMotorVoltage >> 2) + 55;  
        // Derived from equation relating motor voltage and actual RPM (using tachometer)
        
        motorActualRPM = (sINT)((uINT)(adcMotorVoltage >> 1) - (uINT32)(2*dcBusCurrent) + 130);
        
        SATURATE(motorActualRPM, 100, 1600);
        
        speedPIout = PIcontroller_Speed (motorSetRPM, (uINT)motorActualRPM, speed_Kp, speed_Ki);
        // value from 0 to 4095
        
        MotorPWMDuty = (uINT) (((uLONG)speedPIout * MAX_PWM_COUNT) >> 12); 
        
        SATURATE(MotorPWMDuty, MIN_PWM_COUNT, MAX_PWM_COUNT);   
*/
        //------------------------------------------------------------//
        // MotorPWMDuty = (uINT) (adcPotInput >> 1);
                
         //SATURATE(MotorPWMDuty, MIN_PWM_COUNT, MAX_PWM_COUNT);   
    }
    
    if(motorControlMode == CONTROL_SPEED_MODE)
    {
        // Current Setpoint will vary from 0 to 4096
        speedPIout   = PIcontroller_Speed (motorSetRPM, motorActualRPM, speed_Kp, speed_Ki, speed_Kd);
        
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
    
    static uINT sample_count = 0;
    static uLONG current_sample_sum = 0;
    static uLONG voltage_sample_sum = 0;

    // 100us Sampling
    adcBusCurrent_raw      = sampleReadADC(ADC_CHN0_BUS_CURRENT); 
    adcMotorVoltage_raw    = sampleReadADC(ADC_CHN5_BACK_EMF); 
    adcInpVoltage_raw      = sampleReadADC(ADC_CHN1_INPUT_VOLT);
    
    adcBusCurrent   = LPF(adcBusCurrent_raw, adcBusCurrent,   lpfGain);
    adcMotorVoltage = LPF(adcMotorVoltage_raw, adcMotorVoltage, lpfGain);
    adcInputVoltage = LPF(adcInpVoltage_raw, adcInputVoltage, lpfGain);
    
    // Convert ADC current count to real Value - Formula can be applied later
    if(sample_count < 1000) {
        current_sample_sum += (uLONG)adcBusCurrent;
        voltage_sample_sum += (uLONG)adcMotorVoltage;
        sample_count++;
    }
    else {
        dcBusCurrent = (uINT)(uLONG)(current_sample_sum / (uLONG)1000);
        dcBusVoltage = (uINT)(uLONG)(voltage_sample_sum / (uLONG)1000);
        sample_count = 0;
        current_sample_sum = 0;
        voltage_sample_sum = 0;
    }
    
    // 20ms Sampling
    if(++cnt20msSample > 199)
    {
        cnt20msSample = 0;

        
        adcPLCinputVoltage_raw = sampleReadADC(ADC_CHN3_PLC_INPUT); 
        adcTachoInput_raw      = sampleReadADC(ADC_CHN7_TACHO_INPUT); 
        adcPotInput_raw        = sampleReadADC(ADC_CHN8_POT_INPUT);

        
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