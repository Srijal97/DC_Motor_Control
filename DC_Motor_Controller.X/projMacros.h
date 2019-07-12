/* 
 * File:   projMacros.h
 * Author: Eshu, Srijal
 *
 * Created on May 11, 2019, 10:32 AM
 */

#ifndef PROJMACROS_H
#define	PROJMACROS_H

#ifdef	__cplusplus
extern "C" {
#endif

// PWM Defines
#define MASK_OVERRIDE_HIGH_LOW_PWM      (uINT)0x0300
#define MASK_OVERRIDE_HIGH_SIDE_HIGH    (uINT)0x0080    // OVD Data 7th Bit
#define MASK_OVERRIDE_LOW_SIDE_HIGH     (uINT)0x0040    // OVD Data 6th Bit   
#define MASK_OVERRIDE_HIGH_SIDE_LOW     (uINT)0x0000
#define MASK_OVERRIDE_LOW_SIDE_LOW      (uINT)0x0000    
    
// Task Defines
#define NO_EVENT                        (uINT)0x0000
#define EVENT_TIMER_1ms_TASK            (uINT)0x0001
#define EVENT_TIMER_10ms_TASK           (uINT)0x0002
#define EVENT_TIMER_25ms_TASK           (uINT)0x0004
#define EVENT_MODBUS_TASK               (uINT)0x0008
#define EVENT_CAN_COM_TASK              (uINT)0x0010
#define EVENT_ETHERNATE_TASK            (uINT)0x0020          

            
// ADC Channel Macros
#define ADC_CHN0_BUS_CURRENT        (uCHAR)0 
#define ADC_CHN1_INPUT_VOLT         (uCHAR)1 
#define ADC_CHN3_PLC_INPUT          (uCHAR)3        
#define ADC_CHN5_BACK_EMF           (uCHAR)5 
#define ADC_CHN6_INTERANL_TEMP      (uCHAR)6     
#define ADC_CHN7_TACHO_INPUT        (uCHAR)7     
#define ADC_CHN8_POT_INPUT          (uCHAR)8 
    
// Motor Control Macros
#define MOTOR_DIR_REVERSE    (uCHAR)0
#define MOTOR_DIR_FORWARD    (uCHAR)1
    
    

// Motor Function Macros    
#define CONTROL_POT_MODE        (uCHAR)0    // Open loop POT control Mode
#define CONTROL_SPEED_MODE      (uCHAR)1    // Speed Control Mode
#define CONTROL_TORQUE_MODE     (uCHAR)2    // Torque Control Mode
#define CONTROL_MODE_SLIDING    (uCHAR)3    // Sliding Control Mode
#define CONTROL_POT_PID_MODE    (uCHAR)4    // Closed loop with POT to set RPM

#define MAX_PI_OUT              (sINT)2047
#define MIN_PI_OUT              (sINT)-2047
#define MAX_PWM_COUNT           (uINT)2450
#define MIN_PWM_COUNT           (uINT)50

// DC Bus Current Count    
#define MIN_CUR_COUNT           (uINT)100   // Could be offset voltage of Opamp
#define MAX_CUR_COUNT           (uINT)3350  // For DC Bus current = 15A    
    
#define MAX_TXRX_BUFF_LENGHT    (uCHAR)50
    
// MODBUS Commands
#define THIS_DEVICE_ADDRESS         (uCHAR)0x80     // Align to KWP2K    
#define READ_HOLDING_REGISTERS      (uCHAR)0x03  
#define PRESET_SINGLE_REGISTER      (uCHAR)0x06
#define PRESET_MULTIPLE_REGISTERS   (uCHAR)0x10   

#ifdef	__cplusplus
}
#endif

#endif	/* PROJMACROS_H */

