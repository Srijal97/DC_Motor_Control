//***************************************************************************************
// File Name    : Macros.h
// Rev          : None
// Description  : All Global Macros
// Created By   : NKG
//***************************************************************************************

#ifndef _MACROS_H_
#define _MACROS_H_

//***************************************************************************************
typedef unsigned char		    uCHAR;
typedef signed char			    sCHAR;
typedef unsigned int 		    uINT;   // 0 to 4095
typedef signed int			    sINT;   // -2048 to 2047
typedef unsigned short int      uINT16;
typedef unsigned long int       uINT32; 
typedef unsigned long	    	uLONG;
typedef signed long             sLONG;
typedef float                   FLOAT;

//***************************************************************************************  


// Global Errors
#define ERR_OK           (uCHAR)0             // OK 
#define ERR_SPEED        (uCHAR)1             // This device does not work in the 
                                              // active speed mode. 
#define ERR_RANGE        (uCHAR)2             // Parameter out of range. 
#define ERR_VALUE        (uCHAR)3             // Parameter of incorrect value. 
#define ERR_OVERFLOW     (uCHAR)4             // Timer overflow. 
#define ERR_MATH         (uCHAR)5             // Overflow during evaluation. 
#define ERR_ENABLED      (uCHAR)6             // Device is enabled. 
#define ERR_DISABLED     (uCHAR)7             // Device is disabled. 
#define ERR_BUSY         (uCHAR)8             // Device is busy. 
#define ERR_NOTAVAIL     (uCHAR)9             // Requested value or method not 
                                              // available.
#define ERR_RXEMPTY      (uCHAR)10            // No data in receiver. 
#define ERR_TXFULL       (uCHAR)11            // Transmitter is full. 
#define ERR_BUSOFF       (uCHAR)12            // Bus not available. 
#define ERR_OVERRUN      (uCHAR)13            // Overrun error is detected. 
#define ERR_FRAMING      (uCHAR)14            // Framing error is detected. 
#define ERR_PARITY       (uCHAR)15            // Parity error is detected. 
#define ERR_NOISE        (uCHAR)16            // Noise error is detected. 
#define ERR_IDLE         (uCHAR)17            // Idle error is detected. 
#define ERR_FAULT        (uCHAR)18            // Fault error is detected. 
#define ERR_BREAK        (uCHAR)19            // Break char is received during 
                                              // communication. 
#define ERR_CRC          (uCHAR)20            // CRC error is detected. 
#define ERR_ARBITR       (uCHAR)21            // A node losts arbitration. This error 
                                              // occurs if two nodes start transmission 
                                              // at the same time. 
#define ERR_PROTECT      (uCHAR)22            // Protection error is detected. 
#define ERR_UNDERFLOW    (uCHAR)23            // Underflow error is detected. 
#define ERR_UNDERRUN     (uCHAR)24            // Underrun error is detected. 
#define ERR_COMMON       (uCHAR)25            // Common error of a device.
#define ERR_LINSYNC      (uCHAR)26            // LIN synchronization error is 
                                              //  detected. 
#define ERR_FAILED       (uCHAR)27            // Requested functionality or process 
                                              // failed. 
                                          
//***************************************************************************************

//***************************************************************************************


//***************************************************************************************
// Communication Variables
#define PHYSICAL_ADDR			(uCHAR)0x80
#define TARGET_ECU_ADDRESS		(uCHAR)0x00    // 0x20               
#define SOURCE_TESTER_ADDRESS	(uCHAR)0x00    // 0xF3               



#define ABSOLUTE(x)                 ((x > 0) ? (x) : -(x))  

#define SATURATE(x, min, max)       if(x < min) (x = min); \
                                    if(x > max) (x = max)                                                    


// Low Pass Filter (Fixed Point)
#define LPF(Xn, Yn_1, gain) 	  ( (sINT) (((0xFFFF - gain) *  \
                              	  (sLONG)Yn_1)  >> 16)   +    \
                              	  (sINT) (((gain) *           \
                                  (sLONG)Xn)    >> 16) )   
                              

//***************************************************************************************
enum {OFF, ON, TRISTATE};
enum {STOP, START};
enum {NO, YES};
enum {RESET, SET};
enum {INPUT, OUTPUT};
enum {LEAN, RICH};
enum {STEP_REVERSE_DIR, STEP_FORWARD_DIR};
enum {POSITIVE, NEGATIVE} ;

//***************************************************************************************


//***************************************************************************************
// Bits manipulations MACROs
//***************************************************************************************
#define BIT(x)               (0x01 << (x))               	// Set xth bit      
#define SETBITS(x, y)        ((x) |= (y))                	// Set y bits in x
#define CLEARBITS(x, y)      ((x) &= (~(y)))             	// Clear y bits in x
#define SETBIT(x, y)         SETBITS((x), (BIT((y))))    	// Set yth bit in x
#define CLEARBIT(x, y)       CLEARBITS((x), (BIT((y))))  	// Clear yth bit in x
#define ROLBITS(x, y)        (x << y)       							// Rotate left x by y place
#define RORBITS(x, y)        (x >> y)       							// Rotate right x by y place


// Test the yth bit in x 
// returns TRUE when set 
// returns FALSE when reset
#define TESTBIT(x, y)        ((x & BIT(y)) ? TRUE : FALSE)   
#define	TOGGLE_VAR(x)					(x = ~x)                                      
//***************************************************************************************


//***************************************************************************************
// Bytes manipulations MACROs
//***************************************************************************************
#define TESTVAR_EQ(x, y)    ((x == y) ? TRUE : FALSE)
#define TESTVAR_GEQ(x, y)   ((x >= y) ? TRUE : FALSE) 
#define TESTVAR_LEQ(x, y)   ((x <= y) ? TRUE : FALSE)        
//***************************************************************************************


//***************************************************************************************
#define   ARRAY_ASCENDING       (uCHAR)0
#define   ARRAY_DECENDING       (uCHAR)1
#define   MAX_INDEX_TEMP_TABLE  (uCHAR)20

// Saturated Limits of PID output
#define   MAX_PID_OUTPUT    (sINT)4096
#define   MIN_PID_OUTPUT    (sINT)-4096

//***************************************************************************************


// Relay Pin


//***************************************************************************************
// Digital Outputs

// Analog Input channels
#define   ADC_CH0_BAT_TEMP        (uCHAR)0x00
//#define   ADC_CH1_CURRENT         (uCHAR)0x01
//#define   ADC_CH2_BAT_TEMP        (uCHAR)0x02
//#define   ADC_CH3_HUMIDITY        (uCHAR)0x03
//#define   ADC_CH4_AMB_TEMP        (uCHAR)0x04
//#define   ADC_CH5_VCC             (uCHAR)0x05  

//***************************************************************************************
// MODBUS Global Defination
#define   MODBUS_MESSAGE_SIZE_MAX (uCHAR)120
#define   TOTAL_MODBUS_VARIABLE   (uCHAR)166  // 0th Data is not used
#define   SIZE_OF_MODBUS_VARIABLE (uCHAR)1
#define   MAX_MODBUS_VAR_SUPPORTED (uCHAR)28

// Coil Mask Defination
#define   MASK_COIL_NONE            (uINT)0x0000
#define   MASK_COIL_WRITE_MEMORY    (uINT)0x0001  // internal EEPROM writing
#define   MASK_COIL_READ_MEMORY     (uINT)0x0002  // internal EEPROM reading
#define   MASK_COIL_WRITE_RTC       (uINT)0x0004  // RTC will write during EOL testing  
#define   MASK_COIL_SET_VALUES      (uINT)0x0008  // When values need to send to Slaves
#define   MASK_COIL_READ_VALUES     (uINT)0x0010   
#define   MASK_COIL_RELAY1          (uINT)0x0020     
#define   MASK_COIL_RELAY2          (uINT)0x0040
#define   MASK_COIL_RELAY3          (uINT)0x0080
#define   MASK_COIL_SEND_VOLT_DATA  (uINT)0x0100
#define   MASK_COIL_SEND_CURR_DATA  (uINT)0x0200

// Error Mask Defination
#define   MASK_ERROR_NONE         (uINT)0x0000
#define   MASK_ERROR_FLAG1        (uINT)0x0001
#define   MASK_ERROR_FLAG2        (uINT)0x0004
#define   MASK_ERROR_FLAG3        (uINT)0x0008
#define   MASK_ERROR_FLAG4        (uINT)0x0010
#define   MASK_ERROR_FLAG5        (uINT)0x0020
#define   MASK_ERROR_FLAG6        (uINT)0x0040
#define   MASK_ERROR_FLAG8        (uINT)0x0080

#define MODBUS_SLAVE_ADDRESS      (uCHAR)0x01		

// MODBUS Function Code
#define CODE_READ_COIL_STATUS           (uCHAR)1    
#define CODE_READ_INPUT_STATUS          (uCHAR)2
#define CODE_READ_HOLDING_REGISTERS     (uCHAR)3    // read from 40001    
#define CODE_READ_INPUT_REGISTERS       (uCHAR)4    
#define CODE_FORCE_SINGLE_COIL          (uCHAR)5    // Force single coil i.e. bit ON/OFF
#define CODE_PRESET_SINGLE_REG          (uCHAR)6    // Write single register from 40001    
#define CODE_READ_EXCEPTION_STATUS      (uCHAR)7
#define CODE_DIAGNOSTICS                (uCHAR)8
#define CODE_PROGRAM_484                (uCHAR)9
#define CODE_POLL_484                   (uCHAR)10
#define CODE_FETCH_COMM_EVENT_CNT       (uCHAR)11
#define CODE_FETCH_COMM_EVENT_LOG       (uCHAR)12
#define CODE_PROGRAM_CONTROLLER         (uCHAR)13
#define CODE_POLL_CONTROLLER            (uCHAR)14
#define CODE_FORCE_MULTIPLE_COILS       (uCHAR)15
#define CODE_PRESET_MULTIPLE_REGISTERS  (uCHAR)16
#define CODE_REPORT_SLAVE_ID            (uCHAR)17
#define CODE_PROGRAM_884_M84            (uCHAR)18
#define CODE_RESET_COM_LINK             (uCHAR)19
#define CODE_READ_GENERAL_REFERENCE     (uCHAR)20
#define CODE_WRITE_GENERAL_REFERENCE    (uCHAR)21
#define CODE_MASK_WRITE_4X_REGISTER     (uCHAR)22
#define CODE_READ_WRITE_4X_REGISTER     (uCHAR)23
#define CODE_READ_FIFO_QUEUE            (uCHAR)24


//***************************************************************************************
// CAN.h Macros

//***************************************************************************************

 
//***************************************************************************************
// BizLogic.h Macros
//***************************************************************************************


//***************************************************************************************
// Flash.h Macros
//
// Flash Commands

//***************************************************************************************

																					 			
#endif 
























