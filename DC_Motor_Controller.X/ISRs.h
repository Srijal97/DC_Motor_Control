#ifndef ISRS_H
#define	ISRS_H

#ifdef	__cplusplus
extern "C" {
#endif

#include "Macros.h"
#include "projMacros.h"
#include "stdbool.h"
#include "motorFun.h"

    
void enableInterrupts(void);

void disableInterrupts(void);

void initInterrupts(void);

void __attribute__((__interrupt__,no_auto_psv)) _CNInterrupt(void);

extern uCHAR motorControlMode;


#ifdef	__cplusplus
}
#endif

#endif	/* ISRS_H */
