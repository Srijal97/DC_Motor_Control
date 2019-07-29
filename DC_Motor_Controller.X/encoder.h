/* 
 * File:   encoder.h
 * Author: Srijal Poojari
 *
 * Created on July 24, 2019, 11:50 AM
 */

#ifndef ENCODER_H
#define	ENCODER_H

#include <xc.h>


#ifdef	__cplusplus
extern "C" {
#endif

    //configures the QEI module
    void QEI_initialize(void);
    
    //reads the VELCNT register
    uint16_t QEI_velocity_read(void);


#ifdef	__cplusplus
}
#endif

#endif	/* ENCODER_H */

