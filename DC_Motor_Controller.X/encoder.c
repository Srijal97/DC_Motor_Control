/* 
 * File:   encoder.h
 * Author: Srijal Poojari
 *
 * Created on July 24, 2019, 11:50 AM
 */

#include "encoder.h"

void QEI_initialize(void) {
    QEI1CON = 0x8000;
    
    QEI1IOC = 0x4000;

}

uint16_t QEI_velocity_read(void) {
    return VEL1CNT;
}
