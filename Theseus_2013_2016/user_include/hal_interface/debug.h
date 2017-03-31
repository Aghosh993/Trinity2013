/*
 * debug.h
 *
 *  Created on: Feb 19, 2013
 *      Author: Abhimanyu Ghosh
 *
 *  Refactored on March 30, 2017
 */

#ifndef DEBUG_H_
#define DEBUG_H_

#include "common_headers.h"

#define DT_LED_MATRIX	90

void UART1_init(void);
void blink_leds(void);
void LED_MATRIX_ISR_init(void);

#endif /* DEBUG_H_ */
