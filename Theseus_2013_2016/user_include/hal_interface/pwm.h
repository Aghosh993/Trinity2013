/*
 * pwm.h
 *
 *  Created on: Feb 19, 2013
 *      Author: Abhimanyu Ghosh
 *
 *  Refactored on March 30, 2017
 */

#ifndef PWM_H_
#define PWM_H_

#include "common_headers.h"

#include <stdint.h>
#include <math.h>

void pwm_out1_init(uint16_t frequency);
void pwm1_output(float duty);

void pwm_out2_init(uint16_t frequency);
void pwm2_output(float duty);

void pwm_out3_init(uint16_t frequency);
void pwm3_output(float duty);

void config_pwm_freq(int frequency, int* period, int* prescaler);

#endif /* PWM_H_ */
