/*
 * encoder.h
 *
 *  Created on: Feb 19, 2013
 *      Author: root
 */

#ifndef ENCODER_H_
#define ENCODER_H_

#include "common_headers.h"

#define DT_ENCODER 		150

typedef struct{
	int position;
	float speed;
	float acceleration;
} encoderState;

void init_encoder_struct(encoderState* enc);
void TIM2_init_encoder(void);
void TIM8_init_encoder(void);
void encoder_update_ISR_init(void);

#endif /* ENCODER_H_ */
