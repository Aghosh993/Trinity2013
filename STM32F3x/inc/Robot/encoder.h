/*
 * encoder.h
 *
 *  Created on: Feb 19, 2013
 *      Author: root
 */

#ifndef ENCODER_H_
#define ENCODER_H_

#include "common_headers.h"
#include "pwm.h"

#define DT_ENCODER 		25

#define k_p 2.0
#define k_i 0.15
#define k_d 0.71

#define k_p_s 2
#define k_d_s 0

enum motor_ctrl_mode{
	MODE_OPENLOOP,
	MODE_POSITION,
	MODE_SPEED
};

typedef struct{
	int position;
	float speed;
	float acceleration;

	int position_target;
	float speed_target;

	int integral;
	int last_error;
	float last_speed_error;
	float vel_cmd;

	enum motor_ctrl_mode m;
} encoderState;

void init_encoder_struct(encoderState* enc);
void TIM2_init_encoder(void);
void TIM8_init_encoder(void);
void encoder_update_ISR_init(void);

#endif /* ENCODER_H_ */
