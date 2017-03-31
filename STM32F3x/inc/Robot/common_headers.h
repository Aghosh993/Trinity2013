/*
 * common_headers.h
 *
 *  Created on: Feb 19, 2013
 *      Author: root
 */

#ifndef COMMON_HEADERS_H_
#define COMMON_HEADERS_H_

#define DT_IMU 				40

#define K_ULTRASONIC 		0.001130f
#define UV_NUMSAMPLES 		5000

/* GOLDEN CONSTANTS!! DO NOT MODIFY!! */

//#define SOUND_ACTIVATION_SAMPLING_TIME_US		50
//#define SOUND_ACTIVATION_HIGH_BOUND_HERTZ		10300.0f
//#define SOUND_ACTIVATION_LOW_BOUND_HERTZ		7500.0f
//#define SOUND_ACTIVATION_RMS_CALC_BUFFER_SIZE	100U
//#define SOUND_ACTIVATION_RMS_THRESH				0.04f

/* Experimental constants: */

#define SOUND_ACTIVATION_SAMPLING_TIME_US		50
#define SOUND_ACTIVATION_HIGH_BOUND_HERTZ		10300.0f
#define SOUND_ACTIVATION_LOW_BOUND_HERTZ		7500.0f
#define SOUND_ACTIVATION_RMS_CALC_BUFFER_SIZE	100U
#define SOUND_ACTIVATION_RMS_THRESH				0.037f
/*
 * 325 seems to allow for about 3 feet to be the detection (and successful extinguishing) distance
 * 540 is a bit more conservative, and will require the detection distance to be brought down substantially
 * (as in, << 1 ft).
 */
#define UV_THRESHOLD 		525 //475

#define FIREFIGHT_TIMEOUT	10.0f // 10 seconds

#define ST_READY 			-1
#define ST_WANDER 			0
#define ST_HOMING 			1
#define ST_FIREFIGHT 		2
#define ST_CANDLE_BLOWOUT 	3
#define ST_DONE 			4
#define ST_REDO_FIREFIGHT	5

#include "stm32f30x.h"
#include "stm32f30x_adc.h"
#include "stm32f30x_gpio.h"
#include "stm32f30x_rcc.h"
#include "stm32f30x_misc.h"
#include "stm32f30x_tim.h"
#include "stm32f30x_usart.h"

#endif /* COMMON_HEADERS_H_ */
