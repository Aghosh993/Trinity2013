/*
 * encoder.c
 *
 *  Created on: Feb 19, 2013
 *      Author: root
 */
#include "encoder.h"

void TIM8_init_encoder(void)	// PC6, PC7 -> TIM8 CH1 and CH2 mappings:
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//UP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_4);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_4);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_EncoderInterfaceConfig(TIM8,TIM_EncoderMode_TI12,TIM_ICPolarity_Falling,
	TIM_ICPolarity_Falling);
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);
	TIM_Cmd(TIM8, ENABLE);

	TIM_SetCounter(TIM8, 0);
}

void TIM2_init_encoder(void) // PA0, PA1 -> TIM2 CH1 and CH2 mappings:
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_1);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	TIM_TimeBaseStructure.TIM_Period = 0xFFFFFFFF;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_EncoderInterfaceConfig(TIM2,TIM_EncoderMode_TI12,TIM_ICPolarity_Falling,
	TIM_ICPolarity_Falling);
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	TIM_Cmd(TIM2, ENABLE);

	TIM_SetCounter(TIM2, 0);
}

// Initializes the ISR that updates the raw state (position, velocity, acceleration) of each encoder.
// Interrupt Priority: NEXT-HIGHEST (1)

void encoder_update_ISR_init(void)
{
	NVIC_InitTypeDef nv;
	TIM_TimeBaseInitTypeDef TIM7_init;

	nv.NVIC_IRQChannel = TIM7_IRQn;
	nv.NVIC_IRQChannelPreemptionPriority = 1;
	nv.NVIC_IRQChannelSubPriority = 0;
	nv.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init(&nv);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE); //TIM2 on APB1

	/*
	 * UPDATE_FREQUENCY = TIM_CLK/[(PRESCALER+1)(ARR+1)(REPCOUNTER+1)]
	 * 	  Prescaler -> PRESCALER
	 * 	  Period -> ARR
	 * 	  RepetitionCounter -> REPCOUNTER
	 */

	TIM7_init.TIM_Period = (10*DT_ENCODER)-1;
	TIM7_init.TIM_Prescaler = 7199;
	TIM7_init.TIM_RepetitionCounter = 0;
	TIM7_init.TIM_ClockDivision = 0;
	TIM7_init.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM7, &TIM7_init);

	TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM7, ENABLE);

}
void init_encoder_struct(encoderState* enc)
{
	enc->position = 0;
	enc->speed = 0;
	enc->acceleration = 0;
	enc->m = MODE_OPENLOOP;
	enc->integral = 0;
	enc->position_target = 0;
	enc->speed_target = 0;
	enc->last_error = 0;
	enc->last_speed_error = 0;
	enc->vel_cmd = 0.5f;
}




