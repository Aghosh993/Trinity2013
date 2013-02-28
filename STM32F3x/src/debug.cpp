/*
 * debug.cpp
 *
 *  Created on: Feb 19, 2013
 *      Author: root
 */

#include "debug.h"

void blink_leds(void)
{
	int foo = 0;
	for(foo = 0; foo < 160000; ++foo){
		GPIO_WriteBit(GPIOE, GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_15, Bit_SET);
	}
	for(foo = 0; foo < 160000; ++foo){
		GPIO_WriteBit(GPIOE, GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_15, Bit_RESET);
	}
}

void UART1_init(void)
{
	// USART1 Init:

	GPIO_InitTypeDef a;

	USART_InitTypeDef u1;

	u1.USART_BaudRate = 115200;
	u1.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	u1.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	u1.USART_Parity = USART_Parity_No;
	u1.USART_StopBits = USART_StopBits_1;
	u1.USART_WordLength = USART_WordLength_8b;

	a.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
	a.GPIO_Mode = GPIO_Mode_AF;
	a.GPIO_OType = GPIO_OType_PP;
	a.GPIO_Speed = GPIO_Speed_50MHz;
	a.GPIO_PuPd = GPIO_PuPd_NOPULL;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_7);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_7);

	GPIO_Init(GPIOA, &a);

	USART_Init(USART1, &u1);
	USART_Cmd(USART1, ENABLE);

	/* Wait until Receive enable acknowledge flag is set */
	while(USART_GetFlagStatus(USART1, USART_FLAG_REACK) == RESET)
	{}

	/* Wait until Transmit enable acknowledge flag is set */
	while(USART_GetFlagStatus(USART1, USART_FLAG_TEACK) == RESET)
	{}
}



/* Initializes the ISR that controls the LED circle on the board to provide visual feedback to the user,
 * along with some eye candy
 * Priority: 6 (LOW)
 */


void LED_MATRIX_ISR_init(void)
{
	// GPIO Init:

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOE, ENABLE);

	GPIO_InitTypeDef a;

	a.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	a.GPIO_Mode = GPIO_Mode_OUT;
	a.GPIO_PuPd = GPIO_PuPd_NOPULL;
	a.GPIO_OType = GPIO_OType_PP;
	a.GPIO_Speed = GPIO_Speed_Level_2;

	GPIO_Init(GPIOE, &a);

	NVIC_InitTypeDef nv;
	TIM_TimeBaseInitTypeDef TIM6_init;

	nv.NVIC_IRQChannel = TIM6_DAC_IRQn;
	nv.NVIC_IRQChannelPreemptionPriority = 6; // Low-priority interrupt since this is just eye-candy...
	nv.NVIC_IRQChannelSubPriority = 0;
	nv.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init(&nv);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE); //Enable TIM4 on APB1

	/*
	 * UPDATE_FREQUENCY = TIM_CLK/[(PRESCALER+1)(ARR+1)(REPCOUNTER+1)]
	 * 	  Prescaler -> PRESCALER
	 * 	  Period -> ARR
	 * 	  RepetitionCounter -> REPCOUNTER
	 */

	TIM6_init.TIM_Period = (10*DT_LED_MATRIX)-1;
	TIM6_init.TIM_Prescaler = 7199;
	TIM6_init.TIM_RepetitionCounter = 0;
	TIM6_init.TIM_ClockDivision = 0;
	TIM6_init.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM6, &TIM6_init);

	TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM6, ENABLE);
}

