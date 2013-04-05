/*
 * pwm.cpp
 *
 *  Created on: Feb 19, 2013
 *      Author: root
 */

#include "pwm.h"

void pwm_out1_init(uint16_t frequency)
{
	int period = 0;
	int psc = 0;
	config_pwm_freq((int)frequency, &period, &psc);


	TIM_TimeBaseInitTypeDef i;
	TIM_OCInitTypeDef j;
	GPIO_InitTypeDef g;

	i.TIM_CounterMode = TIM_CounterMode_Up;
	i.TIM_ClockDivision = 0;//clk_div;
	i.TIM_RepetitionCounter = 0;
	i.TIM_Prescaler = ((uint16_t)psc)-1;
	i.TIM_Period = ((uint16_t)period)-1;

	j.TIM_OCMode = TIM_OCMode_PWM1;
	j.TIM_OutputState = TIM_OutputState_Enable;
	j.TIM_Pulse = 8191;//25000;
	j.TIM_OCPolarity = TIM_OCPolarity_High;

	g.GPIO_Mode = GPIO_Mode_AF;
	g.GPIO_OType = GPIO_OType_PP;
	g.GPIO_Pin = GPIO_Pin_4;
	g.GPIO_PuPd = GPIO_PuPd_UP;
	g.GPIO_Speed = GPIO_Speed_50MHz;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_2); // AF2, 10:
	GPIO_Init(GPIOB, &g);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	TIM_InternalClockConfig(TIM3);
	TIM_TimeBaseInit(TIM3, &i);
	TIM_OC1Init(TIM3, &j);
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(TIM3, ENABLE);

	TIM_CCxCmd(TIM3, TIM_Channel_1, TIM_CCx_Enable);
	TIM_Cmd(TIM3, ENABLE);
	TIM_GenerateEvent(TIM3, TIM_EventSource_Update);
}

void pwm1_output(float duty)
{
	uint16_t setval = (uint16_t)((float)TIM3->ARR * duty);
	TIM_SetCompare1(TIM3, setval);
}

void pwm_out2_init(uint16_t frequency)
{
	int period = 0;
	int psc = 0;
	config_pwm_freq((int)frequency, &period, &psc);


	TIM_TimeBaseInitTypeDef i;
	TIM_OCInitTypeDef j;
	GPIO_InitTypeDef g;

	i.TIM_CounterMode = TIM_CounterMode_Up;
	i.TIM_ClockDivision = 0;//clk_div;
	i.TIM_RepetitionCounter = 0;
	i.TIM_Prescaler = ((uint16_t)psc)-1;
	i.TIM_Period = ((uint16_t)period)-1;

	j.TIM_OCMode = TIM_OCMode_PWM1;
	j.TIM_OutputState = TIM_OutputState_Enable;
	j.TIM_Pulse = 8191;
	j.TIM_OCPolarity = TIM_OCPolarity_High;

	g.GPIO_Mode = GPIO_Mode_AF;
	g.GPIO_OType = GPIO_OType_PP;
	g.GPIO_Pin = GPIO_Pin_5;
	g.GPIO_PuPd = GPIO_PuPd_UP;
	g.GPIO_Speed = GPIO_Speed_50MHz;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_2); // AF 2, 10:
	GPIO_Init(GPIOB, &g);


	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	TIM_InternalClockConfig(TIM3);
	TIM_TimeBaseInit(TIM3, &i);
	TIM_OC2Init(TIM3, &j);
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(TIM3, ENABLE);

	TIM_CCxCmd(TIM3, TIM_Channel_2, TIM_CCx_Enable);
	TIM_Cmd(TIM3, ENABLE);
	TIM_GenerateEvent(TIM3, TIM_EventSource_Update);
}

void pwm2_output(float duty)
{
	uint16_t setval = (uint16_t)((float)TIM3->ARR * duty);
	TIM_SetCompare2(TIM3, setval);
}
// PE0, TIM16:
void pwm_out3_init(uint16_t frequency)
{
	/*
	int period = 0;
	int psc = 0;
	config_pwm_freq((int)frequency, &period, &psc);


	TIM_TimeBaseInitTypeDef i;
	TIM_OCInitTypeDef j;
	GPIO_InitTypeDef g;

	i.TIM_CounterMode = TIM_CounterMode_Up;
	i.TIM_ClockDivision = 0;
	i.TIM_RepetitionCounter = 0;
	i.TIM_Prescaler = ((uint16_t)psc)-1;
	i.TIM_Period = ((uint16_t)period)-1;

	j.TIM_OCMode = TIM_OCMode_PWM1;
	j.TIM_OutputState = TIM_OutputState_Enable;
	j.TIM_Pulse = 8191;
	j.TIM_OCPolarity = TIM_OCPolarity_High;

	// PE0

	g.GPIO_Mode = GPIO_Mode_AF;
	g.GPIO_OType = GPIO_OType_PP;
	g.GPIO_Pin = GPIO_Pin_12;
	g.GPIO_PuPd = GPIO_PuPd_NOPULL;
	g.GPIO_Speed = GPIO_Speed_50MHz;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_4); // AF 1, 2, 3, or 9 (TIM15); 1, 4 (TIM16); 1, 2, 10 (TIM2); 2, 4, 6, 9, 11 (TIM1)
	GPIO_Init(GPIOA, &g);


	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM16, ENABLE);
	TIM_InternalClockConfig(TIM16);
	TIM_TimeBaseInit(TIM16, &i);
	TIM_OC1Init(TIM16, &j);
	TIM_OC1PreloadConfig(TIM16, TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(TIM16, ENABLE);

	TIM_CtrlPWMOutputs(TIM16, ENABLE);
	TIM_CCPreloadControl(TIM16, ENABLE);

	TIM_CCxCmd(TIM16, TIM_Channel_1, TIM_CCx_Enable);
	TIM_Cmd(TIM16, ENABLE);
	TIM_GenerateEvent(TIM16, TIM_EventSource_Update);
	*/

	///////////////////////////////////////////////////

	int period = 0;
	int psc = 0;
	config_pwm_freq((int)frequency, &period, &psc);


	TIM_TimeBaseInitTypeDef i;
	TIM_OCInitTypeDef j;
	GPIO_InitTypeDef g;

	i.TIM_CounterMode = TIM_CounterMode_Up;
	i.TIM_ClockDivision = 0;
	i.TIM_RepetitionCounter = 0;
	i.TIM_Prescaler = ((uint16_t)psc)-1;
	i.TIM_Period = ((uint16_t)period)-1;

	j.TIM_OCMode = TIM_OCMode_PWM1;
	j.TIM_OutputState = TIM_OutputState_Enable;
	j.TIM_Pulse = 8191;
	j.TIM_OCPolarity = TIM_OCPolarity_High;

	// PE0

	g.GPIO_Mode = GPIO_Mode_AF;
	g.GPIO_OType = GPIO_OType_PP;
	g.GPIO_Pin = GPIO_Pin_14;
	g.GPIO_PuPd = GPIO_PuPd_NOPULL;
	g.GPIO_Speed = GPIO_Speed_Level_1;//GPIO_Speed_50MHz;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOE, ENABLE);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_2); // AF 1, 2, 3, or 9 (TIM15); 1, 4 (TIM16); 1, 2, 10 (TIM2); 2, 4, 6, 9, 11, 12 (TIM1)
	GPIO_Init(GPIOE, &g);


	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	TIM_InternalClockConfig(TIM1);
	TIM_TimeBaseInit(TIM1, &i);
	TIM_OC4Init(TIM1, &j);
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(TIM1, ENABLE);

	TIM_CtrlPWMOutputs(TIM1, ENABLE);
//	TIM_CCPreloadControl(TIM1, ENABLE);

	TIM_CCxCmd(TIM1, TIM_Channel_4, TIM_CCx_Enable);
	TIM_Cmd(TIM1, ENABLE);
	TIM_GenerateEvent(TIM1, TIM_EventSource_Update);
}
void pwm3_output(float duty)
{
	uint16_t setval = (uint16_t)((float)TIM1->ARR * duty);
	TIM_SetCompare4(TIM1, setval);
}

void config_pwm_freq(int frequency, int* period, int* prescaler)
{
	int max = (int)((float)72000000/(float)frequency);

	int max_num = (int)sqrt(max);

	int seive[max_num];

	int startpos = 1;
	int testval = 1;

	int iter = 0;
	int iter2;
	for(iter = 0; iter < max_num; ++iter)
	{
		seive[iter] = iter+1;
	}

	while(startpos < max_num)
	{
		testval = seive[startpos];
		if(testval != -1)
		{
		for(iter = startpos+1; iter < max_num; ++iter)
		{
			if((seive[iter] != -1) && ((seive[iter])%testval == 0))
			{
				seive[iter] = -1;
			}
		}
		}
		++startpos;
	}

	int last_valid_iter;

	iter = 0;
	for(iter = 0; iter < max_num; ++iter)
	{
		if(seive[iter] != -1)
		{
		last_valid_iter = iter;
		}
		else
		{
			for(iter2 = iter; iter2<max_num; ++iter2)
			{
				if(seive[iter2] != -1)
				{
				seive[last_valid_iter+1] = seive[iter2];
				seive[iter2] = -1;
				iter = iter2;
				last_valid_iter += 1;
				iter2 = max_num+1;
				}
			}
		}
	}

	int i = max;
	int clkdiv = 1;
	int divisor = 1;

	while(i > 1)
	{
	for(iter = 1; iter < max_num; ++iter)
	{
		divisor = seive[iter];
		if(divisor == -1){break;}
		else
		{
		if(i%divisor == 0)
			{
			i /= divisor;
			clkdiv *= divisor;
			if(i < 65536)
				{
				*period = i;
				*prescaler = clkdiv;
				return;
				}
			}
		}
	}
	}
}
