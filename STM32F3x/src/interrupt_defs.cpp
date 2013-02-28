/*
 * interrupt_defs.cpp
 *
 *  Created on: Feb 19, 2013
 *      Author: root
 */

#include "common_headers.h"
#include "stm32f3_discovery_l3gd20.h"

#include "encoder.h"

// ISR definitions need to go here, to avoid C++ name-mangling

extern "C"
{
extern volatile float gyro_angle_x;
extern encoderState left_enc, right_enc;
extern int gyro_bias_x, adcval;
extern volatile int led_matrix[8];
extern volatile int led_iter;

	void TIM7_IRQHandler(void) // ISR that performs encoder state update:
										// Runs every DT milliseconds
	{
		TIM_ClearITPendingBit(TIM7, TIM_IT_Update);

		encoderState left_temp, right_temp;

		left_temp.position = left_enc.position;
		left_temp.speed = left_enc.speed;
		left_temp.acceleration = left_enc.acceleration;

		right_temp.position = right_enc.position;
		right_temp.speed = right_enc.speed;
		right_temp.acceleration = right_enc.acceleration;

		left_enc.position = TIM_GetCounter(TIM2);
		left_enc.speed = (float)((left_enc.position - left_temp.position)*1000)/(float)DT_ENCODER; // Since DT is in milliseconds...
		left_enc.acceleration = (float)((left_enc.speed - left_temp.speed)*1000)/(float)DT_ENCODER;

		right_enc.position = right_temp.position + (int16_t)(TIM_GetCounter(TIM8));
		TIM_SetCounter(TIM8, 0); // To get around the stupid 16-bit counter limitation present on all timers except timer 2
		right_enc.speed = (float)((right_enc.position - right_temp.position)*1000)/(float)DT_ENCODER;
		right_enc.acceleration = (float)((right_enc.speed - right_temp.speed)*1000)/(float)DT_ENCODER;
	}

	void TIM1_TRG_COM_TIM17_IRQHandler(void)
	{
		TIM_ClearITPendingBit(TIM17, TIM_IT_Update);
		uint8_t bytes[2];

		union twosComp {		// Takes care of two's complement conversion
			uint16_t un_signed;
			int16_t output;
		} convert;

		L3GD20_Read(bytes, L3GD20_OUT_X_L_ADDR, 2);
		convert.un_signed = (bytes[1] << 8) | bytes[0];

		gyro_angle_x += (convert.output - gyro_bias_x) * (float)0.00763 * (float)0.001 * (float)DT_IMU;
	}
	void TIM6_DAC_IRQHandler(void)
	{
		TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
		GPIO_Write(GPIOE, led_matrix[led_iter] | led_matrix[7-led_iter]);
		++led_iter;
		if(led_iter > 7)
		{
			led_iter = 0;
		}
	}
	void ADC1_2_IRQHandler(void)
	{
		if(ADC_GetITStatus(ADC1, ADC_IT_EOC) == SET)
		{
			adcval = ADC_GetConversionValue(ADC1);
			ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
		}
	}
}


