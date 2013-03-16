/*
 * interrupt_defs.cpp
 *
 *  Created on: Feb 19, 2013
 *      Author: root
 */

#include "common_headers.h"
#include "stm32f3_discovery_l3gd20.h"

#include "encoder.h"

//#include <stdio.h>
#include <math.h>

// ISR definitions need to go here, to avoid C++ name-mangling

extern "C"
{
extern volatile float gyro_angle_x;
extern encoderState left_enc, right_enc;
extern int gyro_bias_x, adcval;
extern volatile int led_matrix[8];
extern volatile int led_iter;
extern __IO int adcData;
extern int new_data;

	void TIM7_IRQHandler(void) // ISR that performs encoder state update:
										// Runs every DT milliseconds
	{
		float left_out, right_out;
		int error, abs_err;

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

		if(left_enc.m == MODE_POSITION)
		{
			left_out = ((float)k_p * (float)(left_enc.position_target - left_enc.position))
					+ ((float)k_i*(float)(left_enc.integral))
					+ ((float)k_d*(float)left_enc.speed);
			left_out = (left_out > 100) ? 100 : ((left_out < -100) ? -100 : left_out);

			pwm2_output(100-((float)(left_out+100)/(float)2));
		}
		else if(left_enc.m == MODE_SPEED)
		{
			pwm2_output(0);
		}

		if(right_enc.m == MODE_POSITION)
		{
			error = right_enc.position - right_enc.position_target;
			right_enc.integral += (float)(error) * (float)DT_ENCODER / (float)1000;

			if(right_enc.integral * k_i > 100 || right_enc.integral * k_i < -100)
			{
				right_enc.integral = (float)100/(float)k_i;
			}

			right_out = ((float)k_p * (float)error)
					+ ((float)k_i*(float)(right_enc.integral))
					+ ((float)k_d*(float)(error - right_enc.last_error)/(float)(DT_ENCODER/(float)1000));
			right_out = (right_out > 100) ? 100 : ((right_out < -100) ? -100 : right_out);
			right_enc.last_error = error;

			abs_err = (error > 0) ? error : (error*-1);

			if(error > 10 || error < -10)
			{
				pwm1_output((((float)(right_out+100)/(float)2))*(float)0.01);
			}
			else
			{
				pwm1_output(0.50f);
				right_enc.m = MODE_OPENLOOP;
			}
		}
		/*
		else if(right_enc.m == MODE_SPEED)
		{
			error = right_enc.speed - right_enc.speed_target;

			right_out = ((float)error * (float)k_p_s);// + ((float)k_d_s * (float)(error - right_enc.last_speed_error)/(float)((float)DT_ENCODER/(float)1000));
//			right_out = (right_out > 100) ? 100 : ((right_out < -100) ? -100 : right_out);
			right_enc.vel_cmd += right_out;
			if(right_enc.vel_cmd > 1)
			{
				right_enc.vel_cmd = 1;
			}

			else if(right_enc.vel_cmd < 0)
			{
				right_enc.vel_cmd = 0;
			}

//			pwm1_output((float)1.0-right_enc.vel_cmd);
			pwm1_output(0.50f);

			right_enc.last_speed_error = error;
		}
		*/
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

	void DMA1_Channel1_IRQHandler(void)
	{
		DMA_ClearITPendingBit(DMA1_IT_TC1);
		new_data = 1;
	}
}


