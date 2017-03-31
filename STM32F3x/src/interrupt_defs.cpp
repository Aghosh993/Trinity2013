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
extern int new_data;
extern uint8_t adc2_new_data;
extern uint8_t adc3_awd1, adc3_awd2;

extern int count, stage;

extern float drive_cmd;// = 0.0f;
extern float err;// = 0.0f;
extern float last_err;// = 0.0f;
extern float diff_err;
extern float rt;// = 0.0f;
extern float d_front;// = 1.0f;
extern float integral;

extern float left, right;

extern __IO uint32_t adcData[2];
extern __IO uint32_t adc2_data[2];

extern int state;

extern int leds_on;
extern float match_time_counter, t_firefight_start, t_homing_start;

#define HOMING_TIME_LIMIT	8.0f

void update_pid(void);

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

		left_enc.position = left_temp.position + (int16_t)TIM_GetCounter(TIM4); //Instead of TIM2 on other boards with functional TIM2
		TIM_SetCounter(TIM4, 0);
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
		update_pid();
		if(state > ST_READY)
		{
			match_time_counter += (float)DT_ENCODER/(float)1000; // Increment main match time counter by DT_ENCODER ms
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

		/*
		uint8_t bytes[2];

		union twosComp {		// Takes care of two's complement conversion
			uint16_t un_signed;
			int16_t output;
		} convert;

		L3GD20_Read(bytes, L3GD20_OUT_X_L_ADDR, 2);
		convert.un_signed = (bytes[1] << 8) | bytes[0];

		gyro_angle_x += (convert.output - gyro_bias_x) * (float)0.00763 * (float)0.001 * (float)DT_IMU;
		*/

		/*
		 * Below: code required to trigger the ping sensor:
		 */

		EXTI_InitTypeDef e;

		e.EXTI_Line = EXTI_Line9;
		e.EXTI_LineCmd = DISABLE;
		e.EXTI_Mode = EXTI_Mode_Interrupt;
		e.EXTI_Trigger = EXTI_Trigger_Rising;

		EXTI_Init(&e);

		GPIO_InitTypeDef g;
		g.GPIO_Mode = GPIO_Mode_OUT;
		g.GPIO_OType = GPIO_OType_PP;
		g.GPIO_Pin = GPIO_Pin_0;
		g.GPIO_PuPd = GPIO_PuPd_NOPULL;
		g.GPIO_Speed = GPIO_Speed_Level_1;

		GPIO_Init(GPIOB, &g);
		int iter = 0;

		for(iter=0;iter<10;++iter)
		{
			GPIO_WriteBit(GPIOB, GPIO_Pin_0, Bit_SET);
		}

		GPIO_WriteBit(GPIOB, GPIO_Pin_0, Bit_RESET);


		g.GPIO_Mode = GPIO_Mode_IN;
		g.GPIO_OType = GPIO_OType_OD;
		g.GPIO_Pin = GPIO_Pin_0;
		g.GPIO_PuPd = GPIO_PuPd_NOPULL;
		g.GPIO_Speed = GPIO_Speed_Level_1;

		GPIO_Init(GPIOB, &g);

		TIM_SetCounter(TIM2, 0);

		stage = 0;

		e.EXTI_Line = EXTI_Line9;
		e.EXTI_LineCmd = ENABLE;
		e.EXTI_Mode = EXTI_Mode_Interrupt;
		e.EXTI_Trigger = EXTI_Trigger_Rising;

		EXTI_Init(&e);

		NVIC_InitTypeDef nv;

		nv.NVIC_IRQChannel = EXTI9_5_IRQn;
		nv.NVIC_IRQChannelCmd = ENABLE;
		nv.NVIC_IRQChannelPreemptionPriority = 0;
		nv.NVIC_IRQChannelSubPriority = 0;

		NVIC_Init(&nv);

	}
	void TIM6_DAC_IRQHandler(void)
	{
		TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
		if(state == ST_READY)
		{
			GPIO_Write(GPIOE, led_matrix[2] | led_matrix[5]); 	// Two greens
		}
		else if(state == ST_WANDER)
		{
			GPIO_Write(GPIOE, led_matrix[6] | led_matrix[0]);					// Two Orange (one's lit via PWM pin for ESC control)
		}
		else if(state == ST_HOMING)
		{
			GPIO_Write(GPIOE, led_matrix[0] | led_matrix[1] | led_matrix[3]);	// Two orange + Two red
		}
		else if(state == ST_FIREFIGHT)
		{
			GPIO_Write(GPIOE, led_matrix[0] | led_matrix[1] | led_matrix[3] | led_matrix[6]);
		}
		else if (state == ST_CANDLE_BLOWOUT)
		{
			GPIO_Write(GPIOE, led_matrix[0] | led_matrix[1] | led_matrix[2] | led_matrix[3] |
							  led_matrix[4] | led_matrix[5] | led_matrix[6]); // All LED's
		}
		else if (state == ST_DONE)
		{
			++led_iter;
			GPIO_Write(GPIOE, led_matrix[led_iter] | led_matrix[6-led_iter]);
			if(led_iter > 6)
			{
				led_iter = 0;
			}
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

	void DMA2_Channel1_IRQHandler(void)
	{
		DMA_ClearITPendingBit(DMA2_IT_TC1);
		adc2_new_data = 1;
	}

	void ADC3_IRQHandler(void)
	{
		int i = 0;
		int sum = 0;
		if(ADC_GetITStatus(ADC3, ADC_IT_AWD1) != RESET)
		{
			ADC_ClearITPendingBit(ADC3, ADC_IT_AWD1);
			for(i=0; i<35;++i)
			{
				sum += ADC_GetConversionValue(ADC3);
			}
			if((float)sum/float(35) > 2048)
			{
				adc3_awd1 = 1;
			}
		}
		else if (ADC_GetITStatus(ADC3, ADC_IT_AWD2) != RESET)
		{
			ADC_ClearITPendingBit(ADC3, ADC_IT_AWD2);
			adc3_awd2 = 1;
		}
	}

	void EXTI9_5_IRQHandler(void)//EXTI5_IRQHandler(void)
	{
		EXTI_ClearITPendingBit(EXTI_Line9);
		if(stage == 0)
		{
			TIM_SetCounter(TIM2, 0);
			TIM_Cmd(TIM2, ENABLE);

			EXTI_InitTypeDef e;

			e.EXTI_Line = EXTI_Line9;
			e.EXTI_LineCmd = ENABLE;
			e.EXTI_Mode = EXTI_Mode_Interrupt;
			e.EXTI_Trigger = EXTI_Trigger_Falling;

			EXTI_Init(&e);

			stage = 1;
			return;
		}
		else if(stage == 1)
		{
			count = TIM_GetCounter(TIM2);
			TIM_Cmd(TIM2, DISABLE);

			NVIC_InitTypeDef nv;

			nv.NVIC_IRQChannel = EXTI9_5_IRQn;
			nv.NVIC_IRQChannelCmd = DISABLE;
			nv.NVIC_IRQChannelPreemptionPriority = 0;
			nv.NVIC_IRQChannelSubPriority = 0;

			NVIC_Init(&nv);

			stage = 0;
			return;
		}
	}

	void update_pid(void)
	{
		int i = 0;
		float mtr_out = 0.50f;
		d_front = ((float)count*(float)0.5*(float)K_ULTRASONIC);

		if(state == ST_HOMING)
		{
			err = (float)(1.0) * (float)((int)adcData[0] - (int)adc2_data[2]);
			if((err > -20 && err < 20))// || match_time_counter-t_homing_start > HOMING_TIME_LIMIT)
			{
				pwm1_output(0.50f);
				pwm2_output(0.50f);
				state = ST_FIREFIGHT;
				return;
			}

			diff_err = (float)(err-last_err)*((float)DT_ENCODER/(float)1000);
			integral += err * 0.00004f;
			drive_cmd = (((float)(err)/(float)3400) + ((float)diff_err/(float)835)); // k_deriv = 944

			rt = 0.0f;
			mtr_out = 0.5f;

			if(drive_cmd > 0.5)
			{
				drive_cmd = 0.5f;
			}
			if(drive_cmd < -0.5)
			{
				drive_cmd = -0.5f;
			}

			left = (1-mtr_out) - drive_cmd - rt;
			right = mtr_out - drive_cmd - rt;

			if(left>1.0)
			{
				left=1.0;
			}
			else if(left<0)
			{
				left=0;//-1.0;
			}

			if(right>1.0f)
			{
				right=1.0f;
			}
			else if(right<0)
			{
				right=0;
			}
		}
		else if(state == ST_WANDER)
		{
			err = ((float)(0.55)*(float)((int)adc2_data[0] - 2200)) + ((float)(0.45)*(float)((int)adcData[1] - 150));

		diff_err = (float)(err-last_err)*((float)DT_ENCODER/(float)1000);
		drive_cmd = (((float)(err)/(float)2500) + ((float)diff_err/(float)3720)); //1100=diff term

		if(d_front < 0.390 || adc2_data[3] > 2000)//1870)//0.69
		{
			rt = 0.5f;
			mtr_out = 0.3f;
			drive_cmd = 0;
		}
		else
		{
			rt = 0.0f;
			mtr_out = 0.85f;
		}

		if(drive_cmd > 0.5)
		{
			drive_cmd = 0.5f;
		}
		if(drive_cmd < -0.5)
		{
			drive_cmd = -0.5f;
		}

		left = (1-mtr_out) - drive_cmd - rt;
		right = mtr_out - drive_cmd - rt;

		if(left>1.0)
		{
			left=1.0;
		}
		else if(left<0)
		{
			left=0;//-1.0;
		}

		if(right>1.0f)
		{
			right=1.0f;
		}
		else if(right<0)
		{
			right=0;
		}
		}
		else if(state == ST_CANDLE_BLOWOUT)
		{
			if((adc2_data[2] > UV_THRESHOLD || adcData[0] > UV_THRESHOLD))
			{
//				err = (float)(1.0) * (float)((int)adcData[0] - (int)adc2_data[2]);
//
//				diff_err = (float)(err-last_err)*((float)DT_ENCODER/(float)1000);
//				integral =0;//+= err * 0.04f;
//				drive_cmd = (((float)(err)/(float)500) + ((float)diff_err/(float)944));
////				drive_cmd = (((float)(err)/(float)350) + ((float)diff_err/(float)944));
//				rt = 0.0f;
//				mtr_out = 0.5f;
//
//				if(drive_cmd > 0.5)
//				{
//					drive_cmd = 0.5f;
//				}
//				if(drive_cmd < -0.5)
//				{
//					drive_cmd = -0.5f;
//				}
//
//				left = (1-mtr_out) - drive_cmd - rt;
//				right = mtr_out - drive_cmd - rt;
//
//				if(left>1.0)
//				{
//					left=1.0;
//				}
//				else if(left<0)
//				{
//					left=0;//-1.0;
//				}
//
//				if(right>1.0f)
//				{
//					right=1.0f;
//				}
//				else if(right<0)
//				{
//					right=0;
//				}
				left = 0.5f;
				right = 0.5f;
			}
			else
			{
				left = 0.5f;
				right = 0.5f;
				state = ST_DONE;
			}

			if(match_time_counter - t_firefight_start > FIREFIGHT_TIMEOUT)
			{
				// Turn off motors:
				pwm1_output(0.5f);
				pwm2_output(0.5f);

				// Turn off the fan:
				pwm3_output(0.05f);

				for(i=0;i<40000000;++i)
				{
					++i;
				}

				if((adc2_data[2] > UV_THRESHOLD || adcData[0] > UV_THRESHOLD))
				{
					/*
					match_time_counter = 0.0f;

					// Turn off the fan:
					// pwm3_output(0.05f);

					// Wait for flame to settle, if it's still there...
					// Twitch robot in some other direction
					pwm1_output(0.85f);
					pwm2_output(0.00f);
					while(match_time_counter < 1.5f)
					{
						if(match_time_counter > 1.5f)
						{
							pwm1_output(0.5f);
							pwm2_output(0.5f);
							break;
						}
					}*/

					pwm1_output(0.0f);
					pwm2_output(0.85f);
					match_time_counter = 0.0f;
					state = ST_REDO_FIREFIGHT;
				}
				else
				{
					pwm1_output(0.5f);
					pwm2_output(0.5f);
					rt = 0;
					state = ST_DONE;
				}
			}
		}
		if(state == ST_REDO_FIREFIGHT)
		{
			if(match_time_counter > 1.5f)
			{
				pwm1_output(0.5f);
				pwm2_output(0.5f);
				state = ST_HOMING;
			}
		}
		if(state == ST_WANDER || state == ST_HOMING || state == ST_CANDLE_BLOWOUT) {
		pwm1_output(left);
		pwm2_output(right);
		last_err = err;
		}
	}

}


