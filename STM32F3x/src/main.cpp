#include "stm32f30x_adc.h"
#include "stm32f30x_gpio.h"
#include "stm32f30x_dma.h"

#include "stm32f3_discovery_l3gd20.h"

#include "encoder.h"
#include "pwm.h"
#include "debug.h"

#include <stdio.h>
#include <stdint.h>
#include <math.h>

/*
 * Timer usage manifest:
 * TIM2: 32-bit encoder interface input (PC6, PC7) (NOT ON TRINITY BOARD DUE TO HARDWARE FAULT!!)
 * TIM4: 16-bit encoder interface input (PD12, PD13) (ONLY ON TRINITY BOARD, DUE TO ABOVE!!)
 * TIM8: 16-bit encoder interface input (PA0, PA1)
 *
 * TIM3: PWM Output Channels 1 and 2 	(PB4, PB5)
 *
 * TIM6: LED matrix ISR
 * TIM7: Encoder update ISR
 * TIM17: IMU update ISR
 *
 * Other pins/resources used:
 *
 * Analog: 	PA2: adcData[0]
 * 			PA3: adcData[1]
 *
 * 			PA4: adc2_data[0]
 * 			PC4: adc2_data[1]
 * 			PC5: adc2_data[2]
 * 			PB2: adc2_data[3]
 *
 * Debug port information: Baud Rate=115200, 8n1 UART (TTL pins PA9, PA10)
 * 							PA9 = TX, PA10 = RX (W/respect to STM32 device)
 *
 * Purpose: Trinity 2013 (Theseus) Primary code development tree
 *
 */

/*
 * ADC TODO: Use PF4 for third channel in DMA setup for ADC1, then attempt to get DMA + continuous
 * conversion up and running in ADC3...? (Need more detailed datasheet reference...)
 */

volatile int led_iter;
volatile int led_matrix[8] = {GPIO_Pin_9, GPIO_Pin_8, GPIO_Pin_15, GPIO_Pin_14, GPIO_Pin_13,
										GPIO_Pin_12, GPIO_Pin_11, GPIO_Pin_10};

void imu_update_ISR_init(void);
int16_t calc_gyro_bias(void);

uint32_t L3GD20_TIMEOUT_UserCallback(void);

void adc1_init_DMA(void);
void adc2_init_DMA(void);
void battery_watchdog_init(void);
void adc1_init(void);

void brake_pins_init(void);

void ping_pin_init(void);
void timer2_timebase_init(void);

void comp_init(void);

float IR_distance(int IR_ADC_VAL);

// Global variables to keep track of encoders and inertial sensors:

encoderState left_enc, right_enc;

volatile float gyro_angle_x;
int gyro_bias_x, adcval;

__IO uint32_t adcData[2];
int new_data;

__IO uint32_t adc2_data[4];
uint8_t adc2_new_data;

uint8_t adc3_awd1, adc3_awd2;

int count, stage;

float drive_cmd;// = 0.0f;
float err;// = 0.0f;
float last_err;// = 0.0f;
float diff_err;
float rt;// = 0.0f;
float d_front;// = 1.0f;
float integral;

float left, right;

int state;

// Initialize all encoder data structures to zero:

int main(void)
{
	SystemInit(); // Set up clocks/PLL/et. al

	UART1_init(); // Debug bridge

	state = ST_HOMING; //ST_WANDER;

	// Initialize ADC data buffers to 0;

	adcData[0] = 0;
	adcData[1] = 0;

	uint8_t iter = 0;
	for(iter=0; iter < 7; ++iter)
	{
		adc2_data[iter] = 0;
	}

	adc3_awd1 = 0;
	adc3_awd2 = 0;

	count = 0;
	stage = 0;

	/*
	 * Initialize global encoder data structure for left and right encoders,
	 * and set position/speed targets as necessary:
	 */

	init_encoder_struct(&left_enc);
	init_encoder_struct(&right_enc);

	// Initialize PWM outputs 1 and 2 at 5.0 kHz duty frequency:

	brake_pins_init();
	pwm_out1_init(2000);
	pwm_out2_init(2000);

	// Initialize hardware quadrature encoder input interfaces:

	TIM8_init_encoder();
	TIM4_init_encoder();

	// Initialize ADC, encoder update, IMU update and LED matrix interrupts:

	new_data = 0;
	adc2_new_data = 0;

	// Initialize ADC1 DMA:
	adc1_init_DMA();

	// Initialize ADC2 DMA:
	adc2_init_DMA();
//	battery_watchdog_init();

	encoder_update_ISR_init();	//Update the state of the two encoders (left/right)
	LED_MATRIX_ISR_init();		//Hand out some eye candy while we're at it...

	// Ping Sensor Init:

	ping_pin_init();
	timer2_timebase_init();
	imu_update_ISR_init();

	// Encoder comparator init for noise debounce:
	comp_init();

	adcval = 0;
//	mtr_out = 0;

	drive_cmd = 0.0f;
	err = 0.0f;
	last_err = 0.0f;
	diff_err = 0.0f;
	rt = 0.0f;
	d_front = 1.0f;
	integral = 0.0f;

//	float left, right;

	while(true)
	{
//		printf("Front: %1.3f, Front left:%1.4f, Front back:%1.4f\n\r", d_front, IR_distance(adc2_data[0]), IR_distance(adcData[1]));
		printf("%4d %4d %4d\n\r", (int)adcData[0] , (int)adc2_data[2], ((int)adcData[0] - (int)adc2_data[2]));

	}
	return 0; // We should never manage to get here...
}

float IR_distance(int IR_ADC_VAL)
{
	float v_sensor_actual = (float)0.0008698 * (float)IR_ADC_VAL;
	return (((float)0.632)*((float)pow(v_sensor_actual,6))-((float)8.012)*((float)pow(v_sensor_actual,5))
			+ ((float)41.05)*((float)pow(v_sensor_actual,4)) - ((float)109.7)*((float)pow(v_sensor_actual,3))
			+ ((float)164.7)*((float)pow(v_sensor_actual,2)) - (((float)138.8)*(float)v_sensor_actual)
			+ (float)60.24);
}

// Initializes the ISR that reads inertial sensors (gyro + accelerometer)
// Interrupt priority is HIGHEST (0)

void imu_update_ISR_init(void)
{
	L3GD20_InitTypeDef l3gInit;

	l3gInit.Axes_Enable = L3GD20_AXES_ENABLE;
	l3gInit.Band_Width = L3GD20_BANDWIDTH_1;
	l3gInit.BlockData_Update = L3GD20_BlockDataUpdate_Continous;
	l3gInit.Endianness = L3GD20_BLE_LSB;
	l3gInit.Full_Scale = L3GD20_FULLSCALE_250;
	l3gInit.Output_DataRate = L3GD20_OUTPUT_DATARATE_1;
	l3gInit.Power_Mode = L3GD20_MODE_ACTIVE;

	L3GD20_Init(&l3gInit);
	gyro_bias_x = calc_gyro_bias();

	gyro_angle_x = 0;

	NVIC_InitTypeDef nv;
	TIM_TimeBaseInitTypeDef TIM17_init;

	nv.NVIC_IRQChannel = TIM1_TRG_COM_TIM17_IRQn;
	nv.NVIC_IRQChannelPreemptionPriority = 0;
	nv.NVIC_IRQChannelSubPriority = 0;
	nv.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init(&nv);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM17, ENABLE);

	/*
	 * UPDATE_FREQUENCY = TIM_CLK/[(PRESCALER+1)(ARR+1)(REPCOUNTER+1)]
	 * 	  Prescaler -> PRESCALER
	 * 	  Period -> ARR
	 * 	  RepetitionCounter -> REPCOUNTER
	 */

	TIM17_init.TIM_Period = (10*DT_IMU)-1;
	TIM17_init.TIM_Prescaler = 7199;
	TIM17_init.TIM_RepetitionCounter = 0;
	TIM17_init.TIM_ClockDivision = 0;
	TIM17_init.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM17, &TIM17_init);

	TIM_ITConfig(TIM17, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM17, ENABLE);
}

uint32_t L3GD20_TIMEOUT_UserCallback(void)
{
	printf("L3GD20 read attempt timed out... check your wiring/code!!!\n\r");
	return 0;
}

void brake_pins_init(void)
{
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOE, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_2;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	GPIO_WriteBit(GPIOE, GPIO_Pin_2, Bit_RESET);
	GPIO_WriteBit(GPIOE, GPIO_Pin_3, Bit_RESET);

	/*
	 * 	a.GPIO_Mode = GPIO_Mode_OUT;
	a.GPIO_PuPd = GPIO_PuPd_NOPULL;
	a.GPIO_OType = GPIO_OType_PP;
	a.GPIO_Speed = GPIO_Speed_Level_2;

	 */
}
/*
 * PB0: OUTPUT
 * PC1: INPUT
 */
void ping_pin_init(void)
{

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOF, ENABLE);

	GPIO_InitTypeDef g;
	g.GPIO_Mode = GPIO_Mode_IN;
	g.GPIO_OType = GPIO_OType_OD;
	g.GPIO_Pin = GPIO_Pin_9;
	g.GPIO_PuPd = GPIO_PuPd_NOPULL;
	g.GPIO_Speed = GPIO_Speed_Level_1;

	GPIO_Init(GPIOF, &g);

	g.GPIO_Mode = GPIO_Mode_OUT;
	g.GPIO_OType = GPIO_OType_PP;
	g.GPIO_Pin = GPIO_Pin_0;
	g.GPIO_PuPd = GPIO_PuPd_NOPULL;
	g.GPIO_Speed = GPIO_Speed_Level_1;

	GPIO_Init(GPIOB, &g);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOF, EXTI_PinSource9);

}

void timer2_timebase_init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	TIM_TimeBaseStructure.TIM_Period = 17999;//1899;//0xFFFFFFFF;//1899;//0xFFFFFFFF;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseStructure.TIM_Prescaler = 71;//0;//71;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
//	TIM_Cmd(TIM2, ENABLE);

	TIM_SetCounter(TIM2, 0);
}

/*
 * Use COMP2, COMP3, COMP4, COMP7
 * COMP2: 	Input->		PA7  (IO1)
 * 			Output->	PB9
 * COMP3:	Input->		PB14 (IO1)
 * 			Output->	PA8
 * COMP4:	Input->		PE7  (IO2)
 * 			Output->	PB1
 * COMP7:	Input->		PC1	 (IO2)
 * 			Output->	PC2
 */

void comp_init(void)
{
	//COMP2 Init:

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	GPIO_InitTypeDef g;

	g.GPIO_Mode = GPIO_Mode_AN;
	g.GPIO_PuPd = GPIO_PuPd_NOPULL;
	g.GPIO_Pin = GPIO_Pin_7;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	GPIO_Init(GPIOA, &g);

	g.GPIO_Mode = GPIO_Mode_AF;
	g.GPIO_OType = GPIO_OType_PP;
	g.GPIO_PuPd = GPIO_PuPd_NOPULL;
	g.GPIO_Pin = GPIO_Pin_9;

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_8);
	GPIO_Init(GPIOB, &g);

	RCC_PCLK2Config(RCC_HCLK_Div1);

	COMP_InitTypeDef c;

	c.COMP_InvertingInput = COMP_InvertingInput_VREFINT;
	c.COMP_NonInvertingInput = COMP_NonInvertingInput_IO1;
	c.COMP_Mode = COMP_Mode_MediumSpeed;
	c.COMP_Output = COMP_Output_None;
	c.COMP_OutputPol = COMP_OutputPol_NonInverted;
	c.COMP_Hysteresis = COMP_Hysteresis_Low;
	c.COMP_BlankingSrce = COMP_BlankingSrce_None;

	COMP_Init(COMP_Selection_COMP2, &c);
	COMP_Cmd(COMP_Selection_COMP2, ENABLE);

	//COMP3 Init:

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	g.GPIO_Mode = GPIO_Mode_AN;
	g.GPIO_PuPd = GPIO_PuPd_NOPULL;
	g.GPIO_Pin = GPIO_Pin_14; 	//PB14 = INPUT

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOA, ENABLE);
	GPIO_Init(GPIOB, &g);

	g.GPIO_Mode = GPIO_Mode_AF;
	g.GPIO_OType = GPIO_OType_PP;
	g.GPIO_PuPd = GPIO_PuPd_NOPULL;
	g.GPIO_Pin = GPIO_Pin_8; 	//PA8 = OUTPUT

	GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_7);
	GPIO_Init(GPIOC, &g);

	RCC_PCLK2Config(RCC_HCLK_Div1);

	c.COMP_InvertingInput = COMP_InvertingInput_VREFINT;
	c.COMP_NonInvertingInput = COMP_NonInvertingInput_IO1;
	c.COMP_Mode = COMP_Mode_MediumSpeed;
	c.COMP_Output = COMP_Output_None;
	c.COMP_OutputPol = COMP_OutputPol_NonInverted;
	c.COMP_Hysteresis = COMP_Hysteresis_Low;
	c.COMP_BlankingSrce = COMP_BlankingSrce_None;

	COMP_Init(COMP_Selection_COMP3, &c);
	COMP_Cmd(COMP_Selection_COMP3, ENABLE);

	//COMP4 Init:

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	g.GPIO_Mode = GPIO_Mode_AN;
	g.GPIO_PuPd = GPIO_PuPd_NOPULL;
	g.GPIO_Pin = GPIO_Pin_7; 	//PE7 = INPUT

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOE | RCC_AHBPeriph_GPIOB, ENABLE);
	GPIO_Init(GPIOE, &g);

	g.GPIO_Mode = GPIO_Mode_AF;
	g.GPIO_OType = GPIO_OType_PP;
	g.GPIO_PuPd = GPIO_PuPd_NOPULL;
	g.GPIO_Pin = GPIO_Pin_1; 	//PB1 = OUTPUT

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_8);
	GPIO_Init(GPIOB, &g);

	RCC_PCLK2Config(RCC_HCLK_Div1);

	c.COMP_InvertingInput = COMP_InvertingInput_VREFINT;
	c.COMP_NonInvertingInput = COMP_NonInvertingInput_IO2;
	c.COMP_Mode = COMP_Mode_MediumSpeed;
	c.COMP_Output = COMP_Output_None;
	c.COMP_OutputPol = COMP_OutputPol_NonInverted;
	c.COMP_Hysteresis = COMP_Hysteresis_Low;
	c.COMP_BlankingSrce = COMP_BlankingSrce_None;

	COMP_Init(COMP_Selection_COMP4, &c);
	COMP_Cmd(COMP_Selection_COMP4, ENABLE);

	//COMP7 Init:

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	g.GPIO_Mode = GPIO_Mode_AN;
	g.GPIO_PuPd = GPIO_PuPd_NOPULL;
	g.GPIO_Pin = GPIO_Pin_1; 	//PC1 = INPUT

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
	GPIO_Init(GPIOC, &g);

	g.GPIO_Mode = GPIO_Mode_AF;
	g.GPIO_OType = GPIO_OType_PP;
	g.GPIO_PuPd = GPIO_PuPd_NOPULL;
	g.GPIO_Pin = GPIO_Pin_2; 	//PC2 = OUTPUT

	GPIO_PinAFConfig(GPIOC, GPIO_PinSource2, GPIO_AF_3);////////!!!!!!!!!!
	GPIO_Init(GPIOC, &g);

	RCC_PCLK2Config(RCC_HCLK_Div1);

	c.COMP_InvertingInput = COMP_InvertingInput_VREFINT;
	c.COMP_NonInvertingInput = COMP_NonInvertingInput_IO2;
	c.COMP_Mode = COMP_Mode_MediumSpeed;
	c.COMP_Output = COMP_Output_None;
	c.COMP_OutputPol = COMP_OutputPol_NonInverted;
	c.COMP_Hysteresis = COMP_Hysteresis_Low;
	c.COMP_BlankingSrce = COMP_BlankingSrce_None;

	COMP_Init(COMP_Selection_COMP7, &c);
	COMP_Cmd(COMP_Selection_COMP7, ENABLE);
}

int16_t calc_gyro_bias(void)
{
	int i = 0;
	int accum_x = 0;

	union twosComp {		// Takes care of two's complement conversion
		uint16_t un_signed;
		int16_t output;
	} convert;

	uint8_t bytes[2];

	for(i = 0; i < 250; ++i)
	{
		L3GD20_Read(bytes, L3GD20_OUT_X_L_ADDR, 2);
		convert.un_signed = (bytes[1] << 8) | bytes[0];
		accum_x += convert.output;
	}
	return (int16_t)((float)accum_x/(float)250);
}

/*
 * Initializes the DMA controller to transfer data between the ADC (ADC1) and
 * a user-supplied location in memory
 * @Args: None
 * @Return Val: None
 */

void adc1_init_DMA(void)
{
	// Init structures required to initialize the peripherals as required:

	ADC_InitTypeDef       ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	GPIO_InitTypeDef      GPIO_InitStructure;

	/* Configure the ADC clock */
	RCC_ADCCLKConfig(RCC_ADC12PLLCLK_Div2);
	/* Enable ADC1 clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ADC12, ENABLE);
	/* GPIOC Periph clock enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

	// Configure ADC Channel 3 and 4 as analog inputs

	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	ADC_StructInit(&ADC_InitStructure);

	/* Calibration procedure */
	ADC_VoltageRegulatorCmd(ADC1, ENABLE);

	/* Insert delay equal to about 900 µs */
	int foo;
	for(foo = 0; foo < 64000; ++foo)
	{
	  ++foo;
	}

	ADC_SelectCalibrationMode(ADC1, ADC_CalibrationMode_Single);
	ADC_StartCalibration(ADC1);

	while(ADC_GetCalibrationStatus(ADC1) != RESET );

	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_Clock = ADC_Clock_AsynClkMode; //ADC_Clock_SynClkModeDiv2
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_1;
	ADC_CommonInitStructure.ADC_DMAMode = ADC_DMAMode_Circular;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = 0xF;//0xF;
	ADC_CommonInit(ADC1, &ADC_CommonInitStructure);

	ADC_InitStructure.ADC_ContinuousConvMode = ADC_ContinuousConvMode_Enable;
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ExternalTrigConvEvent = ADC_ExternalTrigConvEvent_0;
	ADC_InitStructure.ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_OverrunMode = ADC_OverrunMode_Disable;
	ADC_InitStructure.ADC_AutoInjMode = ADC_AutoInjec_Disable;
	ADC_InitStructure.ADC_NbrOfRegChannel = 2;
	ADC_Init(ADC1, &ADC_InitStructure);

	/* ADC1 regular channel3 configuration */
	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 1, ADC_SampleTime_601Cycles5);//ADC_SampleTime_61Cycles5);//ADC_SampleTime_7Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 2, ADC_SampleTime_601Cycles5);//ADC_SampleTime_61Cycles5);//ADC_SampleTime_7Cycles5);

	/* Enable ADC1 */
	ADC_Cmd(ADC1, ENABLE);

	/* wait for ADRDY */
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_RDY));

	/* Start ADC1 Software Conversion */
	ADC_StartConversion(ADC1);

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	DMA_InitTypeDef			DMA_InitStructure;
	DMA_StructInit(&DMA_InitStructure);

	DMA_DeInit(DMA1_Channel1); 											//Set DMA registers to default values
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(ADC1->DR);
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) &adcData; 		//Variable to which ADC values will be stored
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = 2;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

	DMA_Init(DMA1_Channel1, &DMA_InitStructure);

	NVIC_InitTypeDef nv;

	nv.NVIC_IRQChannel = DMA1_Channel1_IRQn;
	nv.NVIC_IRQChannelPreemptionPriority = 2;
	nv.NVIC_IRQChannelSubPriority = 0;
	nv.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init(&nv);

	DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);
	DMA_Cmd(DMA1_Channel1, ENABLE);

	ADC_DMAConfig(ADC1, ADC_DMAMode_Circular);
	ADC_DMACmd(ADC1, ENABLE);
}

void adc2_init_DMA(void)
{
	// Init structures required to initialize the peripherals as required:

	ADC_InitTypeDef       ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	GPIO_InitTypeDef      GPIO_InitStructure;

	/* Configure the ADC clock */
	RCC_ADCCLKConfig(RCC_ADC12PLLCLK_Div2); /////////////MOD!!!!!!!!
	/* Enable ADC2 clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ADC12, ENABLE);
	/* GPIOA, GPIOB, GPIOC Periph clock enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOC, ENABLE);

	// Configure PA4, PB2, PC4, PC5 as analog inputs

	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	ADC_StructInit(&ADC_InitStructure);

	/* Calibration procedure */
	ADC_VoltageRegulatorCmd(ADC2, ENABLE);

	/* Insert delay equal to about 900 µs */
	int foo;
	for(foo = 0; foo < 64000; ++foo)
	{
	  ++foo;
	}

	ADC_SelectCalibrationMode(ADC2, ADC_CalibrationMode_Single);
	ADC_StartCalibration(ADC2);

	while(ADC_GetCalibrationStatus(ADC2) != RESET );

	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_Clock = ADC_Clock_AsynClkMode;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_1;
	ADC_CommonInitStructure.ADC_DMAMode = ADC_DMAMode_Circular;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = 0xF;
	ADC_CommonInit(ADC2, &ADC_CommonInitStructure);

	ADC_InitStructure.ADC_ContinuousConvMode = ADC_ContinuousConvMode_Enable;
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ExternalTrigConvEvent = ADC_ExternalTrigConvEvent_0;
	ADC_InitStructure.ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_OverrunMode = ADC_OverrunMode_Disable;
	ADC_InitStructure.ADC_AutoInjMode = ADC_AutoInjec_Disable;
	ADC_InitStructure.ADC_NbrOfRegChannel = 4;
	ADC_Init(ADC2, &ADC_InitStructure);

	/* ADC2 regular Channel 1,2,3,4 configuration */

	ADC_RegularChannelConfig(ADC2, ADC_Channel_1, 1, ADC_SampleTime_601Cycles5);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_5, 2, ADC_SampleTime_601Cycles5);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_11, 3, ADC_SampleTime_601Cycles5);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_12, 4, ADC_SampleTime_601Cycles5);

	/* Enable ADC2 */
	ADC_Cmd(ADC2, ENABLE);

	/* wait for ADRDY */
	while(!ADC_GetFlagStatus(ADC2, ADC_FLAG_RDY));

	/* Start ADC2 Software Conversion */
	ADC_StartConversion(ADC2);

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);

	DMA_InitTypeDef			DMA_InitStructure;
	DMA_StructInit(&DMA_InitStructure);

	DMA_DeInit(DMA2_Channel1); 											//Set DMA registers to default values
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(ADC2->DR);
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) &adc2_data; 		//Variable to which ADC values will be stored
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = 4;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

	DMA_Init(DMA2_Channel1, &DMA_InitStructure);

	NVIC_InitTypeDef nv;

	nv.NVIC_IRQChannel = DMA2_Channel1_IRQn;
	nv.NVIC_IRQChannelPreemptionPriority = 2;
	nv.NVIC_IRQChannelSubPriority = 0;
	nv.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init(&nv);

	DMA_ITConfig(DMA2_Channel1, DMA_IT_TC, ENABLE);
	DMA_Cmd(DMA2_Channel1, ENABLE);

	ADC_DMAConfig(ADC2, ADC_DMAMode_Circular);
	ADC_DMACmd(ADC2, ENABLE);
}

void battery_watchdog_init(void)
{
	ADC_InitTypeDef       ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	GPIO_InitTypeDef      GPIO_InitStructure;
	/* Configure the ADC clock */
	RCC_ADCCLKConfig(RCC_ADC34PLLCLK_Div128);

	/* Enable ADC1 clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ADC34, ENABLE);
	/* ADC Channel configuration */
	/* GPIOC Periph clock enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOE, ENABLE);

	/* Configure PB1, PE7 as analog inputs */
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
/*
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
*/

	ADC_StructInit(&ADC_InitStructure);

	/* Calibration procedure */
	ADC_VoltageRegulatorCmd(ADC3, ENABLE);

	/* Insert delay equal to 222 µs */
	int foo;
	for(foo = 0; foo < 32000; ++foo)
	{
	  ++foo;
	}

	ADC_SelectCalibrationMode(ADC3, ADC_CalibrationMode_Single);
	ADC_StartCalibration(ADC3);

	while(ADC_GetCalibrationStatus(ADC3) != RESET );

	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_Clock = ADC_Clock_SynClkModeDiv4;//ADC_Clock_AsynClkMode;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStructure.ADC_DMAMode = ADC_DMAMode_OneShot;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = 0;
	ADC_CommonInit(ADC3, &ADC_CommonInitStructure);

	ADC_InitStructure.ADC_ContinuousConvMode = ADC_ContinuousConvMode_Enable;
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ExternalTrigConvEvent = ADC_ExternalTrigConvEvent_0;
	ADC_InitStructure.ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_OverrunMode = ADC_OverrunMode_Disable;
	ADC_InitStructure.ADC_AutoInjMode = ADC_AutoInjec_Disable;
	ADC_InitStructure.ADC_NbrOfRegChannel = 2;
	ADC_Init(ADC3, &ADC_InitStructure);

	/* ADC3 regular Channel 1, 13 configuration */
	ADC_RegularChannelConfig(ADC3, ADC_Channel_1, 1, ADC_SampleTime_601Cycles5);
	ADC_RegularChannelConfig(ADC3, ADC_Channel_5, 2, ADC_SampleTime_601Cycles5);

	ADC_AnalogWatchdog1SingleChannelConfig(ADC3, ADC_Channel_1);
	ADC_AnalogWatchdog2SingleChannelConfig(ADC3, ADC_Channel_5);

	/* Configure AWD 1 & 2 Thresholds: */

	ADC_AnalogWatchdog1ThresholdsConfig(ADC3, 2048, 10);// 2100, 1996); //50%
	ADC_AnalogWatchdog2ThresholdsConfig(ADC3, 0x80, 0x00);//0xC1, 0xBB); //75%

	ADC_AnalogWatchdogCmd(ADC3, ADC_AnalogWatchdog_SingleRegEnable);
//	ADC_AnalogWatchdogCmd(ADC3, ADC_AnalogWatchdog_AllRegEnable);

	// Configure ADC3 global interrupt:

	NVIC_InitTypeDef nv;

	nv.NVIC_IRQChannel = ADC3_IRQn;
	nv.NVIC_IRQChannelPreemptionPriority = 0;
	nv.NVIC_IRQChannelSubPriority = 0;
	nv.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init(&nv);

	// Enable ADC3's global interrupt:

	ADC_ITConfig(ADC3, ADC_IT_AWD1 | ADC_IT_AWD2, ENABLE);

	/* Enable ADC3 */
	ADC_Cmd(ADC3, ENABLE);

	/* wait for ADRDY */
	while(!ADC_GetFlagStatus(ADC3, ADC_FLAG_RDY));

	/* Start ADC3 Software Conversion */
	ADC_StartConversion(ADC3);
}

/* Initializes ADC1 to read PA2 continuously using ADC1 end of conversion (EOC) interrupt
 * @Args: Nothing
 * @Return Val: Nothing
 */

void adc1_init(void) //PA2 -> Channel 3 on ADC1
{
	ADC_InitTypeDef       ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	GPIO_InitTypeDef      GPIO_InitStructure;
	/* Configure the ADC clock */
	RCC_ADCCLKConfig(RCC_ADC12PLLCLK_Div2);

	/* Enable ADC1 clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ADC12, ENABLE);
	/* ADC Channel configuration */
	/* GPIOC Periph clock enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

	/* Configure ADC Channel7 as analog input */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	ADC_StructInit(&ADC_InitStructure);

	/* Calibration procedure */
	ADC_VoltageRegulatorCmd(ADC1, ENABLE);

	/* Insert delay equal to 10 µs */
	int foo;
	for(foo = 0; foo < 32000; ++foo)
	{
	  ++foo;
	}

	ADC_SelectCalibrationMode(ADC1, ADC_CalibrationMode_Single);
	ADC_StartCalibration(ADC1);

	while(ADC_GetCalibrationStatus(ADC1) != RESET );

	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_Clock = ADC_Clock_AsynClkMode;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStructure.ADC_DMAMode = ADC_DMAMode_OneShot;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = 0;
	ADC_CommonInit(ADC1, &ADC_CommonInitStructure);

	ADC_InitStructure.ADC_ContinuousConvMode = ADC_ContinuousConvMode_Enable;
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ExternalTrigConvEvent = ADC_ExternalTrigConvEvent_0;
	ADC_InitStructure.ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_OverrunMode = ADC_OverrunMode_Disable;
	ADC_InitStructure.ADC_AutoInjMode = ADC_AutoInjec_Disable;
	ADC_InitStructure.ADC_NbrOfRegChannel = 1;
	ADC_Init(ADC1, &ADC_InitStructure);

	/* ADC1 regular channel3 configuration */
	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 1, ADC_SampleTime_601Cycles5);//ADC_SampleTime_61Cycles5);//ADC_SampleTime_7Cycles5);

	// Configure ADC1_2 global interrupt:

	NVIC_InitTypeDef nv;

	nv.NVIC_IRQChannel = ADC1_2_IRQn;
	nv.NVIC_IRQChannelPreemptionPriority = 2;
	nv.NVIC_IRQChannelSubPriority = 0;
	nv.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init(&nv);

	// Enable ADC1's global interrupt:

	ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);

	/* Enable ADC1 */
	ADC_Cmd(ADC1, ENABLE);

	/* wait for ADRDY */
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_RDY));

	/* Start ADC1 Software Conversion */
	ADC_StartConversion(ADC1);
}
