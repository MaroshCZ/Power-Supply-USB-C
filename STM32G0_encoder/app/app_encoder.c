/*
 * app_encoder.c
 *
 *  Created on: Mar 26, 2024
 *      Author: Jan Mareš
 */

#include "main.h"
#include "app_encoder.h"
#include "max7219.h"
#include "math.h"

//Variables declaration
int encoderVal = 0;
int encoderValPrev = 0;
int encoderPress = 4;
double voltage = 00.00;
double voltageTemp = 00.00;
double val;
double voltageMin = 0;
double voltageMax = 22;
int g = 0;
int integer_part;
int num_digits;
float result1;
float result2;
float result3;

/*
 * Initialization function
 */
void app_encoder_init(void){
	//HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

	//TIM7 initialization
	LL_TIM_EnableIT_UPDATE(TIM7); //Enable interrupt generation when timer goes to max value and UPDATE event flag is set
	LL_TIM_ClearFlag_UPDATE(TIM7); //Clear update flag on TIMER6

	//Init 7 segment display
	max7219_Init( 7 );
	max7219_Decode_On();
}

/*
 * Loop function
 */
void app_encoder_loop(void){
	encoderVal = (TIM2 -> CNT) >> 2;

	if (encoderVal != encoderValPrev){
			switch (encoderPress) {
				case 1:
					val = 0.01;
					break;
				case 2:
					val = 0.1;
					break;
				case 3:
					val = 1;
					break;
				case 4:
					val = 10;
					break;
			}

			//Get direction of encoder turning
			if (encoderVal > encoderValPrev) {
				voltageTemp += val;
			} else {
				voltageTemp -= val;
			}

			//If required temp value within limits, assign it to voltage
			if (voltageMin <= voltageTemp && voltageTemp <= voltageMax) {
				voltage = ceilf(100*voltageTemp)/100;
			} else {
				voltageTemp = voltage;
			}
		}

	if (g < 1000) {
		g++;
	}

	encoderValPrev = encoderVal;
}

/**
 * Button interrupt service routine
 */
void button_isr(void){
	//Mask unwanted button interrupts caused by debouncing
	EXTI->IMR1 &= ~(EXTI_IMR1_IM3);

	//Zero TIM6 counter and start counting
	LL_TIM_SetCounter(TIM7, 0); //set counter register value of timer 7 to 0
	LL_TIM_EnableCounter(TIM7); //start counting of timer 7

	if (encoderPress > 1){
		encoderPress--;
	}
	else {
		encoderPress = 4;
	}

	//Erase btn (PC3) interrupt flag
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_3);
}

/*
 * Timer interrupt routine
 */
void button_timer_isr(void){
	EXTI->IMR1 |= EXTI_IMR1_IM3; //unmask interrupt mask register on exti line 3
	LL_TIM_ClearFlag_UPDATE(TIM7); //Clear update flag on TIMER7
}
