/*
 * app_encoder.c
 *
 *  Created on: Mar 26, 2024
 *      Author: Jan Mareš
 */

#include "main.h"
#include "app_encoder.h"

//Variables declaration
int encoderVal = 0;
int encoderPress = 0;
int g = 0;

/*
 * Initialization function
 */
void app_encoder_init(void){
	//HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

	//TIM7 initialization
	LL_TIM_EnableIT_UPDATE(TIM7); //Enable interrupt generation when timer goes to max value and UPDATE event flag is set
	LL_TIM_ClearFlag_UPDATE(TIM7); //Clear update flag on TIMER6
}

/*
 * Loop function
 */
void app_encoder_loop(void){
	encoderVal = (TIM2 -> CNT) >> 2;
	if (g < 1000) {
		g++;
	}
	HAL_Delay(100);
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

	if (encoderPress < 4){
		encoderPress++;
	}
	else {
		encoderPress = 0;
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
