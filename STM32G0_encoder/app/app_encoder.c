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
void button_timer_isr(void){}
