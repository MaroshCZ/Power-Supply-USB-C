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
void button_isr(void){}

/*
 * Timer interrupt routine
 */
void button_timer_isr(void){}
