/*
 * app.c
 *
 *  Created on: Apr 29, 2024
 *      Author: Jan Mareš
 */


#include "main.h"
#include "app.h"
#include "max7219.h"

//Variables declaration
int encoderVal; //TIM2 CNT register reading
int encoderValPrev;
int encoderPress = 4; //currently selected digit
int voltage = 254; //final voltage value
int voltageTemp = 0; //temporary voltage value
int val; //variable holding current voltage addition
int voltageMin = 0; //voltage down limit
int voltageMax = 2200; //voltage upper limit

int g = 0;

/*
 * Initialization function
 */
void app_init(void){

	g = 5;

	//TIM7 initialization
	LL_TIM_EnableIT_UPDATE(TIM7); //Enable interrupt generation when timer goes to max value and UPDATE event flag is set
	LL_TIM_ClearFlag_UPDATE(TIM7); //Clear update flag on TIMER7

}

/*
 * Loop function
 */
void app_loop(void){

	if (g < 1000) {
					g++;
				}

}

/**
 * TIM2 encoder turning interrupt service routine
 */
void encoder_turn_isr(void) {
	encoderVal = (TIM2 -> CNT) >> 2;

	if (encoderVal != encoderValPrev){

		//Get direction of encoder turning
		if (encoderVal > encoderValPrev) {
			voltageTemp += val;
		} else {
			voltageTemp -= val;
		}

		//If required temp value within limits, assign it to voltage
		if (voltageMin <= voltageTemp && voltageTemp <= voltageMax) {
			voltage = voltageTemp;
		} else {
			voltageTemp = voltage;
		}

		encoderValPrev = encoderVal;
	}
}


/**
 * Button interrupt service routine
 */
void button_isr(void){
	//Mask unwanted button interrupts caused by debouncing on exti line 3 (PC3)
	EXTI->IMR1 &= ~(EXTI_IMR1_IM3);

	//Zero TIM7 counter and start counting
	LL_TIM_SetCounter(TIM7, 0); //set counter register value of timer 7 to 0
	LL_TIM_EnableCounter(TIM7); //start counting of timer 7

	if (encoderPress > 1){
		encoderPress--;
	}
	else {
		encoderPress = 4;
	}

	switch (encoderPress) {
				case 1:
					val = 1;
					break;
				case 2:
					val = 10;
					break;
				case 3:
					val = 100;
					break;
				case 4:
					val = 1000;
					break;
			}

	//Erase btn (PC3) interrupt flag
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_3);
}

/*
 * Timer interrupt routine
 */
void button_timer_isr(void){
	EXTI->IMR1 |= EXTI_IMR1_IM3; //unmask interrupt mask register on exti line 3 (PC3)
	LL_TIM_ClearFlag_UPDATE(TIM7); //Clear update flag on TIMER7
}

