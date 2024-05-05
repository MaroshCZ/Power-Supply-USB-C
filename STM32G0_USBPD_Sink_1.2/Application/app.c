/*
 * app.c
 *
 *  Created on: Apr 29, 2024
 *      Author: Jan Mareš
 */


#include "main.h"
#include "app.h"
#include "max7219.h"
#include "usbpd_def.h"
#include "usbpd_dpm_user.h"



//Variables declaration
int encoderVal; //TIM2 CNT register reading
int encoderValPrev;
int encoderPress = 4; //currently selected digit
int val = 1000; //variable holding current voltage addition
int voltage = 0; //final voltage value
int voltageTemp = 0; //temporary voltage value
int voltageMin = 0; //voltage down limit
int voltageMax = 2200; //voltage upper limit
int integer_part;
int num_digits;

int g = 0;

/*
 * Initialization function
 */
void app_init(void){

	g = 5;

	//TIM7 initialization
	LL_TIM_EnableIT_UPDATE(TIM7); //Enable interrupt generation when timer goes to max value and UPDATE event flag is set
	LL_TIM_ClearFlag_UPDATE(TIM7); //Clear update flag on TIMER7

	//Init 7 segment display
	max7219_Init( 7 );
	max7219_Decode_On();
}

/*
 * Loop function
 */
void app_loop(void){
	//Blink currently selected digit
	max7219_BlinkDigit(&voltage, encoderPress, 500); //pass voltage address to BlinkDigit function
}


/**
 * TIM2 encoder turning interrupt service routine
 */
void encoder_turn_isr(void) {
	//Get the TIM2 value from CNT register
	encoderVal = (TIM2 -> CNT) >> 2;

	if (encoderVal != encoderValPrev){

		//Get direction of encoder turning
		if (encoderVal > encoderValPrev) {
			voltageTemp += val;
		} else {
			voltageTemp -= val;
		}

		//If required temp value is within limits, assign it to voltage
		if (voltageMin <= voltageTemp && voltageTemp <= voltageMax) {
			voltage = voltageTemp;
		} else {
			voltageTemp = voltage;
		}


		// Get number of int numbers in voltage var
		integer_part = (int)voltage;
		num_digits = 0;

		while (integer_part) {
			integer_part = integer_part/10;
			num_digits++;
		}

		//Print the voltage to the display, set decimal point at position 3 (display 1 has positions 4-1)
		max7219_PrintItos(num_digits, voltage, 3);

		//Save TIM2 CNT value to ValPrev
		encoderValPrev = encoderVal;
	}
}


/**
 * Button interrupt service routine
 */
void button_isr(void){
	//Mask unwanted button interrupts caused by debouncing on exti line 3 (PC3)
	EXTI->IMR1 &= ~(EXTI_IMR1_IM3);

	//Set debouncing time in ms
	TIM7->ARR = 200;

	//Zero TIM7 counter and start counting
	LL_TIM_SetCounter(TIM7, 0); //set counter register value of timer 7 to 0
	LL_TIM_EnableCounter(TIM7); //start counting of timer 7

	//Decrement encoderPress value if higher than 4
	if (encoderPress > 1){
		encoderPress--;
	}
	else {
		encoderPress = 4;
	}

	//Choose addition value based on encoderPress val,
	switch (encoderPress) {
		case 1:
			val = 5;
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
	//Unmask exti line 2 and 3
	EXTI->IMR1 |= EXTI_IMR1_IM3; //unmask interrupt mask register on exti line 3 (PC3)
	EXTI->IMR1 |= EXTI_IMR1_IM2; //unmask interrupt mask register on exti line 2 (PC2)

	//Clear update flag on TIM7
	LL_TIM_ClearFlag_UPDATE(TIM7); //Clear update flag on TIMER7
}

/*
 * Request button interrupt routine
 */
void request_button_isr(void){
	//Mask unwanted button interrupts caused by debouncing on exti line 2 (PC2)
	EXTI->IMR1 &= ~(EXTI_IMR1_IM2);

	//Zero TIM7 counter and start counting
	LL_TIM_SetCounter(TIM7, 0); //set counter register value of timer 7 to 0
	LL_TIM_EnableCounter(TIM7); //start counting of timer 7

	g += 1;

	USBPD_DPM_RequestMessageRequest(0, 7, voltage*10);
}
