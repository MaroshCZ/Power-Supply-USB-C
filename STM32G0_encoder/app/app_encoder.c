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
int voltage = 254;
int voltageTemp = 0;
int val;
int voltageMin = 0;
int voltageMax = 2200;
int g = 0;
int integer_part;
int num_digits;
uint32_t arr;
uint32_t cnt;

/*
 * Initialization function
 */
void app_encoder_init(void){
	//HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

	//TIM2 initialization
	LL_TIM_EnableIT_UPDATE(TIM2); //Enable interrupt generation when timer goes to max value and UPDATE event flag is set
	LL_TIM_ClearFlag_UPDATE(TIM2); //Clear update flag on TIMER7
	LL_TIM_SetCounter(TIM2, READ_REG(TIM2->ARR)/2); //Set TIM2 counter to begin in middle of range

	//TIM7 initialization
	LL_TIM_EnableIT_UPDATE(TIM7); //Enable interrupt generation when timer goes to max value and UPDATE event flag is set
	LL_TIM_ClearFlag_UPDATE(TIM7); //Clear update flag on TIMER7

	//TIM4 initialization
	LL_TIM_EnableIT_UPDATE(TIM4); //Enable interrupt generation when timer goes to max value and UPDATE event flag is set
	LL_TIM_ClearFlag_UPDATE(TIM4); //Clear update flag on TIMER4

	//Init 7 segment display
	max7219_Init( 7 );
	max7219_Decode_On();

	arr = READ_REG(TIM4->ARR);


}

/*
 * Loop function
 */
void app_encoder_loop(void){

	if (g < 1000) {
					g++;
				}

	encoderVal = (TIM2 -> CNT) >> 2;

			if (encoderVal != encoderValPrev){
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
				}

			if (g < 1000) {
				g++;
			}

			// Get number of int numbers of voltage
			integer_part = (int)voltage;
			num_digits = 0;

			while (integer_part) {
				integer_part = integer_part/10;
				num_digits++;
			}

			//Print the voltage to the display
			max7219_PrintItos(num_digits, voltage, 3);

			//Blink currently selected digit
			max7219_BlinkDigit(voltage, encoderPress);
			encoderValPrev = encoderVal;
}


/**
 * Button interrupt service routine
 */
void button_isr(void){
	//Mask unwanted button interrupts caused by debouncing
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


