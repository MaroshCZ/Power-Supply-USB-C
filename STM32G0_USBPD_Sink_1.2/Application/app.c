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
#include "usbpd_user_services.h"
#include "string.h"
#include "stdio.h"



//Variables declaration
int encoderVal; //TIM2 CNT register reading
int encoderValPrev;
int encoderPress = 4; //currently selected digit
int val = 1000; //variable holding current voltage addition
int voltage = 330; //final voltage value
int voltageTemp = 0; //temporary voltage value
int voltageMin = 0; //voltage down limit
int voltageMax = 2200; //voltage upper limit
int current = 1423;
int currentTemp = 0;
int currentMin = 0;
int currentMax = 3000;
int integer_part;
int num_digits;
int indexAPDO;
int indexSRCAPDO;
uint8_t isMinVoltageAPDOInitialized = 0; // Flag to indicate if minvoltageAPDO has been initialized
uint32_t maxvoltageAPDO;
uint32_t minvoltageAPDO;
uint32_t srcPdoIndex; //variable that holds Pdo index from FindVoltageIndex
USBPD_DPM_SNKPowerRequestDetailsTypeDef powerRequestDetails;
USBPD_StatusTypeDef powerProfiles;

int g = 0;

typedef enum {
	ADJUSTMENT_CURRENT = 0x0u, /*!< Current adjustment state */
	ADJUSTMENT_VOLTAGE = 0x1u /*!< Voltage adjustment state */
} AdjustmentState;

// Define a typedef for the state variable
typedef AdjustmentState Adjustment_StateTypedef;

// Declare a variable to hold the current state
volatile Adjustment_StateTypedef currentState = ADJUSTMENT_VOLTAGE;

/*
 * Initialization function
 */
void app_init(void){

	g = 5;

	//TIM7 initialization
	LL_TIM_EnableIT_UPDATE(TIM7); //Enable interrupt generation when timer goes to max value and UPDATE event flag is set
	LL_TIM_ClearFlag_UPDATE(TIM7); //Clear update flag on TIMER7

	//Init 7 segment display
	//max7219_Init( 7 );
	//max7219_Decode_On();

	//Print decimal points and initial values
	max7219_PrintItos(8, current, 8);
	max7219_PrintItos(4, voltage, 3);


}

/*
 * Loop function
 */
void app_loop(void){

	//Blink digit based on specified adjustment
	switch(currentState)
	{
	case(ADJUSTMENT_VOLTAGE):
	{
		//Blink currently selected digit
		max7219_BlinkDigit(&voltage, encoderPress, 500); //pass voltage address to BlinkDigit function
	}
	 break;
	case(ADJUSTMENT_CURRENT):
	{
		//Blink currently selected digit
		max7219_BlinkDigit(&current, encoderPress+4, 500); //pass voltage address to BlinkDigit function
	}
	break;
	}
}


/**
 * TIM2 encoder turning interrupt service routine
 */
void encoder_turn_isr(void) {
	//Get the TIM2 value from CNT register
	encoderVal = (TIM2 -> CNT) >> 2;

	if (encoderVal != encoderValPrev){

		switch(currentState)
		{
		case ADJUSTMENT_VOLTAGE:
		{
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

			//Print the voltage to the display, set decimal point after digit position 3 (display 1 has positions 4-1)
			max7219_PrintItos(num_digits, voltage, 3);

			//Save TIM2 CNT value to ValPrev
			encoderValPrev = encoderVal;
		}
		break;

		case ADJUSTMENT_CURRENT:
		{
			//Get direction of encoder turning
			if (encoderVal > encoderValPrev) {
				currentTemp += val;
			} else {
				currentTemp -= val;
			}

			//If required temp value is within limits, assign it to voltage
			if (currentMin <= currentTemp && currentTemp <= currentMax) {
				current = currentTemp;
			} else {
				currentTemp = current;
			}

			// Get number of int numbers in voltage var
			integer_part = (int)current;
			num_digits = 0;

			while (integer_part) {
				integer_part = integer_part/10;
				num_digits++;
			}

			//Print the voltage to the display, set decimal point after digit position 3 (display 1 has positions 4-1)
			max7219_PrintItos(num_digits+4, current, 8);

			//Save TIM2 CNT value to ValPrev
			encoderValPrev = encoderVal;

		}
		break;

		}
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

	//Choose addition value based on encoderPress val and current ADJUSTMENT_STATE (voltage/current)
	switch (currentState){
		case ADJUSTMENT_VOLTAGE:
			switch (encoderPress) {
			case 1:
				val = 2;
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
		 break;
		case ADJUSTMENT_CURRENT:
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
		 break;
	}


	//Erase btn (PC3) interrupt flag
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_3);
}

/*
 * Timer interrupt routine
 */
void button_timer_isr(void){
	//Unmask exti line 1, 2 and 3
	EXTI->IMR1 |= EXTI_IMR1_IM3; //unmask interrupt mask register on exti line 3 (PC3)
	EXTI->IMR1 |= EXTI_IMR1_IM2; //unmask interrupt mask register on exti line 2 (PC2)
	EXTI->IMR1 |= EXTI_IMR1_IM1; //unmask interrupt mask register on exti line 1 (PC1)

	//Clear update flag on TIM7
	LL_TIM_ClearFlag_UPDATE(TIM7); //Clear update flag on TIMER7
}

/*
 * Request button interrupt routine, request APDO with user voltage and current
 */
void request_button_isr(void){
	//Mask unwanted button interrupts caused by debouncing on exti line 2 (PC2)
	EXTI->IMR1 &= ~(EXTI_IMR1_IM2);

	//Zero TIM7 counter and start counting
	LL_TIM_SetCounter(TIM7, 0); //set counter register value of timer 7 to 0
	LL_TIM_EnableCounter(TIM7); //start counting of timer 7

	g += 1;

	/*
	typedef struct
	{
	  uint32_t RequestedVoltageInmVunits;               //< Sink request operating voltage in mV units
	  uint32_t MaxOperatingCurrentInmAunits;            //< Sink request Max operating current in mA units
	  uint32_t OperatingCurrentInmAunits;               //< Sink request operating current in mA units
	  uint32_t MaxOperatingPowerInmWunits;              //< Sink request Max operating power in mW units
	  uint32_t OperatingPowerInmWunits;                 //< Sink request operating power in mW units
	} USBPD_DPM_SNKPowerRequestDetailsTypeDef;
	#endif */

	sourcecapa_limits();

	indexSRCAPDO = USER_SERV_FindSRCIndex(0, &powerRequestDetails, voltage*10, current, PDO_SEL_METHOD_MAX_CUR);
	USBPD_DPM_RequestSRCPDO(0, indexSRCAPDO, voltage*10, current);

}

/*
 * Change between current and voltage ADJUSTMENT_STATE
 */
void cur_vol_button_isr(void){
	//Mask unwanted button interrupts caused by debouncing on exti line 1 (PC1)
	EXTI->IMR1 &= ~(EXTI_IMR1_IM1);

	//Set debouncing time in ms
	TIM7->ARR = 200;

	//Zero TIM7 counter and start counting
	LL_TIM_SetCounter(TIM7, 0); //set counter register value of timer 7 to 0
	LL_TIM_EnableCounter(TIM7); //start counting of timer 7

	// Toggle the state
	if (currentState == ADJUSTMENT_CURRENT)
	{
		currentState = ADJUSTMENT_VOLTAGE;
	}
	else
	{
		currentState = ADJUSTMENT_CURRENT;
	}
	encoderPress = 3;
}

#define MAX_LINE_PDO      7u
/**
  * @brief  src capa menu navigation
  * @param  Nav
  * @retval None
  * source: demo_disco.c Display_sourcecapa_menu_nav
  */
static void sourcecapa_limits(void)
{
  uint8_t _str[30];
  uint8_t _max = DPM_Ports[0].DPM_NumberOfRcvSRCPDO;
  uint8_t _start, _end = 0;

  /*
  if (hmode == MODE_STANDALONE)
  {
	Menu_manage_selection(_max, MAX_LINE_PDO, &_start, &_end, Nav);
  }
  else
  {
	if((Nav == 1) && (_max > MAX_LINE_PDO))
	{
	  _start = _max - MAX_LINE_PDO;
	  _end = _max;
	}
	else
	{
	  _start = 0;
	  _end = MIN(_max, MAX_LINE_PDO);
	}
  }
  */
  _start = 0;
  _end = 6;

  for(int8_t index=_start; index < _max; index++)
  {
	switch(DPM_Ports[0].DPM_ListOfRcvSRCPDO[index] & USBPD_PDO_TYPE_Msk)
	{
	case USBPD_PDO_TYPE_FIXED :
	  {
		uint32_t maxcurrent = ((DPM_Ports[0].DPM_ListOfRcvSRCPDO[index] & USBPD_PDO_SRC_FIXED_MAX_CURRENT_Msk) >> USBPD_PDO_SRC_FIXED_MAX_CURRENT_Pos)*10;
		uint32_t maxvoltage = ((DPM_Ports[0].DPM_ListOfRcvSRCPDO[index] & USBPD_PDO_SRC_FIXED_VOLTAGE_Msk) >> USBPD_PDO_SRC_FIXED_VOLTAGE_Pos)*50;
		sprintf((char*)_str, "FIXED:%2dV %2d.%dA", (int)(maxvoltage/1000), (int)(maxcurrent/1000), (int)((maxcurrent % 1000) /100));
		break;
	  }
	case USBPD_PDO_TYPE_BATTERY :
	  {
		uint32_t maxvoltage = ((DPM_Ports[0].DPM_ListOfRcvSRCPDO[index] & USBPD_PDO_SRC_BATTERY_MAX_VOLTAGE_Msk) >> USBPD_PDO_SRC_BATTERY_MAX_VOLTAGE_Pos) * 50;
		uint32_t minvoltage = ((DPM_Ports[0].DPM_ListOfRcvSRCPDO[index] & USBPD_PDO_SRC_BATTERY_MIN_VOLTAGE_Msk) >> USBPD_PDO_SRC_BATTERY_MIN_VOLTAGE_Pos) * 50;
		uint32_t maxpower = ((DPM_Ports[0].DPM_ListOfRcvSRCPDO[index] & USBPD_PDO_SRC_BATTERY_MAX_POWER_Msk) >> USBPD_PDO_SRC_BATTERY_MAX_POWER_Pos) * 250;
		if ((maxpower)==100000) /* 100W */
		{
		  sprintf((char*)_str, "B:%2d.%1d-%2d.%1dV %2dW",(int)(minvoltage/1000),(int)(minvoltage/100)%10, (int)(maxvoltage/1000),(int)(maxvoltage/100)%10, (int)(maxpower/1000));
		}
		else
		{
		  sprintf((char*)_str, "B:%2d.%1d-%2d.%1dV %2d.%dW", (int)(minvoltage/1000),(int)(minvoltage/100)%10, (int)(maxvoltage/1000),(int)(maxvoltage/100)%10, (int)(maxpower/1000), (int)(maxpower/100)%10);
		}
	  }
	  break;
	case USBPD_PDO_TYPE_VARIABLE :
	  {
		uint32_t maxvoltage = ((DPM_Ports[0].DPM_ListOfRcvSRCPDO[index] & USBPD_PDO_SRC_VARIABLE_MAX_VOLTAGE_Msk) >> USBPD_PDO_SRC_VARIABLE_MAX_VOLTAGE_Pos) * 50;
		uint32_t minvoltage = ((DPM_Ports[0].DPM_ListOfRcvSRCPDO[index] & USBPD_PDO_SRC_VARIABLE_MIN_VOLTAGE_Msk) >> USBPD_PDO_SRC_VARIABLE_MIN_VOLTAGE_Pos) * 50;
		uint32_t maxcurrent = ((DPM_Ports[0].DPM_ListOfRcvSRCPDO[index] & USBPD_PDO_SRC_VARIABLE_MAX_CURRENT_Msk) >> USBPD_PDO_SRC_VARIABLE_MAX_CURRENT_Pos) * 10;
		sprintf((char*)_str, "V:%2d.%1d-%2d.%1dV %d.%dA", (int)(minvoltage/1000),(int)(minvoltage/100)%10, (int)(maxvoltage/1000),(int)(maxvoltage/100)%10, (int)(maxcurrent/1000), (int)((maxcurrent % 1000) /100));
	  }
	  break;
	case USBPD_PDO_TYPE_APDO :
	  {
		indexAPDO = index + 1;
		uint32_t minvoltageAPDOtemp = ((DPM_Ports[0].DPM_ListOfRcvSRCPDO[index] & USBPD_PDO_SRC_APDO_MIN_VOLTAGE_Msk) >> USBPD_PDO_SRC_APDO_MIN_VOLTAGE_Pos) * 100;
		uint32_t maxvoltageAPDOtemp = ((DPM_Ports[0].DPM_ListOfRcvSRCPDO[index] & USBPD_PDO_SRC_APDO_MAX_VOLTAGE_Msk) >> USBPD_PDO_SRC_APDO_MAX_VOLTAGE_Pos) * 100;
		uint32_t maxcurrentAPDOtemp = ((DPM_Ports[0].DPM_ListOfRcvSRCPDO[index] & USBPD_PDO_SRC_APDO_MAX_CURRENT_Msk) >> USBPD_PDO_SRC_APDO_MAX_CURRENT_Pos) * 50;
		sprintf((char*)_str, "A:%2d.%1d-%2d.%1dV %d.%dA",(int) (minvoltageAPDOtemp/1000),(int)(minvoltageAPDOtemp/100)%10, (int)(maxvoltageAPDOtemp/1000),(int)(maxvoltageAPDOtemp/100)%10, (int)(maxcurrentAPDOtemp/1000), (int)((maxcurrentAPDOtemp % 1000) /100));

		if (!isMinVoltageAPDOInitialized || minvoltageAPDOtemp < minvoltageAPDO) {
			minvoltageAPDO = minvoltageAPDOtemp;
			voltageMin = (int)minvoltageAPDOtemp/10;
			voltage = voltageMin/10;
			isMinVoltageAPDOInitialized = 1; // Set the flag to indicate it has been initialized
		}

		if (maxvoltageAPDOtemp > maxvoltageAPDO) {
			maxvoltageAPDO = maxvoltageAPDOtemp;
			voltageMax = (int)maxvoltageAPDOtemp/10;
		}
	  }
	  break;
	default :
	  sprintf((char*)_str,"Unknown Source PDO");
	  break;
	}
  }
}
