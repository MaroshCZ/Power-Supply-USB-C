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
#include "usbpd_def.h"
#include "string.h"
#include "stdio.h"
#include "stdbool.h"
#include <stm32g0xx_ll_adc.h>

#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <usbpd_trace.h>


typedef struct {
  //USBPD_PPSSDB_TypeDef  DPM_RcvPPSStatus;           /*!< PPS Status received by port partner                         */
  //USBPD_SKEDB_TypeDef   DPM_RcvSNKExtendedCapa;     /*!< SNK Extended Capability received by port partner            */
  uint32_t              voltageSet;       /*!< User selected voltage in centivolts */
  uint32_t              currentSet;       /*!< User selected OCP limit in mA */

  uint32_t              voltageMeas;      /*!< Measured output voltage in centivolts */
  uint32_t              currentMeas;      /*!< Measured output current in centivolts */

  uint32_t              voltageMin;       /*!< Minimal SRC voltage in centivolts */
  uint32_t              voltageMax;       /*!< Maximal SRC voltage in centivolts */
  uint32_t              currentMax;       /*!< Maximal SRC current in mA */

  int                   val;
  int 					encoderPress;     /*!< Currently selected digit */

} SINKData_HandleTypeDef;


//Variables declaration
int encoderVal; //TIM2 CNT register reading
int encoderValPrev;
int encoderPress = 2; //currently selected digit
int val = 10; //variable holding current voltage addition


int voltage = 330; //final voltage value
int voltageTemp = 330; //temporary voltage value
int voltageMin = 330; //voltage down limit
int voltageMax = 2100; //voltage upper limit
int current = 1000;
int currentOCP = 1000;
int cur; //output current
int vol; //output voltage
int V_TRIP;
int dac_value = 500;
int currentTemp = 1000;
int currentOCPTemp = 1000;
int currentMin = 0;
int currentMax = 3000;

int ocp_reset_needed = 0;
bool limitReadFlag = false;
uint8_t isMinVoltageAPDOInitialized = 0; // Flag to indicate if minvoltageAPDO has been initialized
uint32_t maxvoltageAPDO;
uint32_t minvoltageAPDO;


uint32_t srcPdoIndex; //variable that holds Pdo index from FindVoltageIndex
USBPD_DPM_SNKPowerRequestDetailsTypeDef powerRequestDetails;
USBPD_StatusTypeDef powerProfiles;

__IO uint16_t aADCxConvertedValues[ADC_NUM_OF_SAMPLES] = {0};

int g = 0;

typedef enum {
	ADJUSTMENT_CURRENT = 0x0u,      /*!< Current adjustment state */
	ADJUSTMENT_VOLTAGE = 0x1u,      /*!< Voltage adjustment state */
	ADJUSTMENT_CURRENT_OCP = 0x2u   /*!< CurrentOCP adjustment state */
} AdjustmentState;

typedef enum {
	OUTPUT_OFF_STATE = 0x0u,      /*!< Current adjustment state */
	OUTPUT_ON_STATE = 0x1u,      /*!< Voltage adjustment state */

} OutputState;


//USB communication
uint8_t *data = "Hello World from USB CDC\n";
uint8_t usb_buffer[64];

// Define a typedef for the state variable
typedef AdjustmentState Adjustment_StateTypedef;
typedef OutputState Output_StateTypedef;

// Declare a variable to hold the current state
volatile Adjustment_StateTypedef currentState = ADJUSTMENT_VOLTAGE;
volatile Output_StateTypedef outputState = OUTPUT_OFF_STATE;

/*Add variables for LUPART2*/
#define RX_BUFFER_SIZE 64
static uint8_t rxBuffer[RX_BUFFER_SIZE];
static uint32_t rxIndex = 0;
uint32_t counter = 0;


SINKData_HandleTypeDef SNK_data = {
	.voltageMin = 500,
};

SINKData_HandleTypeDef *dhandle = &SNK_data;


// Callback when ADC conversion is complete
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  HAL_GPIO_TogglePin(LED_USER_GPIO_Port, LED_USER_Pin);
}

// Callback when ADW CH3 goes out of range
HAL_ADCEx_LevelOutOfWindow3Callback(ADC_HandleTypeDef *hadc) {
	outputState = OUTPUT_OFF_STATE;
	//Disable output
	HAL_GPIO_WritePin(RELAY_ON_OFF_GPIO_Port, RELAY_ON_OFF_Pin, GPIO_PIN_RESET);

	//Print the voltage and current to the display, set decimal point after digit position 3 (display 1 has positions 4-1)
	max7219_PrintIspecial(SEGMENT_1, voltage, 3);
	max7219_PrintIspecial(SEGMENT_2, current, 4);

}


/*
 * Initialization function
 */
void app_init(void){

	g = 5;

	//TIM7 initialization
	LL_TIM_EnableIT_UPDATE(TIM7); //Enable interrupt generation when timer goes to max value and UPDATE event flag is set
	LL_TIM_ClearFlag_UPDATE(TIM7); //Clear update flag on TIMER7

	//TIM14 initialization
	LL_TIM_EnableIT_UPDATE(TIM14); //Enable interrupt generation when timer goes to max value and UPDATE event flag is set
	LL_TIM_ClearFlag_UPDATE(TIM14); //Clear update flag on TIMER14

	//TIM3 initialization of encoder
	HAL_TIM_Encoder_Start_IT(&htim3, TIM_CHANNEL_ALL);
	__HAL_TIM_SET_COUNTER(&htim3, 30000); //write non 0 value to avoid shift from 0 -> max value
	encoderVal = __HAL_TIM_GET_COUNTER(&htim3)/4;
	encoderValPrev = encoderVal;

	//TIM4 initialization
	HAL_TIM_Base_Start(&htim4);

	//Calibrate and start ADC sensing with DMA
	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&aADCxConvertedValues, ADC_NUM_OF_SAMPLES);
	Update_AWD_Thresholds(0,4095); //(low, high)

	//Init DAC
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac_value);

	//Init 7 segment display
	max7219_Init( 7 );
	max7219_Decode_On();

	//Print decimal points and initial values
	max7219_PrintIspecial(SEGMENT_2, current, 4);
	max7219_PrintIspecial(SEGMENT_1, voltage, 3);

	//Wait for hardware initialization and then turn DB to HIGH (according to TCPP01-M12 datasheet 6.5)
	HAL_Delay(2000);
	HAL_GPIO_WritePin(DB_OUT_GPIO_Port, DB_OUT_Pin, GPIO_PIN_SET);

	//HAL_GPIO_WritePin(OCP_RESET_GPIO_Port, OCP_RESET_Pin, GPIO_PIN_SET);


}



/**
 * @brief  Start I/V sense on both Type-C ports.
 * @retval 0 success else fail
 */
/*
static uint8_t PWR_StartVBusSensing(void)
{
  uint8_t ret = 0u;

  //Start ADCx conversions
  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)aADCxConvertedValues, 2u) != HAL_OK)
  {
    ret++;
  }

  return ret;
}
*/

/*
 * Loop function
 */
void app_loop(void){
	/*
	if (limitReadFlag == false) {
		sourcecapa_limits();
		limitReadFlag = true;
		voltageMin = maxvoltageAPDO/10;
		voltageMax = minvoltageAPDO/10;
		voltage = voltageMin;
	}*/


	//Blink digit based on specified adjustment
	/*
	switch(currentState)
	{
		case(ADJUSTMENT_VOLTAGE):
		{
			//Blink currently selected digit
			max7219_BlinkDigit(SEGMENT_1, &voltage, encoderPress, 500, 3); //pass voltage address to BlinkDigit function
		}
		 break;
		case(ADJUSTMENT_CURRENT):
		{
			//Blink currently selected digit
			max7219_BlinkDigit(SEGMENT_2, &current, encoderPress, 500, 4); //pass voltage address to BlinkDigit function
		}
		 break;
		case(ADJUSTMENT_CURRENT_OCP):
		{

			//Blink currently selected digit
			max7219_BlinkDigit(SEGMENT_2, &currentOCP, encoderPress, 500, 4); //pass voltage address to BlinkDigit function
		}
		break;
	}*/

	if (ocp_reset_needed == 1) {
		HAL_GPIO_WritePin(OCP_RESET_GPIO_Port, OCP_RESET_Pin, GPIO_PIN_RESET);
		HAL_Delay(4); //datasheet says 100ns minimum pull down time for resettin alert, but for me even 1ms was not enough
		HAL_GPIO_WritePin(OCP_RESET_GPIO_Port, OCP_RESET_Pin, GPIO_PIN_SET);
		ocp_reset_needed = 0;
	}


	switch(outputState)
		{
			case(OUTPUT_OFF_STATE):
			{
				switch(currentState)
					{
					case(ADJUSTMENT_VOLTAGE):
					{
						//Blink currently selected digit
						max7219_BlinkDigit(SEGMENT_1, &voltage, encoderPress, 500, 3); //pass voltage address to BlinkDigit function
					}
					 break;
					case(ADJUSTMENT_CURRENT):
					{
						//Blink currently selected digit
						max7219_BlinkDigit(SEGMENT_2, &current, encoderPress, 500, 4); //pass voltage address to BlinkDigit function
					}
					break;
					case(ADJUSTMENT_CURRENT_OCP):
					{

						//Blink currently selected digit
						max7219_BlinkDigit(SEGMENT_2, &currentOCP, encoderPress, 500, 4); //pass voltage address to BlinkDigit function
					}
					break;
					}

			}
			break;

			case(OUTPUT_ON_STATE):
			{
				uint32_t vol = BSP_PWR_VBUSGetVoltage(0)/10; //divide by 10 t oget centivolts since only 4 digit display..
				uint32_t cur = BSP_PWR_VBUSGetCurrent(0);

				dhandle ->currentMeas = cur;
				dhandle ->voltageMeas = vol;
				//Display output voltage
				max7219_PrintIspecial(SEGMENT_1, vol, 3);

				//Display output current
				max7219_PrintIspecial(SEGMENT_2, cur, 4);

				HAL_Delay(500);

			}
			break;
		}

	//CDC_Transmit_FS(data, strlen(data));

}

/**
 * Update ADC CH3 AWD Treshold
 */
void Update_AWD_Thresholds(uint32_t low, uint32_t high) {
    HAL_ADC_Stop(&hadc1);  // Stop ADC before updating thresholds
    HAL_ADC_AnalogWDGConfig(&hadc1, &(ADC_AnalogWDGConfTypeDef){
        .WatchdogMode = ADC_ANALOGWATCHDOG_SINGLE_REG,
        .Channel = ADC_CHANNEL_3, //isense (MCP06C20)
        .ITMode = ENABLE,
        .HighThreshold = high,
        .LowThreshold = low
    });
    HAL_ADC_Start(&hadc1);  // Restart ADC
}


/**
 * TIM2 encoder turning interrupt service routine
 */
void encoder_turn_isr(void) {
	//Get the TIM2 value from CNT register
	encoderVal = (TIM3 -> CNT) >> 2;

	if (encoderVal != encoderValPrev){

		switch(currentState)
		{
		case ADJUSTMENT_VOLTAGE:
		{
			//Get direction of encoder turning
			if (encoderVal < encoderValPrev) {
				voltageTemp += val;
			} else {
				voltageTemp -= val;
			}

			//If required temp value is within limits, assign it to voltage
			if (voltageMin <= voltageTemp && voltageTemp <= voltageMax) {
				voltage = voltageTemp;
			} else if (voltageTemp > voltageMax) {
				voltage = voltageMax;
			} else if (voltageTemp < voltageMin) {
				voltage = voltageMin;
			} else {
				//voltageTemp = voltage;
			}
			voltageTemp = voltage;

			// Get number of int numbers in voltage var
			//Print the voltage to the display, set decimal point after digit position 3 (display 1 has positions 4-1)
			max7219_PrintIspecial(SEGMENT_1, voltage, 3);

			//Print to debug
			char _str[40];
			sprintf(_str,"VBUS selected: %d mV", voltage*10);
			USBPD_TRACE_Add(USBPD_TRACE_DEBUG, 0, 0, (uint8_t*)_str, strlen(_str));

			//Save TIM2 CNT value to ValPrev
			encoderValPrev = encoderVal;
		}
		break;

		case ADJUSTMENT_CURRENT:
		{
			//Get direction of encoder turning
			if (encoderVal < encoderValPrev) {
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

			int isense_Vtrip_mV = (current *G_SENSE*R_SENSE_MOHMS)/1000; // mV  (mA * mOhms * Gain)
			int isense_rawADCtrip= (V_TRIP *4095) / VDDA_APPLI; //value for AWD tershold

			Update_AWD_Thresholds(0, isense_rawADCtrip);


			// Get number of int numbers in voltage var
			//Print the voltage to the display, set decimal point after digit position 3 (display 1 has positions 4-1)
			max7219_PrintIspecial(SEGMENT_2, current, 4);

			//Print to debug
			char _str[40];
			sprintf(_str,"IBUS selected: %d mA", current);
			USBPD_TRACE_Add(USBPD_TRACE_DEBUG, 0, 0, (uint8_t*)_str, strlen(_str));

			//Save TIM2 CNT value to ValPrev
			encoderValPrev = encoderVal;

		}
		break;

		case ADJUSTMENT_CURRENT_OCP:
				{
					//Get direction of encoder turning
					if (encoderVal < encoderValPrev) {
						currentOCPTemp += val;
					} else {
						currentOCPTemp -= val;
					}

					//If required temp value is within limits, assign it to voltage
					if (currentMin <= currentOCPTemp && currentOCPTemp <= currentMax) {
						currentOCP = currentOCPTemp;
					} else {
						currentOCPTemp = currentOCP;
					}

					// Get number of int numbers in voltage var
					//Based on user selected currentOCP calculate the required DAC out...
					V_TRIP = (currentOCP * R_OCP_MOHMS * G_OCP)/1000; // mV (mA * mOhms * Gain)
					//Convert DAC_OUT voltage to 12B resolution
					dac_value = (V_TRIP *4095) / VDDA_APPLI;//__LL_ADC_DIGITAL_SCALE(LL_ADC_RESOLUTION_12B);

					//Write output with DAC..
					HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac_value);

					//Print the voltage to the display, set decimal point after digit position 3 (display 1 has positions 4-1)
					max7219_PrintIspecial(SEGMENT_2, currentOCP, 4);

					//Print to debug
					char _str[50];
					sprintf(_str,"OCP_I selected: %d mA", currentOCP);
					USBPD_TRACE_Add(USBPD_TRACE_DEBUG, 0, 0, (uint8_t*)_str, strlen(_str));

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
void enc_toggle_units_isr(void){
	/*
	const char response[] = "POWER is ON\r\n";
	        LPUART_Transmit(LPUART2, (const uint8_t*)response, sizeof(response) - 1);*/

	//Mask unwanted button interrupts caused by debouncing on exti line 3 (PD8)
	EXTI->IMR1 &= ~(EXTI_IMR1_IM8);

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
			case 1: val = 2; break;
			case 2: val = 10; break;
			case 3: val = 100; break;
			case 4: val = 1000; break;
			}
		 break;
		case ADJUSTMENT_CURRENT_OCP:
		case ADJUSTMENT_CURRENT:
			switch (encoderPress) {
			case 1: val = 5; break;
			case 2: val = 10; break;
			case 3: val = 100; break;
			case 4: val = 1000; break;
			}
		 break;
	}

	char _str[60];
	uint32_t voltageADC = BSP_PWR_VBUSGetVoltage(0);
	uint32_t currentADC= BSP_PWR_VBUSGetCurrent(0);
	uint32_t currentOCP_ADC= BSP_PWR_VBUSGetCurrentOCP(0);

	// Use snprintf to limit the number of characters written
	int len = snprintf(_str, sizeof(_str), "VBUS:%lu mV, IBUS:%lu mA, IOCP:%lu mA", voltageADC, currentADC, currentOCP_ADC);

	USBPD_TRACE_Add(USBPD_TRACE_DEBUG, 0, 0, (uint8_t*)_str, strlen(_str));

	//Erase btn (PC3) interrupt flag
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_3);

}

/*
 * Timer7 interrupt routine for button debouncing
 */
void tim7_btn_isr(void){
	//Unmask exti line 1, 2 and 3
	EXTI->IMR1 |= EXTI_IMR1_IM8; //unmask interrupt mask register on exti line 3 (PD8)
	EXTI->IMR1 |= EXTI_IMR1_IM2; //unmask interrupt mask register on exti line 2 (PB2)
	EXTI->IMR1 |= EXTI_IMR1_IM1; //unmask interrupt mask register on exti line 1 (PB1)

	//Clear update flag on TIM7
	LL_TIM_ClearFlag_UPDATE(TIM7); //Clear update flag on TIMER7
}

/*
 * Timer interrupt routine
 */
void tim14_isr(void){
	//Unmask exti line 6
	EXTI->IMR1 |= EXTI_IMR1_IM6; //unmask interrupt (PB6)

	//Alert set during turning off/on output, we need to clean it
	ocp_reset_needed = 1;

	//Clear update flag on TIM7
	LL_TIM_ClearFlag_UPDATE(TIM14); //Clear update flag on TIMER7
}


/*
 * Request button interrupt routine, request APDO with user voltage and current
 */
void sw3_on_off_isr(void){
	//HAL_GPIO_TogglePin(RELAY_ON_OFF_GPIO_Port, RELAY_ON_OFF_Pin);
	//Read SRC capability
	//USBPD_StatusTypeDef status = USBPD_ERROR;
	//status = USBPD_DPM_RequestGetSourceCapability(0);

	//Mask unwanted button interrupts caused by debouncing on exti line 1 (PB1)
	EXTI->IMR1 &= ~(EXTI_IMR1_IM1);

	//HAL_GPIO_WritePin(OCP_ALERT_GPIO_Port, OCP_RESET_Pin, GPIO_PIN_RESET);

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

	//sourcecapa_limits();

	int indexSRCAPDO = USER_SERV_FindSRCIndex(0, &powerRequestDetails, voltage*10, current, PDO_SEL_METHOD_MAX_CUR);
	//Print to debug
	char _str[70];
	sprintf(_str,"APDO request: indexSRCPDO= %lu, VBUS= %lu mV, Ibus= %d mA", indexSRCAPDO, 10*voltage, current);
	USBPD_TRACE_Add(USBPD_TRACE_DEBUG, 0, 0, (uint8_t*)_str, strlen(_str));
	USBPD_DPM_RequestSRCPDO(0, indexSRCAPDO, voltage*10, current);
	//HAL_Delay(2);
	//HAL_GPIO_WritePin(OCP_ALERT_GPIO_Port, OCP_RESET_Pin, GPIO_PIN_SET);
}

/*
 * Change between current and voltage ADJUSTMENT_STATE
 */
void sw1_toggle_i_v_isr(void){
	//Mask unwanted button interrupts caused by debouncing on exti line 2 (PB2)
	EXTI->IMR1 &= ~(EXTI_IMR1_IM2);

	//Set debouncing time in ms
	TIM7->ARR = 200;

	//Zero TIM7 counter and start counting
	LL_TIM_SetCounter(TIM7, 0); //set counter register value of timer 7 to 0
	LL_TIM_EnableCounter(TIM7); //start counting of timer 7

	// Toggle the state
	if (currentState == ADJUSTMENT_CURRENT_OCP)
	{
		currentState = ADJUSTMENT_VOLTAGE;
	}
	else if (currentState == ADJUSTMENT_VOLTAGE)
	{
		currentState = ADJUSTMENT_CURRENT;
		//Display output current
		max7219_PrintIspecial(SEGMENT_2, current, 4);
	}
	else
	{
		currentState = ADJUSTMENT_CURRENT_OCP;
		//Display output current
		max7219_PrintIspecial(SEGMENT_2, currentOCP, 4);
	}

	//Get Voltage level into TRACE
	char _str[60];
	uint32_t voltageADC = BSP_PWR_VBUSGetVoltage(0);
	uint32_t currentADC= BSP_PWR_VBUSGetCurrent(0);
	uint32_t currentOCP_ADC= BSP_PWR_VBUSGetCurrentOCP(0);

	// Use snprintf to limit the number of characters written
	int len = snprintf(_str, sizeof(_str), "VBUS:%lu mV, IBUS:%lu mA, IOCP:%lu mA", voltageADC, currentADC, currentOCP_ADC);

	USBPD_TRACE_Add(USBPD_TRACE_DEBUG, 0, 0, (uint8_t*)_str, strlen(_str));
}

void sw2_lock_isr(void){
	//Mask unwanted button interrupts caused by debouncing on exti line 2 (PB2)
	EXTI->IMR1 &= ~(EXTI_IMR1_IM2);

	//Mask alert pin during setting the relay on/off
	//EXTI->IMR1 &= ~(EXTI_IMR1_IM6);

	//Set debouncing time in ms
	TIM7->ARR = 200;

	//Zero TIM7 counter and start counting
	LL_TIM_SetCounter(TIM7, 0); //set counter register value of timer 7 to 0
	LL_TIM_EnableCounter(TIM7); //start counting of timer 7

	char _str[60];

	if (outputState == OUTPUT_OFF_STATE)
			{
				outputState = OUTPUT_ON_STATE;
				//Mask unwanted button interrupts caused by debouncing on exti line 6 (PD6)
				EXTI->IMR1 &= ~(EXTI_IMR1_IM6);

				//Set debouncing time in 6 ms
				TIM14->ARR = 6;
				//Zero TIM7 counter and start counting
				LL_TIM_SetCounter(TIM14, 0); //set counter register value of timer 7 to 0
				LL_TIM_EnableCounter(TIM14); //start counting of timer 7

				//put OCP to transparent mode so any alert during on/off is cleared
				//HAL_GPIO_WritePin(OCP_RESET_GPIO_Port, OCP_RESET_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(RELAY_ON_OFF_GPIO_Port, RELAY_ON_OFF_Pin, GPIO_PIN_SET);
				// Use snprintf to limit the number of characters written
				int len = snprintf(_str, sizeof(_str), "--------Output Disabled--------");

			}
			else {
				outputState = OUTPUT_OFF_STATE;
				HAL_GPIO_WritePin(RELAY_ON_OFF_GPIO_Port, RELAY_ON_OFF_Pin, GPIO_PIN_RESET);
				// Use snprintf to limit the number of characters written
				int len = snprintf(_str, sizeof(_str), "--------Output Enabled--------");

				//Display voltage
				max7219_PrintIspecial(SEGMENT_1,voltage, 3);
				//Display current
				max7219_PrintIspecial(SEGMENT_2, current, 4);
			}

	//HAL_GPIO_TogglePin(RELAY_ON_OFF_GPIO_Port, RELAY_ON_OFF_Pin);
	//Get Voltage level into TRACE
	USBPD_TRACE_Add(USBPD_TRACE_DEBUG, 0, 0, (uint8_t*)_str, strlen(_str));
}

void ocp_alert_isr(void) {
	//Disable relay
	//Change output state
	outputState = OUTPUT_OFF_STATE;
	//Disable output
	HAL_GPIO_WritePin(RELAY_ON_OFF_GPIO_Port, RELAY_ON_OFF_Pin, GPIO_PIN_RESET);


	// Get number of int numbers in voltage var
	//Print the voltage to the display, set decimal point after digit position 3 (display 1 has positions 4-1)
	max7219_PrintIspecial(SEGMENT_1, voltage, 3);

	//Display output current
	max7219_PrintIspecial(SEGMENT_2, current, 4);


	ocp_reset_needed = 1;

	//Clear IT flag
	__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_6);
}

#define MAX_LINE_PDO      7u
/**
  * @brief  src capa menu navigation
  * @param  Nav
  * @retval None
  * source: demo_disco.c Display_sourcecapa_menu_nav
  */
void sourcecapa_limits(void)
{
  uint8_t _str[30];
  uint8_t _max = DPM_Ports[0].DPM_NumberOfRcvSRCPDO;
  uint8_t _start = 0;
  SINKData_HandleTypeDef *dhandle = &SNK_data;

  for(int8_t index=_start; index < _max; index++)
  {
	switch(DPM_Ports[0].DPM_ListOfRcvSRCPDO[index] & USBPD_PDO_TYPE_Msk)
	{
	case USBPD_PDO_TYPE_FIXED :
	  {
		uint32_t maxcurrent = ((DPM_Ports[0].DPM_ListOfRcvSRCPDO[index] & USBPD_PDO_SRC_FIXED_MAX_CURRENT_Msk) >> USBPD_PDO_SRC_FIXED_MAX_CURRENT_Pos)*10;
		uint32_t maxvoltage = ((DPM_Ports[0].DPM_ListOfRcvSRCPDO[index] & USBPD_PDO_SRC_FIXED_VOLTAGE_Msk) >> USBPD_PDO_SRC_FIXED_VOLTAGE_Pos)*50;
		//sprintf((char*)_str, "FIXED:%2dV %2d.%dA", (int)(maxvoltage/1000), (int)(maxcurrent/1000), (int)((maxcurrent % 1000) /100));
		break;
	  }
	case USBPD_PDO_TYPE_BATTERY :
	  {

	  }
	  break;
	case USBPD_PDO_TYPE_VARIABLE :
	  {
		uint32_t maxvoltage = ((DPM_Ports[0].DPM_ListOfRcvSRCPDO[index] & USBPD_PDO_SRC_VARIABLE_MAX_VOLTAGE_Msk) >> USBPD_PDO_SRC_VARIABLE_MAX_VOLTAGE_Pos) * 50;
		uint32_t minvoltage = ((DPM_Ports[0].DPM_ListOfRcvSRCPDO[index] & USBPD_PDO_SRC_VARIABLE_MIN_VOLTAGE_Msk) >> USBPD_PDO_SRC_VARIABLE_MIN_VOLTAGE_Pos) * 50;
		uint32_t maxcurrent = ((DPM_Ports[0].DPM_ListOfRcvSRCPDO[index] & USBPD_PDO_SRC_VARIABLE_MAX_CURRENT_Msk) >> USBPD_PDO_SRC_VARIABLE_MAX_CURRENT_Pos) * 10;
		//sprintf((char*)_str, "V:%2d.%1d-%2d.%1dV %d.%dA", (int)(minvoltage/1000),(int)(minvoltage/100)%10, (int)(maxvoltage/1000),(int)(maxvoltage/100)%10, (int)(maxcurrent/1000), (int)((maxcurrent % 1000) /100));
	  }
	  break;
	case USBPD_PDO_TYPE_APDO :
	  {
		uint32_t minvoltage = ((DPM_Ports[0].DPM_ListOfRcvSRCPDO[index] & USBPD_PDO_SRC_APDO_MIN_VOLTAGE_Msk) >> USBPD_PDO_SRC_APDO_MIN_VOLTAGE_Pos) * 100;
		uint32_t maxvoltage = ((DPM_Ports[0].DPM_ListOfRcvSRCPDO[index] & USBPD_PDO_SRC_APDO_MAX_VOLTAGE_Msk) >> USBPD_PDO_SRC_APDO_MAX_VOLTAGE_Pos) * 100;
		uint32_t maxcurrent = ((DPM_Ports[0].DPM_ListOfRcvSRCPDO[index] & USBPD_PDO_SRC_APDO_MAX_CURRENT_Msk) >> USBPD_PDO_SRC_APDO_MAX_CURRENT_Pos) * 50;
		//sprintf((char*)_str, "A:%2d.%1d-%2d.%1dV %d.%dA",(int) (minvoltageAPDOtemp/1000),(int)(minvoltageAPDOtemp/100)%10, (int)(maxvoltageAPDOtemp/1000),(int)(maxvoltageAPDOtemp/100)%10, (int)(maxcurrentAPDOtemp/1000), (int)((maxcurrentAPDOtemp % 1000) /100));

		if (minvoltage < dhandle->voltageMin*10) {
			dhandle -> voltageMin = (int)minvoltage/10;
		}
		if (maxvoltage > dhandle->voltageMax*10) {
			dhandle -> voltageMax = (int)maxvoltage/10;
		}
		if (maxcurrent > dhandle->currentMax) {
			dhandle -> currentMax = (int)maxcurrent;
		}
	  }
	  break;
	default :
	  sprintf((char*)_str,"Unknown Source PDO");
	  break;
	}
  }
}

/*
void usart2_lupart2_handler(void)
{
    if (LL_LPUART_IsActiveFlag_RXNE_RXFNE(LPUART2) && LL_LPUART_IsEnabledIT_RXNE_RXFNE(LPUART2))
    {
        uint8_t received_char = LL_LPUART_ReceiveData8(LPUART2);
        if (rxIndex < RX_BUFFER_SIZE-1)
        {
            rxBuffer[rxIndex++] = received_char;
            if (rxIndex >= 2 && rxBuffer[rxIndex-2] == '\r' && rxBuffer[rxIndex-1] == '\n')
            {
            	// Check for CRLF ending
				rxBuffer[rxIndex-2] = '\0';  // Null-terminate the string, removing CRLF

				// Print received command for debugging
				LPUART_Transmit(LPUART2, (const uint8_t*)"Received: ", 10);
				LPUART_Transmit(LPUART2, rxBuffer, rxIndex-2);
				LPUART_Transmit(LPUART2, (const uint8_t*)"\r\n", 2);

                processCommand(rxBuffer, rxIndex);
                rxIndex = 0;
            }
        }
    }
}

void LPUART_Transmit(USART_TypeDef *LPUARTx, const uint8_t *pData, uint16_t Size)
{
    for (uint16_t i = 0; i < Size; i++)
    {
        // Wait until TXE flag is set (Transmit data register empty)
        while (!LL_LPUART_IsActiveFlag_TXE(LPUARTx));

        // Transmit 8-bit data
        LL_LPUART_TransmitData8(LPUARTx, pData[i]);
    }

    // Wait until TC flag is set (Transmission complete)
    while (!LL_LPUART_IsActiveFlag_TC(LPUARTx));
}
*/

void processUSBCommand(uint8_t* command, uint32_t length)
{
    if (strncmp((char*)command, "POWERON", length) == 0)
    {
        const char* response = "POWER is ON\r\n";
        CDC_Transmit_FS((uint8_t*)response, strlen(response));
    }
    else if (strncmp((char*)command, "POWEROFF", length) == 0)
    {
        const char* response = "POWER is OFF\r\n";
        CDC_Transmit_FS((uint8_t*)response, strlen(response));
    }
    else
    {
        const char* response = "Unknown command\r\n";
        CDC_Transmit_FS((uint8_t*)response, strlen(response));
    }
}



