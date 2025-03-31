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
#include "usbpd_pwr_user.h"
#include "usbpd_user_services.h"
#include "string.h"
#include "stdio.h"
#include "stdbool.h"
#include <stm32g0xx_ll_adc.h>

#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <usbpd_trace.h>


int32_t BSP_USBPD_PWR_VBUSGetCurrentOCP(uint32_t Instance, int32_t *pCurrentOCP);
int32_t BSP_PWR_VBUSGetCurrentOCP(uint32_t PortId);

//Variables declaration
int encoderVal; //TIM2 CNT register reading
int encoderValPrev;
int dac_value = 500;

int ocp_reset_needed = 0;

uint32_t srcPdoIndex; //variable that holds Pdo index from FindVoltageIndex
USBPD_DPM_SNKPowerRequestDetailsTypeDef powerRequestDetails;
USBPD_StatusTypeDef powerProfiles;

__IO uint16_t aADCxConvertedValues[ADC_NUM_OF_SAMPLES] = {0};

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
uint8_t usb_buffer[64]; //in C, array automaticaly decays to pointer to first element uint8_t*

uint8_t *getUSBbuffer(void) {
	return usb_buffer;
}

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
uint32_t usbReadTime = 0;

//Initialize button event and time struct
SystemEvents_TypeDef systemEvents = {0};
BtnPressTimes_TypeDef btnPressTimes = {0};
volatile uint32_t debouncedPins = 0;  // Bitmask for tracking debounced buttons

//Init stateMachine struct
StateMachine_TypeDef stateMachine = {
		.currentState = STATE_INIT,
		.comState = STATE_CLOSED,
		.lockMode = UNLOCKED,
		.OCPMode = OCP_DISABLED,
		.pwrMode = MODE_FIXED,
		.encoder = {
				.selDigit = 2,
				.increment = 10    // Default increment
			}
};

SINKData_HandleTypeDef SNK_data = {
	.voltageMin = 500, //initial min voltage
	.voltageSet = 500, //initial value to display
	.voltageMax = 500, //initial max voltage
	.currentSet = 1000, //initial value to display
	.currentMin = 0,
	.currentOCPSet = 1000,
	.selMethod = PDO_SEL_METHOD_MAX_CUR,
	.hasAPDO = false,
	.srcProfiles[0].profileType = UNKNOWN,
};

// Define the pointer to the struct
SINKData_HandleTypeDef *dhandle = &SNK_data;
StateMachine_TypeDef *sm = &stateMachine;
SystemEvents_TypeDef *events = &systemEvents;


void runStateMachine(void) {
	// Process events and transitions
	switch (sm->currentState) {
		case STATE_OFF:
			//handleOffState(sm, dhandle);
			break;
		case STATE_INIT:
			handleInitState();
			break;
		case STATE_IDLE:
			handleIdleState();
			break;
		case STATE_ACTIVE:
			handleActiveState();
			break;
		case STATE_ERROR:
			//handleErrorState(sm, dhandle);
			break;
		case STATE_SET_VALUES:
			handleSetValuesState();
			break;
		default:
			// Error handling
			sm->currentState = STATE_ERROR;
			//sm->errorCode = ERROR_INVALID_STATE;
			break;
	}

}

// Callback when ADC conversion is complete
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  HAL_GPIO_TogglePin(LED_USER_GPIO_Port, LED_USER_Pin);
}

// Callback when ADWG2 (CH7 ISENSE) goes out of range
void HAL_ADCEx_LevelOutOfWindow2Callback(ADC_HandleTypeDef *hadc) {
	systemEvents.awdgEvent = true;
	//Disable output
	HAL_GPIO_WritePin(RELAY_ON_OFF_GPIO_Port, RELAY_ON_OFF_Pin, GPIO_PIN_RESET);
}

/**
 * Update ADC CH3 AWD Treshold
 * Possibility to update parameters on the fly (read more in HAL_ADC_AnalogWDGConfig declaration)
 * Full config and AWD init in main.c
 */
void Update_AWD_Thresholds(uint32_t low, uint32_t high, uint32_t adc_watchdog) {
		// Just update the thresholds for an already configured AWD
		ADC_AnalogWDGConfTypeDef AnalogWDGConfig = {0};
		AnalogWDGConfig.WatchdogNumber = adc_watchdog; // Specify which AWD you're updating
		AnalogWDGConfig.HighThreshold = high;
		AnalogWDGConfig.LowThreshold = low;
		if (HAL_ADC_AnalogWDGConfig(&hadc1, &AnalogWDGConfig) != HAL_OK)
		{
		    Error_Handler();
		}
}
/*
 * Initialization function
 */
void app_init(void){

	//TIM7 initialization
	LL_TIM_ClearFlag_UPDATE(TIM7); //Clear update flag on TIMER7
	LL_TIM_EnableIT_UPDATE(TIM7); //Enable interrupt generation when timer goes to max value and UPDATE event flag is set
	//LL_TIM_ClearFlag_UPDATE(TIM7); //Clear update flag on TIMER7

	//TIM14 initialization
	LL_TIM_DisableIT_UPDATE(TIM14); //Enable interrupt generation when timer goes to max value and UPDATE event flag is set
	LL_TIM_ClearFlag_UPDATE(TIM14); //Clear update flag on TIMER14

	//TIM14 initialization
	LL_TIM_DisableIT_UPDATE(TIM15); //Enable interrupt generation when timer goes to max value and UPDATE event flag is set
	LL_TIM_ClearFlag_UPDATE(TIM15); //Clear update flag on TIMER14

	//TIM3 initialization of encoder
	HAL_TIM_Encoder_Start_IT(&htim3, TIM_CHANNEL_ALL);
	__HAL_TIM_SET_COUNTER(&htim3, 30000); //write non 0 value to avoid shift from 0 -> max value
	sm->encoder.curValue = __HAL_TIM_GET_COUNTER(&htim3)/4;
	sm->encoder.prevValue = sm->encoder.curValue;

	/*
	//Set tim IT freq to 10Khz (TIM4 runs on PCLK 64MHz)
	TIM4->PSC = 64-1;
	TIM4->ARR = 100-1;

	// Enable update interrupt
	LL_TIM_EnableIT_UPDATE(TIM4);
	LL_TIM_ClearFlag_UPDATE(TIM4);
	LL_TIM_SetCounter(TIM4, 0);
	LL_TIM_EnableCounter(TIM4);*/


	//Init DAC
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac_value);

	//Wait for hardware initialization and then turn DB to HIGH (according to TCPP01-M12 datasheet 6.5)
	HAL_Delay(200);
	HAL_GPIO_WritePin(DB_OUT_GPIO_Port, DB_OUT_Pin, GPIO_PIN_SET);

	//Calibrate and start ADC sensing with DMA
	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&aADCxConvertedValues, ADC_NUM_OF_SAMPLES);

	//TIM4 initialization
	HAL_TIM_Base_Start(&htim4);

	//Init 7 segment display
	max7219_Init( 7 );
	max7219_Decode_On();

	//Print decimal points and initial values
	max7219_PrintIspecial(SEGMENT_2, dhandle->currentSet, 4);
	max7219_PrintIspecial(SEGMENT_1, dhandle->voltageSet, 3);

	//HAL_GPIO_WritePin(OCP_RESET_GPIO_Port, OCP_RESET_Pin, GPIO_PIN_SET);
}


/*
 * Loop function
 */
void app_loop(void) {
	// Process button events
	processSystemEvents();

	// Run the state machine
	runStateMachine();

	// Reset button states after processing
	stateMachine.outputBtnPressed = false;
	stateMachine.lockBtnPressed = false;
	stateMachine.lockBtnLongPressed = false;
	stateMachine.voltageCurrentBtnPressed = false;
	stateMachine.voltageCurrentBtnLongPressed = false;
	stateMachine.rotaryBtnPressed = false;
	stateMachine.encoderTurnedFlag = false;
	stateMachine.stateTimeoutFlag = false;
	stateMachine.periodicCheckFlag = false;
	stateMachine.encoderTurnedFlag = false;
	stateMachine.awdgTriggeredFlag = false;
}

/*
 * Define Callbacks and ISR
 */
//BTN ISR to set event flags
void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == SW1_TOGGLE_I_V_Pin) {
        btnPressTimes.voltageCurrentBtn = HAL_GetTick();
        systemEvents.btnPressEvent = true;
        //EXTI->IMR1 &= ~(EXTI_IMR1_IM2);
    } else if (GPIO_Pin == SW2_DEBUG_BTN_Pin) {
        btnPressTimes.lockBtn = HAL_GetTick();
        systemEvents.btnPressEvent = true;
        //EXTI->IMR1 &= ~(EXTI_IMR1_IM4);
        //lockButtonPressTime = HAL_GetTick();
    } else if (GPIO_Pin == SW3_OFF_ON_Pin) {
        systemEvents.outputBtnEvent = true;
        //EXTI->IMR1 &= ~(EXTI_IMR1_IM1);
    } else if (GPIO_Pin == ENC_TOGGLE_UNITS_Pin) {
        systemEvents.rotaryBtnEvent = true;
        //EXTI->IMR1 &= ~(EXTI_IMR1_IM8);
    }

    // Store the pressed button in bitmask for tracking debounce
    debouncedPins |= GPIO_Pin;
    // Mask the interrupt for this button
    EXTI->IMR1 &= ~GPIO_Pin;

    // Start debounce timer
    TIM7->ARR = 50;
    LL_TIM_SetCounter(TIM7, 0);
    LL_TIM_EnableCounter(TIM7);
}

/*
 * Define Callbacks and ISR
 */
//BTN ISR to set event flags
void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin) {

	if (systemEvents.btnPressEvent == true) {
		uint32_t releaseTime = HAL_GetTick();

		if (GPIO_Pin == SW1_TOGGLE_I_V_Pin) {
			if ((releaseTime - btnPressTimes.voltageCurrentBtn) > 1000) {
				systemEvents.voltageCurrentBtnLongEvent = true;
			} else {
				systemEvents.voltageCurrentBtnEvent = true;
			}

		} else if (GPIO_Pin == SW2_DEBUG_BTN_Pin) {

			if ((releaseTime - btnPressTimes.lockBtn) > 1000) {
				systemEvents.lockBtnLongEvent = true;
			} else {
				systemEvents.lockBtnEvent = true;
			}
		} else {
			return;
		}

		// Start debounce timer
		TIM7->ARR = 50;
		LL_TIM_SetCounter(TIM7, 0);
		LL_TIM_EnableCounter(TIM7);

		//Reset btnPressEvent
		systemEvents.btnPressEvent = false;

		 // Store the pressed button in bitmask for tracking debounce
		debouncedPins |= GPIO_Pin;
		// Mask the interrupt for this button
		EXTI->IMR1 &= ~GPIO_Pin;
	}
}

//TIM capture callback
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM3) {
    	systemEvents.encoderTurnEvent = true;
    }
}

/*
 * Timer7 interrupt routine for button debouncing
 */
void TIM7_ISR(void){
	//Unmask exti line 1, 2 and 4,8

	LL_TIM_ClearFlag_UPDATE(TIM7);
	LL_TIM_DisableCounter(TIM7);

	// Unmask only the buttons that were debounced
	EXTI->IMR1 |= debouncedPins;

	// Reset the bitmask after unmasking
	debouncedPins = 0;

}

void TIM14_ISR(void) {
	if (LL_TIM_IsActiveFlag_UPDATE(TIM14)) {
		// Clear the update interrupt flag
		LL_TIM_ClearFlag_UPDATE(TIM14);

		// Handle periodic check for ACTIVE state
		systemEvents.periodicCheckEvent = true;
		//Reset CNT value
		LL_TIM_SetCounter(TIM14, 0);

		//Start timer again
		LL_TIM_EnableCounter(TIM14);
	}
}


void TIM15_ISR(void) {
	if (LL_TIM_IsActiveFlag_UPDATE(TIM15)) {
		// Clear the update interrupt flag
		LL_TIM_ClearFlag_UPDATE(TIM15);

		// Handle periodic check for ACTIVE state
		systemEvents.stateTimeoutEvent = true;
		//Reset CNT value
		//LL_TIM_DisableCounter(TIM15);  // Stop the timer after timeout

	}
}

/*
 * Define Process functions
 */
// Process button events in the main loop (Convert hardware events into logical events)
void processButtonEvents(void) {
    if (events->outputBtnEvent) {
    	//Reset btn event flag
        events->outputBtnEvent = false;
        sm->outputBtnPressed = true;

    } else if (events->voltageCurrentBtnEvent) {
    	events->voltageCurrentBtnEvent = false;
    	sm->voltageCurrentBtnPressed = true;
    } else if (events->voltageCurrentBtnLongEvent) {
       	events->voltageCurrentBtnLongEvent = false;
       	sm->voltageCurrentBtnLongPressed = true;

       	if (sm->pwrMode == MODE_FIXED) {
       		sm->pwrMode = MODE_APDO;
       	} else {
       		sm->pwrMode = MODE_FIXED;
       	}

    } else if (events->lockBtnEvent) {
    	events->lockBtnEvent = false;

    	if (sm->lockMode == UNLOCKED) {
    		sm->lockBtnPressed = true;

			//Toggle OCP mode
			switch(sm->OCPMode) {
				case OCP_DISABLED:
					sm->OCPMode = OCP_ENABLED;

					//Update AWD limits
					int isense_Vtrip_mV = (dhandle->currentSet *G_SENSE*R_SENSE_MOHMS)/1000; // mV  (mA * mOhms * Gain)
					int isense_rawADCtrip= (isense_Vtrip_mV *4095) / VDDA_APPLI; //value for AWD treshold
					Update_AWD_Thresholds(0, isense_rawADCtrip, ADC_ANALOGWATCHDOG_2);
					break;
				case OCP_ENABLED:
					sm->OCPMode = OCP_DISABLED;
					//Update AWD limits
					Update_AWD_Thresholds(0, OCP_DISABLED_HT, ADC_ANALOGWATCHDOG_2);

					//__HAL_ADC_DISABLE_IT(&hadc1, ADC_IT_AWD2); can be also used to write bit but
					//only when ADSTART bit is cleared to 0 (this ensures that no conversion is ongoing).
					break;
			}
    	}

    } else if (events->lockBtnLongEvent) {
    	events->lockBtnLongEvent = false;
    	sm->lockBtnLongPressed = true;

    	if (sm->lockMode == LOCKED) {
    		sm->lockMode = UNLOCKED;

    		//Drive lock LED
    		HAL_GPIO_WritePin(LED_LOCK_GPIO_Port, LED_LOCK_Pin, GPIO_PIN_RESET);

    		// Mask buttons except OUT
    		EXTI->IMR1 |= EXTI_IMR1_IM8; //unmask ENCbtn
    		EXTI->IMR1 |= EXTI_IMR1_IM2; //unmask SW1 I/V
    	} else {
    		sm->lockMode = LOCKED;

    		//Drive lock LED
    		HAL_GPIO_WritePin(LED_LOCK_GPIO_Port, LED_LOCK_Pin, GPIO_PIN_SET);

    		// Unmask buttons except OUT
    		EXTI->IMR1 &= ~(EXTI_IMR1_IM2); // mask SW1 I/V
    		EXTI->IMR1 &= ~(EXTI_IMR1_IM8); // mask ENCbtnt
    	}

    } else if (events->rotaryBtnEvent) {
    	//Reset btn event flag
    	events->rotaryBtnEvent = false;
		sm->rotaryBtnPressed = true;

    }
}

// Main system event processor that calls specialized handlers
void processSystemEvents(void) {
    // Process different event types using specialized functions
    processButtonEvents();

    if (events->encoderTurnEvent) {
    	sm->encoderTurnedFlag = true;
    	events->encoderTurnEvent = false;
    }
    if (events->stateTimeoutEvent) {
		sm->stateTimeoutFlag= true;
		events->stateTimeoutEvent = false;
	}
	if (events->periodicCheckEvent) {
		sm->periodicCheckFlag= true;
		events->periodicCheckEvent = false;
	}
    // Process AWDG events
    if (events->awdgEvent) {
        events->awdgEvent = false;
        sm->awdgTriggeredFlag = true;
    }
}

void handleCOMportstatus(uint8_t host_com_port_open){
	 static bool entryDone = false;
	 if (host_com_port_open == 1 && !entryDone) {
		 //Set COM state to OPEN
		 sm->comState = STATE_OPEN;
		 //Set default state timeout to 4000;
		 sm->timeoutCounter = 4000;

		 //Drive lock LED
		 HAL_GPIO_WritePin(LED_LOCK_GPIO_Port, LED_LOCK_Pin, GPIO_PIN_SET);

		 //Disable BTN interrupts
		 EXTI->IMR1 &= ~(EXTI_IMR1_IM2); //SW1
		 EXTI->IMR1 &= ~(EXTI_IMR1_IM4); //SW2
		 EXTI->IMR1 &= ~(EXTI_IMR1_IM1); //SW3
		 EXTI->IMR1 &= ~(EXTI_IMR1_IM8); //ENC btn

		 entryDone = true;
	 } else if (host_com_port_open == 0) {
		 sm->comState = STATE_CLOSED;
		 //Reset lock LED
		 HAL_GPIO_WritePin(LED_LOCK_GPIO_Port, LED_LOCK_Pin, GPIO_PIN_RESET);

		 //Unmask exti line 1, 2 and 3
		 EXTI->IMR1 |= EXTI_IMR1_IM8; //unmask exti line 8
		 EXTI->IMR1 |= EXTI_IMR1_IM4; //unmask exti line 4
		 EXTI->IMR1 |= EXTI_IMR1_IM2; //unmask exti line 2
		 EXTI->IMR1 |= EXTI_IMR1_IM1; //unmask exti line 1

		 //If COM closed return savely to IDLE
		 sm->currentState = STATE_IDLE;

		 entryDone = false;
	 }
}
void cleanString(const char* input, char* output, const char* delimiter) {
    char temp[128];  // Temporary buffer for safe modification
    strncpy(temp, input, sizeof(temp) - 1);
    temp[sizeof(temp) - 1] = '\0';  // Ensure null-termination

    char* token = strtok(temp, delimiter);
    output[0] = '\0';  // Start with an empty output string

    while (token != NULL) {
        strcat(output, token);  // Append token to output
        token = strtok(NULL, delimiter);
    }
}

void processUSBCommand(uint8_t* command, uint32_t length)
{
	// Null-terminate the command to ensure string functions work properly
	command[length] = '\0';

	// Make a copy for tokenization
	char cmd_copy[64] = {0};
	strncpy(cmd_copy, (char*)command, sizeof(cmd_copy)-1);

	// Clear white spaces..
	char cmd_trimmed[64];
	cleanString(cmd_copy, cmd_trimmed, " ");

	// Extract command part (everything up to : )
	char* cmd_part = strtok(cmd_trimmed, ":");
	char* params = NULL; // extracts the part after "numbers"

	// Parameters start after the delimiter
	if (cmd_part != NULL && strlen(cmd_part) < length) {
		params = strtok(NULL, ":");  // Get remaining part after ':'
	}

	// Define command table
	/*
	static const Command_t commands[] = {
		{"*IDN?", idnHandler},
		{"PROFILES", profilesHandler},
		{"VOUT", voutHandler},
		{"IOUT", ioutHandler},
		{"VSET", vsetHandler},
		{"ISET", isetHandler},
		{"OCP", ocpHandler},
		{"OUT", outHandler},
		{NULL, NULL} // Sentinel
	};*/

	//Create buffer for response
	char response[64];

	//Process commands
    if (strcmp(cmd_part, "OCP1") == 0)
    {
    	snprintf(response, sizeof(response), "OCP enabled\r\n");

        sm->OCPMode = OCP_ENABLED;

		//Update AWD limits
		int isense_Vtrip_mV = (dhandle->currentSet *G_SENSE*R_SENSE_MOHMS)/1000; // mV  (mA * mOhms * Gain)
		int isense_rawADCtrip= (isense_Vtrip_mV *4095) / VDDA_APPLI; //value for AWD treshold
		Update_AWD_Thresholds(0, isense_rawADCtrip, ADC_ANALOGWATCHDOG_2);
    }
    else if (strcmp(cmd_part, "OCP0") == 0)
    {
    	snprintf(response, sizeof(response), "OCP disabled\r\n");

        sm->OCPMode = OCP_DISABLED;
		//Update AWD limits
		Update_AWD_Thresholds(0, OCP_DISABLED_HT, ADC_ANALOGWATCHDOG_2);
    }
    else if (strcmp(cmd_part, "VSET1") == 0)
    {
    	sm->rotaryBtnPressed = true;
    	sm->setValueMode = SET_VOLTAGE;

    	uint32_t voltage = atof(params)*100; // Convert float to uint (also V to centivolts)

    	if (dhandle->voltageMin < voltage && voltage < dhandle->voltageMax) {
    	   	dhandle->voltageSet = voltage; //save in centivolts
    		snprintf(response, sizeof(response), "Voltage set to new value: %lu.%02lu V\r\n", dhandle->voltageSet / 100, dhandle->voltageSet % 100);
    	} else {
    		snprintf(response, sizeof(response), "ERROR: Voltage out of bounds\r\n");
    	}
    }
    else if (strcmp(cmd_part, "TIMEOUT1") == 0)
	{
		uint32_t ms = atof(params)*1000; // Convert float to uint milliseconds

		if (0 < ms && ms < 10000) {
			sm->timeoutCounter = ms;
			snprintf(response, sizeof(response), "Timeout set to new value: %lu.%03lu s\r\n", ms / 1000, ms % 1000);
		} else {
			snprintf(response, sizeof(response), "ERROR: Timeout out of bounds (0-10s)\r\n");
		}
	}
    else if (strcmp(cmd_part, "VSET1?") == 0)
	{
		// Convert to volts and format as "XX.XX V"
		snprintf(response, sizeof(response), "Voltage is set to: %lu.%02lu V\r\n", dhandle->voltageSet / 100, dhandle->voltageSet % 100);

		sm->OCPMode = OCP_DISABLED;
		//Update AWD limits
		Update_AWD_Thresholds(0, OCP_DISABLED_HT, ADC_ANALOGWATCHDOG_2);
	}
    else if (strcmp(cmd_part, "ISET1") == 0)
	{
    	sm->rotaryBtnPressed = true;
    	sm->setValueMode = SET_CURRENT;

		uint32_t current = atof(params)*1000; // Convert float parameter to integer

		if (dhandle->currentMin < current && current < dhandle->currentMax) {
			dhandle->currentSet = current; //save in mA
			snprintf(response, sizeof(response), "Current set to new value: %lu.%03lu A\r\n", dhandle->currentSet / 1000, dhandle->currentSet % 1000);
		} else {
			snprintf(response, sizeof(response), "ERROR: Current out of bounds\r\n");
		}

	}
    else if (strcmp(cmd_part, "ISET1?") == 0)
  	{
  		snprintf(response, sizeof(response), "Current is set to: %lu.%03lu A\r\n", dhandle->currentSet / 1000, dhandle->currentSet % 1000);

  		sm->OCPMode = OCP_DISABLED;
  		//Update AWD limits
  		Update_AWD_Thresholds(0, OCP_DISABLED_HT, ADC_ANALOGWATCHDOG_2);
  	}
    else if (strcmp(cmd_part, "OUT0") == 0)
   	{
   		snprintf(response, sizeof(response), "Output disabled\r\n");
   		//Simulate button press
   		if (sm->currentState == STATE_ACTIVE) {
   	   		sm->outputBtnPressed = true;
   		}
   	}
    else if (strcmp(cmd_part, "OUT1") == 0)
	{
		snprintf(response, sizeof(response), "Output enabled\r\n");

		//Simulate button press
		if (sm->currentState == STATE_IDLE) {
			sm->outputBtnPressed = true;
		}
	}
    else if (strcmp(cmd_part, "VOUT1?") == 0)
   	{
   		snprintf(response, sizeof(response), "Measured output voltage: %lu.%02lu V\r\n", dhandle->voltageMeas / 100, dhandle->voltageMeas % 100);
   	}
    else if (strcmp(cmd_part, "IOUT1?") == 0)
   	{
  		snprintf(response, sizeof(response), "Current is set to: %lu.%03lu A\r\n", dhandle->currentMeas / 1000, dhandle->currentMeas % 1000);
   	}
    else if (strcmp(cmd_part, "*IDN?") == 0)
	{
		snprintf(response, sizeof(response), "USB-PD PPS Sink v0.2 (100W,22V,5A)\r\n");
	}
    else if (strcmp(cmd_part, "PROFILES?") == 0)
   	{
    	bool printToCOM = true;
    	sourcecapa_limits(printToCOM);
   	}
    else
    {
        snprintf(response, sizeof(response), "Unknown command\r\n");
    }

    // Send response
    CDC_Transmit_FS((uint8_t*)response, strlen(response));
}

/*
 * Define state Handle functions
 */
void handleInitState(void) {
    // Entry actions (if just entered this state)
    static bool entryDone = false;

    //=======================================================
	// ENTRY ACTIONS - Executed once when entering the state
	//=======================================================
    if (!entryDone) {
    	// Set the state entry time and timeout duration
		sm->stateEntryTime = HAL_GetTick();
		sm->timeoutCounter = 5000;  // 2 seconds timeout

        // Save state for return from temporary states
        strcpy(sm->lastStateStr, "INIT");
        sm->lastState = STATE_INIT;

        entryDone = true;
    }

    //==========================================================
	// DO ACTIONS - Executed every time the state is processed
	//==========================================================


    //=================================================
	// TRANSITION CHECKS - Check for state transitions
	//=================================================
    if (dhandle->srcProfiles[0].profileType != UNKNOWN) {
    	// Display maximal values
    	max7219_PrintIspecial(SEGMENT_2, dhandle->currentMax, 4);
    	max7219_PrintIspecial(SEGMENT_1, dhandle->voltageMax, 3);

    	// If APDO available set APDO mode
    	if (dhandle->hasAPDO == true) {
    		sm->pwrMode = MODE_APDO;
    	} else {
    		sm->pwrMode = MODE_FIXED;
    	}

    	// Check if the timeout has elapsed
		if (HAL_GetTick() - sm->stateEntryTime > sm->timeoutCounter) {
			//After initialization transition to IDLE state
			sm->currentState = STATE_IDLE;
			entryDone = false;
		}
    }
}

void handleIdleState(void) {
    // Entry actions (if just entered this state)
    static bool entryDone = false;

    //=======================================================
	// ENTRY ACTIONS - Executed once when entering the state
	//=======================================================
    if (!entryDone) {
        // Display set values
        max7219_PrintIspecial(SEGMENT_2, dhandle->currentSet, 4);
        max7219_PrintIspecial(SEGMENT_1, dhandle->voltageSet, 3);

        // Ensure output is off
        HAL_GPIO_WritePin(RELAY_ON_OFF_GPIO_Port, RELAY_ON_OFF_Pin, GPIO_PIN_RESET);

        // Check temperature and control fan (not shown in your code)

        // Save state for return from temporary states
        strcpy(sm->lastStateStr, "IDLE");
        sm->lastState = STATE_IDLE;

        entryDone = true;
    }

    //=================================================
	// TRANSITION CHECKS - Check for state transitions
	//=================================================

    // Process events and transitions
    if (sm->outputBtnPressed) {
        sm->currentState = STATE_ACTIVE;
        entryDone = false;
    } else if (sm->rotaryBtnPressed) {
        sm->currentState = STATE_SET_VALUES;
        if (sm->comState == STATE_CLOSED) {
        	sm->timeoutCounter = 4000;  // 4 seconds timeout
        }
        sm->rotaryBtnPressed = false;
        entryDone = false;
    }
}

void handleActiveState(void) {
	//Declare static variables
	// Set check interval Make the timer persistent across function calls
	//static uint32_t lastCheckTime = 0; // declared to 0 only once, then retains value
    static bool entryDone = false;
	//const uint32_t CHECK_INTERVAL_MS = 500; // Check every 500ms

	//=======================================================
	// ENTRY ACTIONS - Executed once when entering the state
	//=======================================================

    if (!entryDone) {
        // Ensure output is off
        HAL_GPIO_WritePin(RELAY_ON_OFF_GPIO_Port, RELAY_ON_OFF_Pin, GPIO_PIN_SET);

        // Save state for return from temporary states
        strcpy(sm->lastStateStr, "ACTIVE");
        sm->lastState = STATE_ACTIVE;

        // Initialize the periodic check timer
        TIM14->ARR = 500;
		LL_TIM_SetCounter(TIM14, 0); //set counter register value of timer 7 to 0
		LL_TIM_EnableIT_UPDATE(TIM14); // Enable update interrupt
		LL_TIM_EnableCounter(TIM14);

        entryDone = true;
    }

    //==========================================================
	// DO ACTIONS - Executed every time the state is processed
	//==========================================================

    //Periodic check to display measured values
    if (sm->periodicCheckFlag) {
		uint32_t vol = BSP_PWR_VBUSGetVoltage(0)/10; //divide by 10 to get centivolts since only 4 digit display..
		uint32_t cur = BSP_PWR_VBUSGetCurrent(0);

		dhandle ->currentMeas = cur;
		dhandle ->voltageMeas = vol;
		//Display output voltage
		max7219_PrintIspecial(SEGMENT_1, vol, 3);
		//Display output current
		max7219_PrintIspecial(SEGMENT_2, cur, 4);
    }

    //=================================================
	// TRANSITION CHECKS - Check for state transitions
	//=================================================
    if (sm->outputBtnPressed) {
        sm->currentState = STATE_IDLE;
        entryDone = false;
    } else if (sm->awdgTriggeredFlag) {
        sm->currentState = STATE_IDLE;
        entryDone = false;
    } else if (sm->rotaryBtnPressed) {
        sm->currentState = STATE_SET_VALUES;
        if (sm->comState == STATE_CLOSED) {
			sm->timeoutCounter = 4000;  // 4 seconds timeout
		}
        sm->rotaryBtnPressed = false;
        entryDone = false;
    }

    //EXIT ACTION
    if (!entryDone) {
    	LL_TIM_DisableCounter(TIM14);
    }
}

void handleSetValuesState(void) {
    // Entry actions (if just entered this state)
    static bool entryDone = false;
    static bool showDigit = false;  // Start with digit shown

    //=======================================================
   	// ENTRY ACTIONS - Executed once when entering the state
   	//=======================================================

    if (!entryDone) {

    	// Display set values
        max7219_PrintIspecial(SEGMENT_2, dhandle->currentSet, 4);
        max7219_PrintIspecial(SEGMENT_1, dhandle->voltageSet, 3);

        // Initialize the periodic timer
		TIM14->ARR = 500;
		LL_TIM_SetCounter(TIM14, 0); //set counter register value of timer 14 to 0
		LL_TIM_EnableIT_UPDATE(TIM14); // Enable update interrupt
		LL_TIM_EnableCounter(TIM14);

		// Initialize the timeout timer
		LL_TIM_DisableCounter(TIM15);
		TIM15->ARR = sm->timeoutCounter;
		LL_TIM_SetCounter(TIM15, 0); //set counter register value of timer 14 to 0
		LL_TIM_EnableIT_UPDATE(TIM15); // Enable update interrupt
		LL_TIM_EnableCounter(TIM15);

        entryDone = true;
    }

    //==========================================================
	// DO ACTIONS - Executed every time the state is processed
	//==========================================================

    // User interaction - reset the timeout
	if (sm->rotaryBtnPressed || sm->encoderTurnedFlag || sm->voltageCurrentBtnPressed) {
		// Reset the timeout timer whenever there's user interaction
		LL_TIM_DisableCounter(TIM15);
		LL_TIM_SetCounter(TIM15, 0); //set counter register value of timer 14 to 0
		LL_TIM_EnableIT_UPDATE(TIM15); // Enable update interrupt
		LL_TIM_EnableCounter(TIM15);
	}

	//Process voltageCurrentBtn press
	if (sm->voltageCurrentBtnPressed) {
		//Reset flag
		sm->voltageCurrentBtnPressed = false;
		//Process
		if (sm->setValueMode == SET_VOLTAGE) {
			sm->setValueMode = SET_CURRENT;
		} else if (sm->setValueMode == SET_CURRENT) {
			sm->setValueMode = SET_VOLTAGE;
		}

		//Get values into debug trace
		char _str[60];
		uint32_t voltageADC = BSP_PWR_VBUSGetVoltage(0);
		uint32_t currentADC= BSP_PWR_VBUSGetCurrent(0);
		uint32_t currentOCP_ADC= BSP_PWR_VBUSGetCurrentOCP(0);

		// Use snprintf to limit the number of characters written
		int len = snprintf(_str, sizeof(_str), "VBUS:%lu mV, IBUS:%lu mA, IOCP:%lu mA", voltageADC, currentADC, currentOCP_ADC);

		USBPD_TRACE_Add(USBPD_TRACE_DEBUG, 0, 0, (uint8_t*)_str, strlen(_str));
	}

    //Process encoder press
    if (sm->rotaryBtnPressed) {
		//Decrement encoderPress value if higher than 4
		if (sm->encoder.selDigit > 1){
			sm->encoder.selDigit--;
		}
		else {
			sm->encoder.selDigit = 4;
		}

		//Choose addition value based on setValueMode
		int val;
		switch (sm->setValueMode){
			case SET_VOLTAGE:
				switch (sm->encoder.selDigit) {
				case 1: val = 2; break;
				case 2: val = 10; break;
				case 3: val = 100; break;
				case 4: val = 1000; break;
				}
			 break;
			//case SET_CURRENT:
			case SET_CURRENT:
				switch (sm->encoder.selDigit) {
				case 1: val = 5; break;
				case 2: val = 10; break;
				case 3: val = 100; break;
				case 4: val = 1000; break;
				}
			 break;
		}

		sm->encoder.increment = val;
    }


    //Process encoder turn
    if (sm->encoderTurnedFlag) {
		// Handle encoder pulse event
		int encoderVal = (TIM3 -> CNT) >> 2;

		//
		//Erase FLAG!!
		//
		sm->encoder.curValue= encoderVal;

		if (encoderVal != sm->encoder.prevValue){
			//Get the turn direction and save it
			sm->encoder.direction = (encoderVal < sm->encoder.prevValue) ? 1 : -1;

			//Save TIM3 CNT value to ValPrev
			sm->encoder.prevValue = encoderVal;

			//Set encoder Turn event flag
			sm->encoder.turnEvent = true;
		}
    }

    // Handle digit blinking based on current set mode
    if (sm->periodicCheckFlag) {
    	if (sm->setValueMode == SET_VOLTAGE) {
			max7219_BlinkDigit2(SEGMENT_1, dhandle->voltageSet, sm->encoder.selDigit, 3, showDigit);
			max7219_PrintIspecial(SEGMENT_2, dhandle->currentSet,4);
		} else { // SET_CURRENT
			max7219_BlinkDigit2(SEGMENT_2, dhandle->currentSet, sm->encoder.selDigit, 4, showDigit);
			max7219_PrintIspecial(SEGMENT_1, dhandle->voltageSet,3);
		}
    	//Toggle showDigit
    	showDigit = !showDigit;
    }

    // On turnEvent update voltage/current
	if (sm->encoder.turnEvent) {
		//Reset event flag
		sm->encoder.turnEvent = false;
		char _str[40];
		//Update displays
		switch (sm->setValueMode) {
			case SET_VOLTAGE:
				updateVoltage();
				//Print to debug !!calling it from inside updateVoltage() results in hardFault error!!
				sprintf(_str,"VBUS selected: %lu mV", dhandle->voltageSet * 10);
				USBPD_TRACE_Add(USBPD_TRACE_DEBUG, 0, 0, (uint8_t*)_str, strlen(_str));

				break;
			case SET_CURRENT:
				updateCurrent();
				//Print to debug !!calling it from inside updateVoltage() results in hardFault error!!
				sprintf(_str,"IBUS selected: %lu mA", dhandle->currentSet);
				USBPD_TRACE_Add(USBPD_TRACE_DEBUG, 0, 0, (uint8_t*)_str, strlen(_str));

				//IF OCP enabled update the tresholds
				if (sm->OCPMode == OCP_ENABLED) {
					//Update AWD limits
					int isense_Vtrip_mV = (dhandle->currentSet *G_SENSE*R_SENSE_MOHMS)/1000; // mV  (mA * mOhms * Gain)
					int isense_rawADCtrip= (isense_Vtrip_mV *4095) / VDDA_APPLI; //value for AWD treshold
					Update_AWD_Thresholds(0, isense_rawADCtrip, ADC_ANALOGWATCHDOG_2);
				}
				break;
		}
	}


	//=================================================
	// TRANSITION CHECKS - Check for state transitions
	//=================================================

    // If timeout from setValues make USB PD request with new values
    if (sm->stateTimeoutFlag) {

    	uint32_t compVoltage = compensateVoltage();
    	//Make a USBPD request
		int indexSRCAPDO = USER_SERV_FindSRCIndex(0, &powerRequestDetails, compVoltage*10, dhandle->currentSet, dhandle ->selMethod);
		//Print to debug
		char _str[70];
		sprintf(_str,"APDO request: indexSRCPDO= %int, VBUS= %lu mV, Ibus= %lu mA", indexSRCAPDO, 10*dhandle->voltageSet, dhandle->currentSet);
		USBPD_TRACE_Add(USBPD_TRACE_DEBUG, 0, 0, (uint8_t*)_str, strlen(_str));
		USBPD_DPM_RequestSRCPDO(0, indexSRCAPDO, compVoltage*10, dhandle->currentSet);

    	//Return to last state
	    if (sm->lastState == STATE_IDLE) {
	        sm->currentState = STATE_IDLE;
	    } else if (sm->lastState == STATE_ACTIVE) {
	        sm->currentState = STATE_ACTIVE;
	    }
	    entryDone = false;
    }

    //EXIT ACTION
	if (!entryDone) {
		LL_TIM_DisableCounter(TIM14);
	}

}

/*
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
				max7219_PrintIspecial(SEGMENT_1,dhandle->voltageSet, 3);
				//Display current
				max7219_PrintIspecial(SEGMENT_2, dhandle->currentSet, 4);
			}

	//HAL_GPIO_TogglePin(RELAY_ON_OFF_GPIO_Port, RELAY_ON_OFF_Pin);
	//Get Voltage level into TRACE
	USBPD_TRACE_Add(USBPD_TRACE_DEBUG, 0, 0, (uint8_t*)_str, strlen(_str));
}*/

/**
  * @brief  src capa menu navigation
  * @param  Nav
  * @retval None
  * source: demo_disco.c Display_sourcecapa_menu_nav
  */
void sourcecapa_limits(bool printToCOM)
{
	// Entry actions (if just entered this state)
	static bool firstEntry = true;

	uint8_t _max = DPM_Ports[0].DPM_NumberOfRcvSRCPDO;
	uint8_t _start = 0;
	//SINKData_HandleTypeDef *dhandle = &SNK_data;
	dhandle->numProfiles = _max;
	static char all_profiles[500] = {0}; // Buffer for all profiles
	uint16_t offset = 0; // Position tracker in the all_profiles buffer
	char str_info[32] = {0};

	// Clear the buffer at the start
	memset(all_profiles, 0, sizeof(all_profiles));

	for(int8_t index=_start; index < _max; index++)
	{
		char _str[50] = {0};

		switch(DPM_Ports[0].DPM_ListOfRcvSRCPDO[index] & USBPD_PDO_TYPE_Msk)
		{
			case USBPD_PDO_TYPE_FIXED :
			{
				uint32_t maxcurrent = ((DPM_Ports[0].DPM_ListOfRcvSRCPDO[index] & USBPD_PDO_SRC_FIXED_MAX_CURRENT_Msk) >> USBPD_PDO_SRC_FIXED_MAX_CURRENT_Pos)*10;
				uint32_t maxvoltage = ((DPM_Ports[0].DPM_ListOfRcvSRCPDO[index] & USBPD_PDO_SRC_FIXED_VOLTAGE_Msk) >> USBPD_PDO_SRC_FIXED_VOLTAGE_Pos)*50;
				sprintf((char*)_str, "FIXED:%2dV %2d.%dA \r\n", (int)(maxvoltage/1000), (int)(maxcurrent/1000), (int)((maxcurrent % 1000) /100));
				if (maxcurrent > dhandle->currentMax) {
					dhandle -> currentMax = (int)maxcurrent;
				}
				// Copy profiles to SNK data
				if (firstEntry) {
					if ((maxvoltage/1000 * maxcurrent/1000) <= 100) {
						dhandle->srcProfiles[index].voltageMax = maxvoltage/10; //save in centivolts
						dhandle->srcProfiles[index].currentMax = maxcurrent; //save in mA
						dhandle->srcProfiles[index].profileType = FIXED;
					}
				}

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
				sprintf((char*)_str, "V:%2d.%1d-%2d.%1dV %d.%dA \r\n", (int)(minvoltage/1000),(int)(minvoltage/100)%10, (int)(maxvoltage/1000),(int)(maxvoltage/100)%10, (int)(maxcurrent/1000), (int)((maxcurrent % 1000) /100));
			}
			break;
			case USBPD_PDO_TYPE_APDO :
			{
				uint32_t minvoltage = ((DPM_Ports[0].DPM_ListOfRcvSRCPDO[index] & USBPD_PDO_SRC_APDO_MIN_VOLTAGE_Msk) >> USBPD_PDO_SRC_APDO_MIN_VOLTAGE_Pos) * 100;
				uint32_t maxvoltage = ((DPM_Ports[0].DPM_ListOfRcvSRCPDO[index] & USBPD_PDO_SRC_APDO_MAX_VOLTAGE_Msk) >> USBPD_PDO_SRC_APDO_MAX_VOLTAGE_Pos) * 100;
				uint32_t maxcurrent = ((DPM_Ports[0].DPM_ListOfRcvSRCPDO[index] & USBPD_PDO_SRC_APDO_MAX_CURRENT_Msk) >> USBPD_PDO_SRC_APDO_MAX_CURRENT_Pos) * 50;
				sprintf((char*)_str, "APDO:%2d.%1d-%2d.%1dV %d.%dA \r\n",(int) (minvoltage/1000),(int)(minvoltage/100)%10, (int)(maxvoltage/1000),(int)(maxvoltage/100)%10, (int)(maxcurrent/1000), (int)((maxcurrent % 1000) /100));

				// Set flag
				dhandle->hasAPDO = true;

				// Extract min and max values
				if (minvoltage < dhandle->voltageMin*10) {
					dhandle -> voltageMin = (int)minvoltage/10;
				}
				if (maxvoltage > dhandle->voltageMax*10) {
					dhandle -> voltageMax = (int)maxvoltage/10;
				}
				if (maxcurrent > dhandle->currentMax) {
					dhandle -> currentMax = (int)maxcurrent;
				}

				// Copy profiles to SNK data
				if (firstEntry) {
					if ((maxvoltage/1000 * maxcurrent/1000) <= 100) {
						dhandle->srcProfiles[index].voltageMin = minvoltage/10; //save in centivolts
						dhandle->srcProfiles[index].voltageMax = maxvoltage/10; //save in centivolts
						dhandle->srcProfiles[index].currentMax = maxcurrent; //save in mA
						dhandle->srcProfiles[index].profileType = APDO;
					}
				}
			}
			break;
			default :
			{
				sprintf((char*)_str,"Unknown Source PDO \r\n");
				break;
			}

			}
			// Add current profile to the buffer with bounds checking
			uint16_t len = strlen(_str);
			if (offset + len < sizeof(all_profiles) - 1) {
				memcpy(all_profiles + offset, _str, len);
				offset += len;
		} //switch end

	} //for end

	// Ensure null termination
	all_profiles[offset] = '\0';

	// Send all profiles at once
	if (printToCOM && offset > 0) {
		CDC_Transmit_FS((uint8_t*)all_profiles, offset);
	}

	// Clear first entry flag
	firstEntry = false;
}


// Helper function to update voltage
void updateVoltage(void) {

	switch (sm -> pwrMode) {
		case MODE_FIXED:
		{
				// Create helper variable
				int8_t index = dhandle->selectedProfile + sm->encoder.direction;

				//Find index of next FixedPDO based on encoder turn direction
				if ((0 <= index) && (index <= dhandle->numProfiles -1)) {
					if (dhandle->srcProfiles[index].profileType != FIXED) {
						index = 0; //we are out of fixed profiles that are defined at beginning, return to 0
					}
				} else if (index < 0) {
					// Search for the last FixedPDO
					for (index = dhandle->numProfiles -1; index >= 0; index--) {
						if (dhandle->srcProfiles[index].profileType == FIXED) {
							break;
						}
					} //for loop end
				}

				dhandle->selectedProfile = index;
				// Apply parameters of found srcProfile index
				dhandle->voltageMax = dhandle->srcProfiles[index].voltageMax;
				dhandle->voltageMin = dhandle->voltageMax;
				dhandle->voltageSet = dhandle->voltageMax;
				dhandle->currentMax = dhandle->srcProfiles[index].currentMax;

				//If new voltage has lower current limit adjust accordingly
				if (dhandle->currentSet > dhandle->currentMax) {
					dhandle->currentSet = dhandle->currentMax;
				}
		}
			break;
		case MODE_APDO:
		{
			//Get direction of encoder turning
			int voltageTemp = dhandle->voltageSet;
			voltageTemp += sm->encoder.direction * sm->encoder.increment;

			// Extract total bounds of APDO
			for (int8_t i = 0; i <= dhandle->numProfiles-1; i++) {
				if (dhandle->srcProfiles[i].profileType == APDO) {
					if (dhandle->srcProfiles[i].voltageMin < dhandle->voltageMin) {
						dhandle->voltageMin = dhandle->srcProfiles[i].voltageMin;
					}
					if (dhandle->srcProfiles[i].voltageMax > dhandle->voltageMax) {
						dhandle->voltageMax = dhandle->srcProfiles[i].voltageMax;
					}
					if (dhandle->srcProfiles[i].currentMax > dhandle->currentMax) {
						dhandle->currentMax = dhandle->srcProfiles[i].currentMax;
					}
				}

			}

			// If required temp value is within limits, assign it to voltage else assign limits
			if (dhandle->voltageMin <= voltageTemp && voltageTemp <= dhandle->voltageMax) {
				dhandle->voltageSet = voltageTemp; //asign new voltage
			} else {
				return;
			}

			int8_t index = dhandle->selectedProfile;
			// Search if there is APDO with higher current and satisfies voltageSet
			for (int8_t i = dhandle->numProfiles-1; i >= 0; i--) {
				if (dhandle->srcProfiles[i].profileType == APDO) {
					if (dhandle->voltageSet < dhandle->srcProfiles[i].voltageMax) {
						dhandle->currentMax = dhandle->srcProfiles[i].currentMax;
						index = i;
					}
				}
				else {
					break;
				}
			}
			dhandle->selectedProfile = index;
			//dhandle->selectedProfile = USER_SERV_FindSRCIndex(0, &powerRequestDetails, compVoltage*10, dhandle->currentSet, dhandle ->selMethod);
		}
			break;
		default:
		{
		}
			break;
	}


	//Print selected voltage to disp, decimal at digit 3
	max7219_PrintIspecial(SEGMENT_1, dhandle->voltageSet, 3);
}

// Helper function to update voltage and update AWD limit
void updateCurrent(void) {
	//Get direction of encoder turning
	int currentTemp = dhandle->currentSet;
	currentTemp += sm->encoder.direction * sm->encoder.increment;

	//If required temp value is within limits, assign it to voltage else assign limits
	if (dhandle->currentMin <= currentTemp && currentTemp <= dhandle->currentMax) {
		dhandle->currentSet = currentTemp; //asign new voltage
	} else {
		return;
	}

	//Print selected voltage to disp, decimal at digit 3
	max7219_PrintIspecial(SEGMENT_2, dhandle->currentSet, 4);
}

// Helper function to update voltage
void updateCurrentOCP(void) {
	//Get direction of encoder turning
	int currentTemp = dhandle->currentOCPSet;
	currentTemp += sm->encoder.direction * sm->encoder.increment;

	//If required temp value is within limits, assign it to voltage
	if ( (dhandle->currentMin <= currentTemp) && (currentTemp <= (dhandle->currentMax + 300)) ) {
		dhandle->currentOCPSet = currentTemp;
	} else {
		//currentOCPTemp = currentOCP;
	}

	int V_TRIP = (dhandle->currentOCPSet * R_OCP_MOHMS * G_OCP)/1000; // mV (mA * mOhms * Gain)
	//Convert DAC_OUT voltage to 12B resolution
	int dac_value = (V_TRIP *4095) / VDDA_APPLI;//__LL_ADC_DIGITAL_SCALE(LL_ADC_RESOLUTION_12B);
	//Write output with DAC..
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac_value);

	//Print selected voltage to disp, decimal at digit 3
	max7219_PrintIspecial(SEGMENT_2, dhandle->currentOCPSet, 4);
}

//Make voltage correction for the voltage drops on rshunts
uint32_t compensateVoltage(void) {
	uint32_t correction = (dhandle->currentMeas * (R_OCP_MOHMS + R_SENSE_MOHMS) ) / 1000;
	uint32_t compVoltage = dhandle->voltageSet + correction;

	return (compVoltage > dhandle->voltageMax) ? dhandle->voltageMax : compVoltage;
}
