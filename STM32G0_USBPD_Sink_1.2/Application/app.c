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
#include "demo_app.h"


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

//Initialize button event struct
SystemEvents systemEvents = {0};
//Init stateMachine struct
StateMachine stateMachine = {
		.currentState = STATE_INIT,
		.encoder = {
				.selDigit = 2,
				.increment = 10    // Default increment
			}
};

SINKData_HandleTypeDef SNK_data = {
	.voltageMin = 500,
	.voltageSet = 600, //initial value to display
	.currentSet = 1000, //initial value to display
	.currentMin = 0,
	.currentOCPSet = 1000,
	.selMethod = PDO_SEL_METHOD_MAX_CUR,
	.encoder = {
		.selDigit = 2,
		.increment = 10    // Default increment
	}
};

// Define the pointer to the struct
SINKData_HandleTypeDef *dhandle = &SNK_data;
StateMachine *sm = &stateMachine;


void runStateMachine(void) {
    // Process events and transitions
    switch (sm->currentState) {
        case STATE_OFF:
            //handleOffState(sm, dhandle);
            break;
        case STATE_INIT:
            handleInitState(sm, dhandle);
            break;
        case STATE_IDLE:
            handleIdleState(sm, dhandle);
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
	outputState = OUTPUT_OFF_STATE;
	//Disable output
	HAL_GPIO_WritePin(RELAY_ON_OFF_GPIO_Port, RELAY_ON_OFF_Pin, GPIO_PIN_RESET);

	//Print the voltage and current to the display, set decimal point after digit position 3 (display 1 has positions 4-1)
	max7219_PrintIspecial(SEGMENT_1, dhandle->voltageSet, 3);
	max7219_PrintIspecial(SEGMENT_2, dhandle->currentSet, 4);

}


/*
 * Initialization function
 */
void app_init(void){

	//TIM7 initialization
	LL_TIM_EnableIT_UPDATE(TIM7); //Enable interrupt generation when timer goes to max value and UPDATE event flag is set
	LL_TIM_ClearFlag_UPDATE(TIM7); //Clear update flag on TIMER7

	//TIM14 initialization
	LL_TIM_DisableIT_UPDATE(TIM14); //Enable interrupt generation when timer goes to max value and UPDATE event flag is set
	LL_TIM_ClearFlag_UPDATE(TIM14); //Clear update flag on TIMER14

	//TIM14 initialization
	LL_TIM_DisableIT_UPDATE(TIM15); //Enable interrupt generation when timer goes to max value and UPDATE event flag is set
	LL_TIM_ClearFlag_UPDATE(TIM15); //Clear update flag on TIMER14

	//TIM3 initialization of encoder
	HAL_TIM_Encoder_Start_IT(&htim3, TIM_CHANNEL_ALL);
	__HAL_TIM_SET_COUNTER(&htim3, 30000); //write non 0 value to avoid shift from 0 -> max value
	encoderVal = __HAL_TIM_GET_COUNTER(&htim3)/4;
	//encoderValPrev = encoderVal;

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
	//CDC_Transmit_FS(data, strlen(data));*/
	// Process button events
	processSystemEvents(&stateMachine, &systemEvents);

	// Run the state machine
	//runStateMachine(&stateMachine, &SNK_data);
	runStateMachine();

	// Reset button states after processing
	stateMachine.outputBtnPressed = false;
	stateMachine.lockBtnPressed = false;
	stateMachine.voltageCurrentBtnPressed = false;
	stateMachine.rotaryBtnPressed = false;
	stateMachine.encoderTurnedFlag = false;
}

/*
 * Define Callbacks and ISR
 */
//BTN ISR to set event flags
void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == SW1_TOGGLE_I_V_Pin) {
        systemEvents.voltageCurrentBtnEvent = true;
        EXTI->IMR1 &= ~(EXTI_IMR1_IM2);
    } /*else if (GPIO_Pin == SW2_DEBUG_BTN_Pin) {
        systemEvents.lockBtnEvent = true;
        EXTI->IMR1 &= ~(EXTI_IMR1_IM4);
        int indexSRCAPDO = USER_SERV_FindSRCIndex(0, &powerRequestDetails, dhandle->voltageSet*10, dhandle->currentSet, dhandle ->selMethod);
		//Print to debug
		char _str[80];
		sprintf(_str,"APDO request: indexSRCPDO= %lu, VBUS= %lu mV, Ibus= %d mA", indexSRCAPDO, 10*dhandle->voltageSet, dhandle->currentSet);
		USBPD_TRACE_Add(USBPD_TRACE_DEBUG, 0, 0, (uint8_t*)_str, strlen(_str));
		//USBPD_DPM_RequestSRCPDO(0, indexSRCAPDO, dhandle->voltageSet*10, dhandle->currentSet);
		USBPD_DPM_RequestSRCPDO(0, 6, 6000, 1000);
        //lockButtonPressTime = HAL_GetTick();
    } */else if (GPIO_Pin == SW3_OFF_ON_Pin) {
        systemEvents.outputBtnEvent = true;
        EXTI->IMR1 &= ~(EXTI_IMR1_IM1);
    } else if (GPIO_Pin == ENC_TOGGLE_UNITS_Pin) {
        systemEvents.rotaryBtnEvent = true;
        EXTI->IMR1 &= ~(EXTI_IMR1_IM8);
    }

    // Start debounce timer
    TIM7->ARR = 200;
    LL_TIM_SetCounter(TIM7, 0);
    LL_TIM_EnableCounter(TIM7);
}

//TIM capture callback
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM3) {
    	systemEvents.encoderTurnEvent = true;
    }
}

void TIM14_ISR(void) {
	if (LL_TIM_IsActiveFlag_UPDATE(TIM14)) {
		// Clear the update interrupt flag
		LL_TIM_ClearFlag_UPDATE(TIM14);

		if (stateMachine.currentState == STATE_ACTIVE || stateMachine.currentState == STATE_SET_VALUES) {
			// Handle periodic check for ACTIVE state
			systemEvents.periodicCheckEvent = true;
			//Reset CNT value
			LL_TIM_SetCounter(TIM14, 0);

			//Start timer again
			LL_TIM_EnableCounter(TIM14);
		}
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


		if (stateMachine.currentState == STATE_SET_VALUES) {

			//Make a USBPD request
			int indexSRCAPDO = USER_SERV_FindSRCIndex(0, &powerRequestDetails, dhandle->voltageSet*10, dhandle->currentSet, dhandle ->selMethod);
			//Print to debug

			char _str[70];
			sprintf(_str,"APDO request: indexSRCPDO= %int, VBUS= %lu mV, Ibus= %lu mA", indexSRCAPDO, 10*dhandle->voltageSet, dhandle->currentSet);
			USBPD_TRACE_Add(USBPD_TRACE_DEBUG, 0, 0, (uint8_t*)_str, strlen(_str));
			USBPD_DPM_RequestSRCPDO(0, indexSRCAPDO, dhandle->voltageSet*10, dhandle->currentSet);
		}

	}
}

/*
 * Define Process functions
 */
// Process button events in the main loop (Convert hardware events into logical events)
void processButtonEvents(StateMachine *sm, SystemEvents *events) {
    if (events->outputBtnEvent) {
    	//Reset btn event flag
        events->outputBtnEvent = false;
        sm->outputBtnPressed = true;
    } else if (events->voltageCurrentBtnEvent) {
    	events->voltageCurrentBtnEvent = false;
    	sm->voltageCurrentBtnPressed = true;
    } else if (events->lockBtnEvent) {
    	events->lockBtnEvent = false;
		sm->lockBtnPressed = true;
    } else if (events->rotaryBtnEvent) {
    	//Reset btn event flag
    	events->rotaryBtnEvent = false;
		sm->rotaryBtnPressed = true;

    }
}

// Main system event processor that calls specialized handlers
void processSystemEvents(StateMachine *sm, SystemEvents *events) {
    // Process different event types using specialized functions
    processButtonEvents(sm, events);

    if (events->encoderTurnEvent) {
    	sm->encoderTurnedFlag = true;
    	events->encoderTurnEvent = false;
    }
    // Process AWDG events
    if (events->awdgEvent) {
        //events->awdgEvent = false;
        //sm->awdgTriggered = true;
    }

    // Process timer events
    /*
    if (events->timer1Event) {
        events->timer1Event = false;
        // Handle timer1 event
    }*/

}

/*
 * Define runStateMachine
 */
/*void runStateMachine(StateMachine *sm, SINKData_HandleTypeDef *dhandle) {
    // Process events and transitions
    switch (sm->currentState) {
        case STATE_OFF:
            //handleOffState(sm, dhandle);
            break;
        case STATE_INIT:
            handleInitState(sm, dhandle);
            break;
        case STATE_IDLE:
            handleIdleState(sm, dhandle);
            break;
        case STATE_ACTIVE:
            handleActiveState(sm, dhandle);
            break;
        case STATE_LOCK:
            //handleLockState(sm, dhandle);
            break;
        case STATE_ERROR:
            //handleErrorState(sm, dhandle);
            break;
        case STATE_OCP_TOGGLE:
            //handleOCPToggleState(sm, dhandle);
            break;
        case STATE_SET_VALUES:
            handleSetValuesState(sm, dhandle);
            break;
        default:
            // Error handling
            sm->currentState = STATE_ERROR;
            //sm->errorCode = ERROR_INVALID_STATE;
            break;
    }

}*/

/*
 * Define state Handle functions
 */
void handleInitState(StateMachine *sm, SINKData_HandleTypeDef *dhandle) {
    // Entry actions (if just entered this state)
    static bool entryDone = false;

    //=======================================================
	// ENTRY ACTIONS - Executed once when entering the state
	//=======================================================
    if (!entryDone) {
    	// Set the state entry time and timeout duration
		sm->stateEntryTime = HAL_GetTick();
		sm->timeoutCounter = 2000;  // 2 seconds timeout

    	//Show SRC limits on displays
        //max7219_PrintIspecial(SEGMENT_2, dhandle->currentSet, 4);
        //max7219_PrintIspecial(SEGMENT_1, dhandle->voltageSet, 3);


        // Save state for return from temporary states
        strcpy(sm->lastStateStr, "INIT");
        sm->lastState = STATE_INIT;

        entryDone = true;
    }

    //==========================================================
	// DO ACTIONS - Executed every time the state is processed
	//==========================================================

    // Check if the timeout has elapsed
    if (HAL_GetTick() - sm->stateEntryTime > sm->timeoutCounter) {
		//After initialization transition to IDLE state
		sm->currentState = STATE_IDLE;
		entryDone = false;
    }

}

void handleIdleState(StateMachine *sm, SINKData_HandleTypeDef *dhandle) {
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
    } else if (sm->lockBtnHoldActive) {
        sm->currentState = STATE_LOCK;
        entryDone = false;
    } else if (sm->ocpBtnPressed) {
        sm->currentState = STATE_OCP_TOGGLE;
        sm->stateEntryTime = HAL_GetTick();
        sm->timeoutCounter = 500;  // 0.5 seconds timeout
        entryDone = false;
    } else if (sm->rotaryBtnPressed) {
        sm->currentState = STATE_SET_VALUES;
        sm->timeoutCounter = 4000;  // 4 seconds timeout
        sm->rotaryBtnPressed = false;
        entryDone = false;
    }
}

void handleActiveState(StateMachine *sm, SINKData_HandleTypeDef *dhandle) {
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

        // Initialize the check timer
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
    if (systemEvents.periodicCheckEvent) {
        systemEvents.periodicCheckEvent = false;
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
    } else if (sm->lockBtnHoldActive) {
        sm->currentState = STATE_LOCK;
        entryDone = false;
    } else if (sm->ocpBtnPressed) {
        sm->currentState = STATE_OCP_TOGGLE;
        sm->stateEntryTime = HAL_GetTick();
        sm->timeoutCounter = 500;  // 0.5 seconds timeout
        entryDone = false;
    } else if (sm->rotaryBtnPressed) {
        sm->currentState = STATE_SET_VALUES;
        sm->timeoutCounter = 4000;  // 4 seconds timeout
        sm->rotaryBtnPressed = false;
        entryDone = false;
    }

    if (sm->outputBtnPressed || sm->ocpBtnPressed) {
    	LL_TIM_DisableCounter(TIM14);
    }
}

void handleSetValuesState(StateMachine *sm, SINKData_HandleTypeDef *dhandle) {
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

		// Initialize the periodic timer
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
		sm->encoderTurnedFlag = false;

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
    if (systemEvents.periodicCheckEvent) {
    	if (sm->setValueMode == SET_VOLTAGE) {
			max7219_BlinkDigit2(SEGMENT_1, dhandle->voltageSet, sm->encoder.selDigit, 3, showDigit);
			max7219_PrintIspecial(SEGMENT_2, dhandle->currentSet,4);
		} else { // SET_CURRENT
			max7219_BlinkDigit2(SEGMENT_2, dhandle->currentSet, sm->encoder.selDigit, 4, showDigit);
			max7219_PrintIspecial(SEGMENT_1, dhandle->voltageSet,3);
		}
    	//Toggle showDigit
    	showDigit = !showDigit;
    	//Erase periodic check flag
    	systemEvents.periodicCheckEvent = false;
    }

    // On turnEvent update voltage/current
	if (sm->encoder.turnEvent) {
		//Reset event flag
		sm->encoder.turnEvent = false;
		//Update displays
		switch (sm->setValueMode) {
			case SET_VOLTAGE:
				//updateVoltage(sm,dhandle);
				int voltageTemp = dhandle->voltageSet;
				voltageTemp += sm->encoder.direction * sm->encoder.increment;

				//If required temp value is within limits, assign it to voltage else assign limits
				if (voltageTemp > dhandle->voltageMax) {
					dhandle->voltageSet = dhandle->voltageMax;

				} else if (voltageTemp < dhandle->voltageMin) {
					dhandle->voltageSet = dhandle->voltageMin;

				} else {
					dhandle->voltageSet = voltageTemp;
				}

				//Print selected voltage to disp, decimal at digit 3
				max7219_PrintIspecial(SEGMENT_1, dhandle->voltageSet, 3);
				break;
			case SET_CURRENT:
				//updateCurrent(sm,dhandle);
				break;
		}
	}


	//=================================================
	// TRANSITION CHECKS - Check for state transitions
	//=================================================
    // Process events and transitions
	if (sm->lockBtnHoldActive) {
        sm->currentState = STATE_LOCK;
        entryDone = false;
    } else if (sm->ocpBtnPressed) {
        sm->currentState = STATE_OCP_TOGGLE;
        sm->stateEntryTime = HAL_GetTick();
        sm->timeoutCounter = 500;  // 0.5 seconds timeout
        entryDone = false;
    } else if (systemEvents.stateTimeoutEvent) {
	    //Reset timeout event flag
    	systemEvents.stateTimeoutEvent = false;

    	//Return to last state
	    if (sm->lastState == STATE_IDLE) {
	        sm->currentState = STATE_IDLE;
	    } else if (sm->lastState == STATE_ACTIVE) {
	        sm->currentState = STATE_ACTIVE;
	    }
	    entryDone = false;

    }
}

/**
 * Update ADC CH3 AWD Treshold
 * Possibility to update parameters on the fly (read more in HAL_ADC_AnalogWDGConfig declaration)
 * Full config and AWD init in main.c
 */
void Update_AWD_Thresholds(uint32_t low, uint32_t high) {
	// Just update the thresholds for an already configured AWD
	ADC_AnalogWDGConfTypeDef AnalogWDGConfig = {0};
	AnalogWDGConfig.WatchdogNumber = ADC_ANALOGWATCHDOG_2; // Specify which AWD you're updating
	AnalogWDGConfig.HighThreshold = high;
	AnalogWDGConfig.LowThreshold = low;

	if (HAL_ADC_AnalogWDGConfig(&hadc1, &AnalogWDGConfig) != HAL_OK)
	{
	    Error_Handler();
	}
}

// Helper function to update voltage
void updateCurrentOCP(SINKData_HandleTypeDef *handle) {
	//Get direction of encoder turning
	int currentTemp = handle->currentOCPSet;
	currentTemp += handle->encoder.direction * handle->encoder.increment;

	//If required temp value is within limits, assign it to voltage
	if ( (handle->currentMin <= currentTemp) && (currentTemp <= (handle->currentMax + 300)) ) {
		handle->currentOCPSet = currentTemp;
	} else {
		//currentOCPTemp = currentOCP;
	}

	int V_TRIP = (handle->currentOCPSet * R_OCP_MOHMS * G_OCP)/1000; // mV (mA * mOhms * Gain)
	//Convert DAC_OUT voltage to 12B resolution
	int dac_value = (V_TRIP *4095) / VDDA_APPLI;//__LL_ADC_DIGITAL_SCALE(LL_ADC_RESOLUTION_12B);
	//Write output with DAC..
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac_value);

	//Print selected voltage to disp, decimal at digit 3
	max7219_PrintIspecial(SEGMENT_2, handle->currentOCPSet, 4);

	//Print to debug
	char _str[40];
	sprintf(_str,"IOCP selected: %lu mA", handle->currentOCPSet);
	USBPD_TRACE_Add(USBPD_TRACE_DEBUG, 0, 0, (uint8_t*)_str, strlen(_str));

}

/**
 * TIM2 encoder turning interrupt service routine
 */
/*
void encoder_turn_isr(void) {
	//Get the TIM3 (encoder) value from CNT register
	encoderVal = (TIM3 -> CNT) >> 2;

	dhandle->encoder.curValue= encoderVal;

	if (encoderVal != dhandle->encoder.prevValue){

		dhandle->encoder.direction = (encoderVal < dhandle->encoder.prevValue) ? 1 : -1;

		switch(currentState)
		{
		case ADJUSTMENT_VOLTAGE:
		{
			//updateVoltage(dhandle);
		}
		break;

		case ADJUSTMENT_CURRENT:
		{
			//updateCurrent(dhandle);

		}
		break;

		case ADJUSTMENT_CURRENT_OCP:
		{
			//updateCurrentOCP(dhandle);
		}
		break;

		}

		//Save TIM2 CNT value to ValPrev
		dhandle->encoder.prevValue = encoderVal;
	}
}
*/

/**
 * Button interrupt service routine
 */
/*
void enc_toggle_units_isr(void){

	//const char response[] = "POWER is ON\r\n";
	       // LPUART_Transmit(LPUART2, (const uint8_t*)response, sizeof(response) - 1);

	//Mask unwanted button interrupts caused by debouncing on exti line 3 (PD8)
	EXTI->IMR1 &= ~(EXTI_IMR1_IM8);

	//Set debouncing time in ms
	TIM7->ARR = 200;

	//Zero TIM7 counter and start counting
	LL_TIM_SetCounter(TIM7, 0); //set counter register value of timer 7 to 0
	LL_TIM_EnableCounter(TIM7); //start counting of timer 7

	//Decrement encoderPress value if higher than 4
	if (dhandle->encoder.selDigit > 1){
		dhandle->encoder.selDigit--;
	}
	else {
		dhandle->encoder.selDigit = 4;
	}

	//Choose addition value based on encoderPress val and current ADJUSTMENT_STATE (voltage/current)
	int val;
	switch (currentState){
		case ADJUSTMENT_VOLTAGE:
			switch (dhandle->encoder.selDigit) {
			case 1: val = 2; break;
			case 2: val = 10; break;
			case 3: val = 100; break;
			case 4: val = 1000; break;
			}
		 break;
		case ADJUSTMENT_CURRENT_OCP:
		case ADJUSTMENT_CURRENT:
			switch (dhandle->encoder.selDigit) {
			case 1: val = 5; break;
			case 2: val = 10; break;
			case 3: val = 100; break;
			case 4: val = 1000; break;
			}
		 break;
	}

	dhandle->encoder.increment = val;

	char _str[60];
	uint32_t voltageADC = BSP_PWR_VBUSGetVoltage(0);
	uint32_t currentADC= BSP_PWR_VBUSGetCurrent(0);
	uint32_t currentOCP_ADC= BSP_PWR_VBUSGetCurrentOCP(0);

	// Use snprintf to limit the number of characters written
	int len = snprintf(_str, sizeof(_str), "VBUS:%lu mV, IBUS:%lu mA, IOCP:%lu mA", voltageADC, currentADC, currentOCP_ADC);

	USBPD_TRACE_Add(USBPD_TRACE_DEBUG, 0, 0, (uint8_t*)_str, strlen(_str));

	//Erase btn (PC3) interrupt flag
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_3);

}*/

/*
 * Timer7 interrupt routine for button debouncing
 */
void tim7_btn_isr(void){
	//Unmask exti line 1, 2 and 3
	EXTI->IMR1 |= EXTI_IMR1_IM8; //unmask interrupt mask register on exti line 8
	EXTI->IMR1 |= EXTI_IMR1_IM4; //unmask interrupt mask register on exti line 4
	EXTI->IMR1 |= EXTI_IMR1_IM2; //unmask interrupt mask register on exti line 2
	EXTI->IMR1 |= EXTI_IMR1_IM1; //unmask interrupt mask register on exti line 1

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


	int indexSRCAPDO = USER_SERV_FindSRCIndex(0, &powerRequestDetails, dhandle->voltageSet*10, dhandle->currentSet, dhandle ->selMethod);
	//Print to debug
	char _str[70];
	sprintf(_str,"APDO request: indexSRCPDO= %int, VBUS= %lu mV, Ibus= %lu mA", indexSRCAPDO, 10*dhandle->voltageSet, dhandle->currentSet);
	USBPD_TRACE_Add(USBPD_TRACE_DEBUG, 0, 0, (uint8_t*)_str, strlen(_str));
	USBPD_DPM_RequestSRCPDO(0, indexSRCAPDO, dhandle->voltageSet*10, dhandle->currentSet);
	//HAL_Delay(2);
	//HAL_GPIO_WritePin(OCP_ALERT_GPIO_Port, OCP_RESET_Pin, GPIO_PIN_SET);
}

/*
 * Change between current and voltage ADJUSTMENT_STATE
 */
void sw1_toggle_i_v_isr(void){
	//Mask unwanted button interrupts caused by debouncing on exti line 2 (PB2)
	//EXTI->IMR1 &= ~(EXTI_IMR1_IM2);

	//Set debouncing time in ms
	//TIM7->ARR = 200;

	//Zero TIM7 counter and start counting
	//LL_TIM_SetCounter(TIM7, 0); //set counter register value of timer 7 to 0
	//LL_TIM_EnableCounter(TIM7); //start counting of timer 7

	// Toggle the state
	if (currentState == ADJUSTMENT_CURRENT_OCP)
	{
		currentState = ADJUSTMENT_VOLTAGE;
	}
	else if (currentState == ADJUSTMENT_VOLTAGE)
	{
		currentState = ADJUSTMENT_CURRENT;
		//Display output current
		max7219_PrintIspecial(SEGMENT_2, dhandle->currentSet, 4);
	}
	else
	{
		currentState = ADJUSTMENT_CURRENT_OCP;
		//Display output current
		max7219_PrintIspecial(SEGMENT_2, dhandle->currentOCPSet, 4);
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
				max7219_PrintIspecial(SEGMENT_1,dhandle->voltageSet, 3);
				//Display current
				max7219_PrintIspecial(SEGMENT_2, dhandle->currentSet, 4);
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
	max7219_PrintIspecial(SEGMENT_1, dhandle->voltageSet, 3);

	//Display output current
	max7219_PrintIspecial(SEGMENT_2, dhandle->currentSet, 4);


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
		if (maxcurrent > dhandle->currentMax) {
			  			dhandle -> currentMax = (int)maxcurrent+100;
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
			dhandle -> currentMax = (int)maxcurrent+100;
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



