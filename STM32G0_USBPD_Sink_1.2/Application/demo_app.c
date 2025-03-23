/*
 * demo_app.c
 *
 *  Created on: Jun 25, 2024
 *      Author: Jan Mareš
 */


/* Includes ------------------------------------------------------------------*/
#include "usbpd_core.h"
#include "usbpd_dpm_core.h"
#include "usbpd_dpm_conf.h"
#include "usbpd_dpm_user.h"
#include "usbpd_devices_conf.h"
#include "usbpd_pwr_if.h"
#include "demo_app.h"
#include "string.h"
#include "stdio.h"
#include "cmsis_os.h"
#include "max7219.h"
#if defined(_GUI_INTERFACE)
#include "gui_api.h"
#include "app.h"
#endif /* _GUI_INTERFACE */


//**//
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

void runStateMachine(StateMachine *sm, SINKData_HandleTypeDef *dhandle) {
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

    // Check for timeouts in temporary states
    if (sm->currentState == STATE_OCP_TOGGLE ||
        sm->currentState == STATE_SET_VALUES) {

        if (HAL_GetTick() - sm->stateEntryTime > sm->timeoutCounter) {
            // Return to previous state
            if (strcmp(sm->lastStateStr, "IDLE") == 0) {
                sm->currentState = STATE_IDLE;
            } else if (strcmp(sm->lastStateStr, "ACTIVE") == 0) {
                sm->currentState = STATE_ACTIVE;
            }
        }
    }
}


//BTN ISR to set event flags
void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == SW1_TOGGLE_I_V_Pin) {
        systemEvents.voltageCurrentBtnEvent = true;
        EXTI->IMR1 &= ~(EXTI_IMR1_IM2);
    } else if (GPIO_Pin == SW2_DEBUG_BTN_Pin) {
        systemEvents.lockBtnEvent = true;
        EXTI->IMR1 &= ~(EXTI_IMR1_IM4);
        //lockButtonPressTime = HAL_GetTick();
    } else if (GPIO_Pin == SW3_OFF_ON_Pin) {
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

void handleIdleState(StateMachine *sm, SINKData_HandleTypeDef *dhandle) {
    // Entry actions (if just entered this state)
    static bool entryDone = false;
    if (!entryDone) {
        // Display set values
        max7219_PrintIspecial(SEGMENT_2, dhandle->currentSet, 4);
        max7219_PrintIspecial(SEGMENT_1, dhandle->voltageSet, 3);

        // Ensure output is off
        HAL_GPIO_WritePin(RELAY_ON_OFF_GPIO_Port, RELAY_ON_OFF_Pin, GPIO_PIN_RESET);

        // Check temperature and control fan (not shown in your code)

        // Save state for return from temporary states
        strcpy(sm->lastStateStr, "IDLE");

        entryDone = true;
    }

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
        sm->stateEntryTime = HAL_GetTick();
        sm->timeoutCounter = 4000;  // 4 seconds timeout
        entryDone = false;
    }
}

void handleActiveState(StateMachine *sm, SINKData_HandleTypeDef *dhandle) {
	//Declare static variables
	// Set check interval Make the timer persistent across function calls
	static uint32_t lastCheckTime = 0; // declared to 0 only once, then retains value
    static bool entryDone = false;
	const uint32_t CHECK_INTERVAL_MS = 250; // Check every 250ms

	//=======================================================
	// ENTRY ACTIONS - Executed once when entering the state
	//=======================================================

    if (!entryDone) {
        // Ensure output is off
        HAL_GPIO_WritePin(RELAY_ON_OFF_GPIO_Port, RELAY_ON_OFF_Pin, GPIO_PIN_SET);

        // Check temperature and control fan (not shown in your code)
        int refreshTime = HAL_GetTick();
        sm->timeoutCounter = 2000;  // 2 seconds timeout

        // Save state for return from temporary states
        strcpy(sm->lastStateStr, "ACTIVE");

        entryDone = true;
    }

    //==========================================================
	// DO ACTIONS - Executed every time the state is processed
	//==========================================================

    //Periodic check to display measured values
    uint32_t currentTime = HAL_GetTick();
    if (currentTime - lastCheckTime >= CHECK_INTERVAL_MS) {
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
        sm->stateEntryTime = HAL_GetTick();
        sm->timeoutCounter = 4000;  // 4 seconds timeout
        entryDone = false;
    }
}


void handleInitState(StateMachine *sm, SINKData_HandleTypeDef *dhandle) {
    // Entry actions (if just entered this state)
    static bool entryDone = false;
    if (!entryDone) {
    	// Set the state entry time and timeout duration
		sm->stateEntryTime = HAL_GetTick();
		sm->timeoutCounter = 2000;  // 2 seconds timeout

    	//Show SRC limits on displays
        //max7219_PrintIspecial(SEGMENT_2, dhandle->currentSet, 4);
        //max7219_PrintIspecial(SEGMENT_1, dhandle->voltageSet, 3);


        // Save state for return from temporary states
        strcpy(sm->lastStateStr, "INIT");

        entryDone = true;
    }

    // Check if the timeout has elapsed
    if (HAL_GetTick() - sm->stateEntryTime > sm->timeoutCounter) {
		//After initialization transition to IDLE state
		sm->currentState = STATE_IDLE;
		entryDone = false;
    }

}

void handleSetValuesState(StateMachine *sm, SINKData_HandleTypeDef *dhandle) {
    // Entry actions (if just entered this state)
    static bool entryDone = false;
    if (!entryDone) {

    	// Display set values
        max7219_PrintIspecial(SEGMENT_2, dhandle->currentSet, 4);
        max7219_PrintIspecial(SEGMENT_1, dhandle->voltageSet, 3);

        entryDone = true;
    }
    // User interaction - reset the timeout
	if (sm->rotaryBtnPressed || sm->encoderTurnedFlag) {
		// Reset the timeout timer whenever there's user interaction
		sm->stateEntryTime = HAL_GetTick();
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

		//Choose addition value based on encoderPress val and current ADJUSTMENT_STATE (voltage/current)
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
		sm->encoder.curValue= encoderVal;

		if (encoderVal != sm->encoder.prevValue){

			sm->encoder.direction = (encoderVal < sm->encoder.prevValue) ? 1 : -1;


			//Save TIM3 CNT value to ValPrev
			sm->encoder.prevValue = encoderVal;

			//Set encoder Turn event flag
			sm->encoder.turnEvent = true;
		}
    }

    // Handle digit blinking based on current set mode
	if (sm->setValueMode == SET_VOLTAGE) {
		//max7219_BlinkDigit(SEGMENT_1, &dhandle->voltageSet, sm->encoder.selDigit, 500, 3);
	} else { // SET_CURRENT
		//max7219_BlinkDigit(SEGMENT_2, &dhandle->currentSet, sm->encoder.selDigit, 500, 4);
	}

	if (sm->encoder.turnEvent) {
		//Reset event flag
		sm->encoder.turnEvent = false;
		//Update displays
		switch (sm->setValueMode) {
			case SET_VOLTAGE:
				updateVoltage(sm,dhandle);
				break;
			case SET_CURRENT:
				updateCurrent(sm,dhandle);
				break;
		}
	}


    // Process events and transitions
	if (sm->lockBtnHoldActive) {
        sm->currentState = STATE_LOCK;
        entryDone = false;
    } else if (sm->ocpBtnPressed) {
        sm->currentState = STATE_OCP_TOGGLE;
        sm->stateEntryTime = HAL_GetTick();
        sm->timeoutCounter = 500;  // 0.5 seconds timeout
        entryDone = false;
    }
}

// Helper function to update voltage
void updateVoltage(StateMachine *sm, SINKData_HandleTypeDef *handle) {
	//Get direction of encoder turning
	int voltageTemp = handle->voltageSet;
	voltageTemp += sm->encoder.direction * sm->encoder.increment;

	//If required temp value is within limits, assign it to voltage else assign limits
	if (voltageTemp > handle->voltageMax) {
		handle->voltageSet = handle->voltageMax;

	} else if (voltageTemp < handle->voltageMin) {
		handle->voltageSet = handle->voltageMin;

	} else {
		handle->voltageSet = voltageTemp;
	}

	//Print selected voltage to disp, decimal at digit 3
	max7219_PrintIspecial(SEGMENT_1, handle->voltageSet, 3);

	//Print to debug
	char _str[40];
	sprintf(_str,"VBUS selected: %d mV", handle->voltageSet*10);
	USBPD_TRACE_Add(USBPD_TRACE_DEBUG, 0, 0, (uint8_t*)_str, strlen(_str));

}

// Helper function to update voltage
void updateCurrent(StateMachine *sm, SINKData_HandleTypeDef *handle) {
	//Get direction of encoder turning
	int currentTemp = handle->currentSet;
	currentTemp += sm->encoder.direction * sm->encoder.increment;

	//If required temp value is within limits, assign it to voltage else assign limits
	if (currentTemp > handle->currentMax) {
		handle->currentSet = handle->currentMax;

	} else if (currentTemp < handle->currentMin) {
		handle->currentSet = handle->currentMin;

	} else {
		handle->currentSet = currentTemp;
	}

	//Update AWD limits
	int isense_Vtrip_mV = (handle->currentSet *G_SENSE*R_SENSE_MOHMS)/1000; // mV  (mA * mOhms * Gain)
	int isense_rawADCtrip= (isense_Vtrip_mV *4095) / VDDA_APPLI; //value for AWD tershold
	Update_AWD_Thresholds(0, isense_rawADCtrip+100);

	//Print selected voltage to disp, decimal at digit 3
	max7219_PrintIspecial(SEGMENT_2, handle->currentSet, 4);

	//Print to debug
	char _str[40];
	sprintf(_str,"IBUS selected: %d mA", handle->currentSet);
	USBPD_TRACE_Add(USBPD_TRACE_DEBUG, 0, 0, (uint8_t*)_str, strlen(_str));
}
