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
#endif /* _GUI_INTERFACE */


//**//
//Initialize button event struct
ButtonEvents buttonEvents = {0};




void runStateMachine(StateMachine *sm, SINKData_HandleTypeDef *dhandle) {
    // Process events and transitions
    switch (sm->currentState) {
        case STATE_OFF:
            handleOffState(sm, dhandle);
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
            handleLockState(sm, dhandle);
            break;
        case STATE_ERROR:
            handleErrorState(sm, dhandle);
            break;
        case STATE_OCP_TOGGLE:
            handleOCPToggleState(sm, dhandle);
            break;
        case STATE_SET_VALUES:
            handleSetValuesState(sm, dhandle);
            break;
        default:
            // Error handling
            sm->currentState = STATE_ERROR;
            sm->errorCode = ERROR_INVALID_STATE;
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
        buttonEvents.voltageCurrentBtnEvent = true;
    } else if (GPIO_Pin == SW2_DEBUG_BTN_Pin) {
        buttonEvents.lockBtnEvent = true;
        lockButtonPressTime = HAL_GetTick();
    } else if (GPIO_Pin == SW3_OFF_ON_Pin) {
        buttonEvents.outputBtnEvent = true;
    } else if (GPIO_Pin == ENC_TOGGLE_UNITS_Pin) {
        buttonEvents.rotaryBtnEvent = true;
    }

    // Start debounce timer
    EXTI->IMR1 &= ~(EXTI_IMR1_IM2);
    TIM7->ARR = 200;
    LL_TIM_SetCounter(TIM7, 0);
    LL_TIM_EnableCounter(TIM7);
}

// Process button events in the main loop (Convert hardware events into logical events)
void processButtonEvents(StateMachine *sm, ButtonEvents *events) {
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


void handleInitState(StateMachine *sm, SINKData_HandleTypeDef *dhandle) {
    // Entry actions (if just entered this state)
    static bool entryDone = false;
    if (!entryDone) {
        // Do initialiyation tasks

    	//Show SRC limits on displays
        //max7219_PrintIspecial(SEGMENT_2, dhandle->currentSet, 4);
        //max7219_PrintIspecial(SEGMENT_1, dhandle->voltageSet, 3);


        // Save state for return from temporary states
        strcpy(sm->lastStateStr, "INIT");

        entryDone = true;
    }

	sm->currentState = STATE_IDLE;
	entryDone = false;

}

void handleSetValuesState(StateMachine *sm, SINKData_HandleTypeDef *dhandle) {
    // Entry actions (if just entered this state)
    static bool entryDone = false;
    if (!entryDone) {

    	// Initialize the state
		sm->stateEntryTime = HAL_GetTick();
		sm->timeoutCounter = 4000;  // 4 second timeout

    	// Display set values
        max7219_PrintIspecial(SEGMENT_2, dhandle->currentSet, 4);
        max7219_PrintIspecial(SEGMENT_1, dhandle->voltageSet, 3);

        entryDone = true;
    }

    // Handle digit blinking based on current set mode
        if (sm->setValueMode == SET_VOLTAGE) {
            max7219_BlinkDigit(SEGMENT_1, &dhandle->voltageSet, sm->selectedDigit, 500, 3);
        } else { // SET_CURRENT
            max7219_BlinkDigit(SEGMENT_2, &dhandle->currentSet, sm->selectedDigit, 500, 4);
        }
	switch (sm->displayMode) {
			case SET_VOLTAGE:
				updateVoltage(dhandle);
				break;
			case SET_CURRENT:
				updateCurrent(dhandle);
				break;

	switch (sm->setValueMode) {
			case SET_VOLTAGE:
				updateVoltage(dhandle);
				break;
			case SET_CURRENT:
				updateCurrent(dhandle);
				break;

    // Process events and transitions
    if (sm->rotaryBtnPressed) {
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
    }
}
