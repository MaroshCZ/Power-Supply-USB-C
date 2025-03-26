/*
 * demo_app.h
 *
 *  Created on: Jun 25, 2024
 *      Author: Jan Mareš
 */

#ifndef DEMO_APP_H_
#define DEMO_APP_H_

/*Includes*/
#include "app.h"
#include "stdio.h"
#include "stdbool.h"

/* Exported constants --------------------------------------------------------*/
typedef enum{
     DEMO_OK,
     DEMO_ERROR
} DEMO_ErrorCode;

// Definition of PPS states
typedef enum {
    STATE_OFF,
    STATE_INIT,
    STATE_IDLE,
    STATE_ACTIVE,
    STATE_LOCK,
    STATE_ERROR,
    //STATE_DISPLAY_TOGGLE,
    STATE_OCP_TOGGLE,
    STATE_SET_VALUES,
    // Sub-states can be handled within each state function
} SystemState;


//Definition of encoder data struct
typedef struct {
	int curValue;      // Current encoder value
	int prevValue;     // Previous encoder value
	int selDigit;  	   // Currently selected digit
	int increment;     // Current increment value
	int direction;	   // Direction: 1 for clockwise, -1 for counter-clockwise
}Encoder_TypeDef;

// Definition of state machine struct
typedef struct {
    // Current and last system state
    SystemState currentState;
    SystemState lastState;

    // State timers and counters
    uint32_t stateEntryTime;
    uint32_t timeoutCounter;

    // Button states
    bool outputBtnPressed;
    bool lockBtnPressed;
    bool lockBtnHoldActive;
    bool ocpBtnPressed;
    bool voltageCurrentBtnPressed;
    //bool showBtnPressed;
    bool rotaryBtnPressed;

    // Other flags
    bool encoderTurnedFlag;

    // Encoder data
    Encoder_TypeDef encoder;

    // Display states
    enum {
        SHOW_LIMITS,
        SHOW_SET,
        SHOW_MEASURED
    } displayMode;

    // Set value states
    enum {
        SET_VOLTAGE,
        SET_CURRENT
    } setValueMode;

    // OCP state
    bool ocpEnabled;

    // Error flags
    uint8_t errorCode;
    bool errorActive;

    // Last state for returning from special modes
    char lastStateStr[10];

} StateMachine;

// Button management structure to hold all button-related state
typedef struct {
    // Button press tracking
    uint32_t buttonPressTimes[BTN_COUNT];
    bool buttonStates[BTN_COUNT];
    uint32_t currentTime;
    uint32_t pressDuration;

    // Button debounce state
    bool debounceActive;
} ButtonManager;

// System event flags
typedef struct {
	// Button events
	    volatile bool outputBtnEvent;
	    volatile bool lockBtnEvent;
	    volatile bool lockBtnLongEvent;
	    volatile bool voltageCurrentBtnEvent;
	    volatile bool ocpBtnEvent;
	    volatile bool rotaryBtnEvent;

	    // Encoder events
	    volatile bool encoderTurnEvent;

	    // Periodic check event
	    volatile bool periodicCheckEvent;
	    volatile bool stateTimeoutEvent;

	    // ADC/AWDG events
	    volatile bool awdgEvent;
} SystemEvents;


/*Function definition*/
void handleOffState(StateMachine *sm, SINKData_HandleTypeDef *dhandle);
void handleInitState(StateMachine *sm, SINKData_HandleTypeDef *dhandle);
void handleIdleState(StateMachine *sm, SINKData_HandleTypeDef *dhandle);
void handleActiveState(StateMachine *sm, SINKData_HandleTypeDef *dhandle);
void handleLockState(StateMachine *sm, SINKData_HandleTypeDef *dhandle);
void handleErrorState(StateMachine *sm, SINKData_HandleTypeDef *dhandle);
void handleDisplayToggleState(StateMachine *sm, SINKData_HandleTypeDef *dhandle);
void handleOCPToggleState(StateMachine *sm, SINKData_HandleTypeDef *dhandle);
void handleSetValuesState(StateMachine *sm, SINKData_HandleTypeDef *dhandle);

/*State machine*/
void runStateMachine(StateMachine *sm, SINKData_HandleTypeDef *dhandle);
void processButtonEvents(StateMachine *sm, SystemEvents *events);
void processSystemEvents(StateMachine *sm, SystemEvents *events);
void updateVoltage(StateMachine *sm, SINKData_HandleTypeDef *handle);
void updateCurrent(StateMachine *sm, SINKData_HandleTypeDef *handle);
void TIM14_ISR(void);
void demo_app_loop(void);

extern ADC_HandleTypeDef hadc1;
extern DAC_HandleTypeDef hdac1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

/*Exported variables*/
extern StateMachine stateMachine;
extern SystemEvents systemEvents;

#endif /* DEMO_APP_H_ */
