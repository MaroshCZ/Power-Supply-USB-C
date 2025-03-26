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

#define G_OCP          100u // V/V
#define R_OCP_MOHMS      5u // 5 mOhms
#define R_A         200000u // 200 kOhms
#define R_B          36000u // 36 kOhms
#define R_SENSE_MOHMS   30u // 30 mOhms
#define G_SENSE         20u // V/V

// Add these to your existing definitions
#define BTN_VOLTAGE_CURRENT 0
#define BTN_LOCK            1
#define BTN_OUTPUT          2
#define BTN_ROTARY          3
#define BTN_COUNT           4  // Total number of buttons

#define LONG_PRESS_THRESHOLD  800  // ms threshold for long press

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
	int turnEvent;
}Encoder_TypeDef;

typedef struct {
  //USBPD_PPSSDB_TypeDef  DPM_RcvPPSStatus;           /*!< PPS Status received by port partner                         */
  //USBPD_SKEDB_TypeDef   DPM_RcvSNKExtendedCapa;     /*!< SNK Extended Capability received by port partner            */
  uint32_t              voltageSet;       /*!< User selected voltage in centivolts */
  uint32_t              currentSet;       /*!< User selected OCP limit in mA */
  uint32_t				currentOCPSet;

  uint32_t              voltageMeas;      /*!< Measured output voltage in centivolts */
  uint32_t              currentMeas;      /*!< Measured output current in centivolts */

  uint32_t              voltageMin;       /*!< Minimal SRC voltage in centivolts */
  uint32_t              voltageMax;       /*!< Maximal SRC voltage in centivolts */
  uint32_t              currentMax;       /*!< Maximal SRC current in mA */
  uint32_t              currentMin;       /*!< Minimal current in mA (0)*/
  USBPD_USER_SERV_PDO_SelectionMethodTypeDef selMethod;
  Encoder_TypeDef       encoder;

} SINKData_HandleTypeDef;

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

#define BTN_COUNT 4
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
void TIM15_ISR(void);
void TIM7_ISR(void);
void demo_app_loop(void);
void sourcecapa_limits(void);

extern ADC_HandleTypeDef hadc1;
extern DAC_HandleTypeDef hdac1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

#define ADC_NUM_OF_SAMPLES 3
extern __IO uint16_t aADCxConvertedValues[ADC_NUM_OF_SAMPLES];

// Declare external global variable (but do not define it here)
extern SINKData_HandleTypeDef SNK_data;
extern SINKData_HandleTypeDef *dhandle;

/*Exported variables*/
extern StateMachine stateMachine;
extern SystemEvents systemEvents;

#endif /* DEMO_APP_H_ */
