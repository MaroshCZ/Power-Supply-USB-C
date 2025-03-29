/*
 * app.h
 *
 *  Created on: Apr 29, 2024
 *      Author: Jan Mareš
 */

#ifndef APP_H_
#define APP_H_

#include "usbpd_user_services.h"
#include "stdbool.h"
//
typedef enum
{
  CHANGE_VOLTAGE,
  CHANGE_CURRENT,
  CHANGE_OCP
} USBPD_USER_SERV_StateTypeDef;


typedef struct {
	int curValue;      // Current encoder value
	int prevValue;     // Previous encoder value
	int selDigit;  	   // Currently selected digit
	int increment;     // Current increment value
	int direction;	   // Direction: 1 for clockwise, -1 for counter-clockwise
	bool turnEvent;	   // Turn event
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

#define G_OCP          	   100u // V/V
#define R_OCP_MOHMS     	 5u // 5 mOhms
#define R_A         	200000u // 200 kOhms
#define R_B         	 36000u // 36 kOhms
#define R_SENSE_MOHMS  		30u // 30 mOhms
#define G_SENSE             20u // V/V
#define OCP_DISABLED_HT   6000u // high treshold [mA]

//declaration of functions
void app_init(void);
void app_loop(void);

//declaration of isr
void encoder_turn_isr(void);
void enc_toggle_units_isr(void);
void button_timer_isr(void);
void timer14_isr(void);
void tim7_btn_isr(void);
void sw1_toggle_i_v_isr(void);
void sw3_on_off_isr(void);
void sw2_lock_isr(void);
void ocp_alert_isr(void);

void sourcecapa_limits(void);

#define ADC_NUM_OF_SAMPLES 3
extern ADC_HandleTypeDef hadc1;
extern DAC_HandleTypeDef hdac1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;


/* Variable containing ADC conversions results
   aADCxConvertedValues[0u]: VSENSE
   aADCxConvertedValues[1u]: ISENSE
*/
extern __IO uint16_t aADCxConvertedValues[ADC_NUM_OF_SAMPLES];
//get_dhandle(SINKData_HandleTypeDef *)

//__IO: Indicates that this variable can change at any time, usually due to hardware activity.

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
    bool rotaryBtnPressed;

    // Other flags
    bool encoderTurnedFlag;
    bool stateTimeoutFlag;
    bool periodicCheckFlag;
    bool awdgTriggeredFlag;

    // Encoder data
    Encoder_TypeDef encoder;

    // Display states
	enum {
		OCP_DISABLED,
		OCP_ENABLED
	} OCPMode;

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

// Button event flags
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



void usart2_lupart2_handler(void);

/*Define State functions*/
void handleOffState(StateMachine *sm, SINKData_HandleTypeDef *dhandle);
void handleInitState(void);
void handleIdleState(void);
void handleActiveState(void);
void handleLockState(StateMachine *sm, SINKData_HandleTypeDef *dhandle);
void handleErrorState(StateMachine *sm, SINKData_HandleTypeDef *dhandle);
void handleDisplayToggleState(StateMachine *sm, SINKData_HandleTypeDef *dhandle);
void handleOCPToggleState(StateMachine *sm, SINKData_HandleTypeDef *dhandle);
void handleSetValuesState(void);

/*Define procces functions and Statemachine*/
void runStateMachine(void); //StateMachine *sm, SINKData_HandleTypeDef *dhandle
void processButtonEvents(void);
void processSystemEvents(void);

/*Define additional fcns and ISR*/
void updateCurrentOCP(void);
void Update_AWD_Thresholds(uint32_t low, uint32_t high, uint32_t adc_watchdog);
void TIM14_ISR(void);

#endif /* APP_H_ */
