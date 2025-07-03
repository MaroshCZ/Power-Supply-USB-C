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

typedef enum {
	UNKNOWN,
	FIXED,
	APDO
}USBPD_Profile_Type_TypeDef;

typedef struct {
	uint32_t  					voltageMin;
	uint32_t  					voltageMax;
	uint32_t 					currentMax;
	USBPD_Profile_Type_TypeDef  profileType; // UNKNOWN/FIXED/APDO
}USBPD_Profiles_TypeDef;

typedef struct {
  //USBPD_PPSSDB_TypeDef  DPM_RcvPPSStatus;           /*!< PPS Status received by port partner                         */
  //USBPD_SKEDB_TypeDef   DPM_RcvSNKExtendedCapa;     /*!< SNK Extended Capability received by port partner            */
  uint32_t            	    voltageSet;       /*!< User selected voltage in centivolts */
  uint32_t              	currentSet;       /*!< User selected OCP limit in mA */
  uint32_t					currentOCPSet;
  uint32_t					awdgTresholdSet;

  uint32_t             		voltageMeas;      /*!< Measured output voltage in mV */
  uint32_t              	currentMeas;      /*!< Measured output current in mAmps */
  uint32_t					voltageMeasCorrected;

  int32_t					voltageError;
  uint32_t					correctedRequestVoltage;

  uint32_t              	voltageMin;       /*!< Minimal SRC voltage in centivolts */
  uint32_t              	voltageMax;       /*!< Maximal SRC voltage in centivolts */
  uint32_t              	currentMax;       /*!< Maximal SRC current in mA */
  uint32_t              	currentMin;       /*!< Minimal current in mA (0)*/
  USBPD_USER_SERV_PDO_SelectionMethodTypeDef selMethod; /*!< Minimal current in mA (0)*/
  USBPD_Profiles_TypeDef 	srcProfiles[7];   /*!< Struct holding profiles data, max of 7 profiles according to USB PD specification*/
  uint8_t 					numProfiles;      /*!< Number of avaible profiles on adapter (source)*/
  uint8_t					selectedProfile;  /*!< Index of selected profile*/
  bool 						hasAPDO;          /*!< hasAPDO flag*/

  bool 						requestOngoing;

} SINKData_HandleTypeDef;

#define G_OCP          	   100u // V/V
#define R_OCP_MOHMS     	 5u // 5 mOhms
#define R_A         	200000u // 200 kOhms
#define R_B         	 36000u // 36 kOhms
#define R_SENSE_MOHMS  		30u // 30 mOhms
#define G_SENSE             20u // V/V
#define OCP_DISABLED_HT   3908u // high treshold [mA] 5250mA*G_OCP*R_SENSE/1000 * 4095/3300 = 3908
#define ADC_MAX_VALUE 	  4095u // ADC resolution
#define DEBOUNCE_TIME_MS    50u // btn debounce time
#define ENCODER_INITIAL_VALUE 30000u
#define SEGMENT_DISP_INTENSIVITY 7u

#define ADC_NUM_OF_SAMPLES 3
extern ADC_HandleTypeDef hadc1;
extern DAC_HandleTypeDef hdac1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;


/* Variable containing ADC conversions results
   aADCxConvertedValues[0u]: VSENSE
   aADCxConvertedValues[1u]: ISENSE
*/

// Raw values converted by ADC
extern __IO uint16_t aADCxConvertedValues[ADC_NUM_OF_SAMPLES];

//__IO: Indicates that this variable can change at any time, usually due to hardware activity.

// Definition of PPS states
typedef enum {
    STATE_OFF,
    STATE_INIT,
    STATE_IDLE,
    STATE_ACTIVE,
    STATE_ERROR,
    STATE_SET_VALUES,
    // Sub-states can be handled within each state function
} SystemState_TypeDef;

// Definition of PPS states
typedef enum {
    STATE_OPEN,
    STATE_CLOSED
} COMPortState_TypeDef;


// Definition of state machine struct
typedef struct {
    // Current and last system state
    SystemState_TypeDef currentState;
    SystemState_TypeDef lastState;
    COMPortState_TypeDef comState;

    // State timers and counters
    uint32_t stateEntryTime;
    uint32_t timeoutCounter;

    // Button states
    bool outputBtnPressed;
    bool lockBtnPressed;
    bool lockBtnLongPressed;
    bool voltageCurrentBtnPressed;
    bool voltageCurrentBtnLongPressed;
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

	// Lock mode
	enum {
		LOCKED,
		UNLOCKED
	} lockMode;

	enum {
		LED_ON,
		LED_OFF,
		LED_BLINK_SLOW,
		LED_BLINK_FAST,
	} lockLedState;

	// PWR mode
	enum {
		MODE_FIXED,
		MODE_APDO
	} pwrMode;

    // Set value states
    enum {
        SET_VOLTAGE,
        SET_CURRENT
    } setValueMode;

    // Error flags
    uint8_t errorCode;
    bool errorActive;

} StateMachine_TypeDef;

// Button event flags
typedef struct {
	// Button events
	volatile bool outputBtnEvent;
	volatile bool lockBtnEvent;
	volatile bool lockBtnLongEvent;
	volatile bool voltageCurrentBtnEvent;
	volatile bool voltageCurrentBtnLongEvent;
	volatile bool rotaryBtnEvent;
	volatile bool btnPressEvent;

	// Encoder events
	volatile bool encoderTurnEvent;

	// Periodic check event
	volatile bool periodicCheckEvent;
	volatile bool stateTimeoutEvent;

	// ADC/AWDG events
	volatile bool awdgEvent;
} SystemEvents_TypeDef;

// Button press times
typedef struct {
	volatile uint32_t voltageCurrentBtn;
	volatile uint32_t lockBtn;
} BtnPressTimes_TypeDef;

// Declaration of main app functions
void app_init(void);
void app_loop(void);

// Declaration of ISR
void encoder_turn_isr(void);
void enc_toggle_units_isr(void);
void button_timer_isr(void);
void timer14_isr(void);
void tim7_btn_isr(void);
void sw1_toggle_i_v_isr(void);
void sw3_on_off_isr(void);
void sw2_lock_isr(void);
void ocp_alert_isr(void);

void TIM7_ISR(void);
void TIM14_ISR(void);
void TIM15_ISR(void);


/*Define State functions*/
void handleOffState(void); //or: StateMachine_TypeDef *sm, SINKData_HandleTypeDef *dhandle
void handleInitState(void);
void handleIdleState(void);
void handleActiveState(void);
void handleErrorState(void);
void handleSetValuesState(void);

/*Define procces functions and Statemachine*/
void runStateMachine(void); //StateMachine *sm, SINKData_HandleTypeDef *dhandle
void processButtonEvents(void);
void processSystemEvents(void);
void processUSBCommand(uint8_t* command, uint32_t length);
void processLockLedState(void);


/*Define additional fcns*/
void sourcecapa_limits(bool printToCOM);
void updateVoltage(void);
void updateCurrent(void);
void updateCurrentOCP(void);
uint32_t compensateVoltage(uint32_t measuredVoltage);
int32_t roundToNearest20mV(int32_t valueInMv);
int32_t correctCurrentMeas(uint32_t measuredCurrent);
int32_t correctVoltageMeas(uint32_t measuredVoltage, uint32_t measuredCurrent);
void correctOutputVoltage(void);
void Update_AWD_Thresholds(uint32_t low, uint32_t high, uint32_t adc_watchdog);
void updateLockLED(void);
SINKData_HandleTypeDef *getSNK_data(void);

/*Define helper functions for SCPI communication with PC*/
uint8_t* getUSBbuffer(void);
void handleCOMportstatus(uint8_t host_com_port_open);
void cleanString(const char* input, char* output, const char* delimiter);
#endif /* APP_H_ */
