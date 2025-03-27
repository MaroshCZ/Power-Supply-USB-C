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

#define G_OCP          100u // V/V
#define R_OCP_MOHMS      5u // 5 mOhms
#define R_A         200000u // 200 kOhms
#define R_B          36000u // 36 kOhms
#define R_SENSE_MOHMS   30u // 30 mOhms
#define G_SENSE         20u // V/V

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

// Declare external global variable (but do not define it here)
extern SINKData_HandleTypeDef SNK_data;
extern SINKData_HandleTypeDef *dhandle;
//get_dhandle(SINKData_HandleTypeDef *)

//__IO: Indicates that this variable can change at any time, usually due to hardware activity.

void usart2_lupart2_handler(void);


#endif /* APP_H_ */
