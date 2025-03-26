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

/*
#define G_OCP          100u // V/V
#define R_OCP_MOHMS      5u // 5 mOhms
#define R_A         200000u // 200 kOhms
#define R_B          36000u // 36 kOhms
#define R_SENSE_MOHMS   30u // 30 mOhms
#define G_SENSE         20u // V/V*/

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

//void sourcecapa_limits(void);


//#define ADC_NUM_OF_SAMPLES 3
extern ADC_HandleTypeDef hadc1;
extern DAC_HandleTypeDef hdac1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;


/* Variable containing ADC conversions results
   aADCxConvertedValues[0u]: VSENSE
   aADCxConvertedValues[1u]: ISENSE
*/
//extern __IO uint16_t aADCxConvertedValues[ADC_NUM_OF_SAMPLES];


//__IO: Indicates that this variable can change at any time, usually due to hardware activity.

void usart2_lupart2_handler(void);


#endif /* APP_H_ */
