/*
 * app.h
 *
 *  Created on: Apr 29, 2024
 *      Author: Jan Mareš
 */

#ifndef APP_H_
#define APP_H_

//
typedef enum
{
  CHANGE_VOLTAGE,
  CHANGE_CURRENT
} USBPD_USER_SERV_StateTypeDef;

//declaration of functions
void app_init(void);
void app_loop(void);

//declaration of isr
void encoder_turn_isr(void);
void button_isr(void);
void button_timer_isr(void);
void cur_vol_button_isr(void);
void request_button_isr(void);

static void sourcecapa_limits(void);

#define ADC_NUM_OF_SAMPLES 2
extern ADC_HandleTypeDef hadc1;

/* Variable containing ADC conversions results
   aADCxConvertedValues[0u]: VSENSE
   aADCxConvertedValues[1u]: ISENSE
*/
extern __IO uint16_t aADCxConvertedValues[ADC_NUM_OF_SAMPLES];
//__IO: Indicates that this variable can change at any time, usually due to hardware activity.


#endif /* APP_H_ */
