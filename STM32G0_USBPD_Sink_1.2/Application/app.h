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


static void sourcecapa_limits(void);


#endif /* APP_H_ */
