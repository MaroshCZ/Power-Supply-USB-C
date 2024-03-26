/*
 * app_encoder.h
 *
 *  Created on: Mar 26, 2024
 *      Author: Jan Mareš
 */

#ifndef APP_ENCODER_H_
#define APP_ENCODER_H_

//declaration of functions
void app_loop(void);
void app_encoder_init(void);

//declaration of isr
void button_isr(void);
void button_timer_isr(void);

#endif /* APP_ENCODER_H_ */
