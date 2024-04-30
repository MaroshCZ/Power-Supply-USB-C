/*
 * app.h
 *
 *  Created on: Apr 29, 2024
 *      Author: Jan Mareš
 */

#ifndef APP_H_
#define APP_H_

//declaration of functions
void app_init(void);
void app_loop(void);

//declaration of isr
void button_isr(void);
void button_timer_isr(void);

#endif /* APP_H_ */
