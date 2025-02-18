/*
 * demo_app.h
 *
 *  Created on: Jun 25, 2024
 *      Author: Jan Mareš
 */

#ifndef DEMO_APP_H_
#define DEMO_APP_H_

/* Exported constants --------------------------------------------------------*/
typedef enum{
     DEMO_OK,
     DEMO_ERROR
} DEMO_ErrorCode;

/*Exported functions*/
DEMO_ErrorCode DEMO_Init(void);
DEMO_ErrorCode DEMO_InitTask(void);

#endif /* DEMO_APP_H_ */
