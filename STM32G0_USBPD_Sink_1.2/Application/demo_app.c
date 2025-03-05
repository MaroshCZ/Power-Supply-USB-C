/*
 * demo_app.c
 *
 *  Created on: Jun 25, 2024
 *      Author: Jan Mareš
 */


/* Includes ------------------------------------------------------------------*/
#include "usbpd_core.h"
#include "usbpd_dpm_core.h"
#include "usbpd_dpm_conf.h"
#include "usbpd_dpm_user.h"
#include "usbpd_devices_conf.h"
#include "usbpd_pwr_if.h"
#include "demo_app.h"
#include "string.h"
#include "stdio.h"
#include "cmsis_os.h"
#include "max7219.h"
#if defined(_GUI_INTERFACE)
#include "gui_api.h"
#endif /* _GUI_INTERFACE */

/**
  * @brief  Demo initialisation
  * @retval DEMO_ErrorCode status
  */
DEMO_ErrorCode DEMO_Init(void)
{
  /*Initialize the 7 segment display */
  //max7219_Init( 7 );
  //max7219_Decode_On();

  /*Print initial values*/
  //max7219_PrintItos(SEGMENT_1, 4, 3300, 3);
  //max7219_PrintItos(SEGMENT_2, 4, 0, 4);

 return DEMO_OK;
}

DEMO_ErrorCode DEMO_InitTask(void)
{

}
