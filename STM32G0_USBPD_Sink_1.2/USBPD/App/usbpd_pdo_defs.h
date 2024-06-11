/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usbpd_pdo_defs.h
  * @author  MCD Application Team
  * @brief   Header file for definition of PDO/APDO values for 2 ports(DRP/SNK) configuration
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __USBPD_PDO_DEF_H_
#define __USBPD_PDO_DEF_H_

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usbpd_def.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Define   ------------------------------------------------------------------*/
#define PORT0_NB_SOURCEPDO         0U   /* Number of Source PDOs (applicable for port 0)   */
#define PORT0_NB_SINKPDO           3U   /* Number of Sink PDOs (applicable for port 0)     */
#define PORT1_NB_SOURCEPDO         0U   /* Number of Source PDOs (applicable for port 1)   */
#define PORT1_NB_SINKPDO           0U   /* Number of Sink PDOs (applicable for port 1)     */

/* USER CODE BEGIN Define */

/* USER CODE END Define */

/* Exported typedef ----------------------------------------------------------*/

/* USER CODE BEGIN typedef */

/**
  * @brief  USBPD Port PDO Structure definition
  *
  */

/* USER CODE END typedef */

/* Exported define -----------------------------------------------------------*/

/* USER CODE BEGIN Exported_Define */

#define USBPD_CORE_PDO_SRC_FIXED_MAX_CURRENT 3
#define USBPD_CORE_PDO_SNK_FIXED_MAX_CURRENT 1500

 /* Definitions for APDO values */
 /* PDP = 7.5W */
 #define USBPD_PDO_APDO_5VPROG_7P5W_MAX_CURRENT 1.5 /* Max Current in A (PDP / 5) */

 /* PDP = 15W */
 #define USBPD_PDO_APDO_5VPROG_15W_MAX_CURRENT  3   /* Max Current in A (PDP / 5) */

 /* PDP = 45W */
 #define USBPD_PDO_APDO_9VPROG_45W_MAX_CURRENT  3   /* Max Current in A */
 #define USBPD_PDO_APDO_15VPROG_45W_MAX_CURRENT 3   /* Max Current in A (PDP / 5) */

 /* PDP = 100W*/
 #define USBPD_PDO_APDO_20VPROG_100W_MAX_CURRENT 5   /* Max Current in A (PDP / 5) */

 /* Programmable Power Supply Voltage Ranges */
 #define USBPD_PDO_APDO_5VPROG_MIN_VOLTAGE 3.3   /* Min voltage in V */
 #define USBPD_PDO_APDO_5VPROG_MAX_VOLTAGE 5.9 /* Max voltage in V */
 #define USBPD_PDO_APDO_9VPROG_MIN_VOLTAGE 3.3   /* Min voltage in V */
 #define USBPD_PDO_APDO_9VPROG_MAX_VOLTAGE 11  /* Max voltage in V */
 #define USBPD_PDO_APDO_15VPROG_MIN_VOLTAGE 3.3   /* Min voltage in V */
 #define USBPD_PDO_APDO_15VPROG_MAX_VOLTAGE 16  /* Max voltage in V */
 #define USBPD_PDO_APDO_20VPROG_MIN_VOLTAGE 3.3   /* Min voltage in V */
 #define USBPD_PDO_APDO_20VPROG_MAX_VOLTAGE 20  /* Max voltage in V */

/* USER CODE END Exported_Define */

/* Exported constants --------------------------------------------------------*/

/* USER CODE BEGIN constants */

/* USER CODE END constants */

/* Exported macro ------------------------------------------------------------*/

/* USER CODE BEGIN macro */

/* USER CODE END macro */

/* Exported variables --------------------------------------------------------*/

/* USER CODE BEGIN variables */

/* USER CODE END variables */

#ifndef __USBPD_PWR_IF_C
extern uint8_t USBPD_NbPDO[4];
extern uint32_t PORT0_PDO_ListSRC[USBPD_MAX_NB_PDO];
extern uint32_t PORT0_PDO_ListSNK[USBPD_MAX_NB_PDO];
#else
uint8_t USBPD_NbPDO[4] = {(PORT0_NB_SINKPDO),
                          (PORT0_NB_SOURCEPDO)};
/* Definition of Source PDO for Port 0 */
uint32_t PORT0_PDO_ListSRC[USBPD_MAX_NB_PDO] =
{
  /* PDO 1 */
        (0x00000000U),
  /* PDO 2 */
        (0x00000000U),
  /* PDO 3 */
        (0x00000000U),
  /* PDO 4 */
        (0x00000000U),
  /* PDO 5 */
        (0x00000000U),
  /* PDO 6 */
        (0x00000000U),
  /* PDO 7 */
        (0x00000000U)
};

/* Definition of Sink PDO for Port 0 */
uint32_t PORT0_PDO_ListSNK[USBPD_MAX_NB_PDO] =
{
  /* PDO 1 */
        (0x00019096U),
  /* PDO 2 */
        (0x0002D096U),
  /* PDO 3 */
        (0x0603C096U),
  /* PDO 4 */
        (0x00000000U),
  /* PDO 5 */
        (0x00000000U),
  /* PDO 6 */
        (0x00000000U),
  /* PDO 7 */
        (0x00000000U)
};

#endif

/* Exported functions --------------------------------------------------------*/

/* USER CODE BEGIN functions */

/* USER CODE END functions */

#ifdef __cplusplus
}
#endif

#endif /* __USBPD_PDO_DEF_H_ */
