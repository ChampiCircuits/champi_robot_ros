/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define STEP_LEFT_Pin GPIO_PIN_6
#define STEP_LEFT_GPIO_Port GPIOE
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define DIR_RIGHT_Pin GPIO_PIN_0
#define DIR_RIGHT_GPIO_Port GPIOF
#define DIR_REAR_Pin GPIO_PIN_1
#define DIR_REAR_GPIO_Port GPIOF
#define BAU_Pin GPIO_PIN_3
#define BAU_GPIO_Port GPIOF
#define STEP_OPT1_Pin GPIO_PIN_6
#define STEP_OPT1_GPIO_Port GPIOF
#define STEP_OPT0_Pin GPIO_PIN_7
#define STEP_OPT0_GPIO_Port GPIOF
#define STEP_REAR_Pin GPIO_PIN_8
#define STEP_REAR_GPIO_Port GPIOF
#define STEP_RIGHT_Pin GPIO_PIN_9
#define STEP_RIGHT_GPIO_Port GPIOF
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOH
#define DIR_OPT0_Pin GPIO_PIN_0
#define DIR_OPT0_GPIO_Port GPIOA
#define XSHUT0_Pin GPIO_PIN_7
#define XSHUT0_GPIO_Port GPIOA
#define XSHUT3_Pin GPIO_PIN_2
#define XSHUT3_GPIO_Port GPIOB
#define D4_Pin GPIO_PIN_11
#define D4_GPIO_Port GPIOF
#define D7_Pin GPIO_PIN_12
#define D7_GPIO_Port GPIOF
#define TIRETTE_Pin GPIO_PIN_15
#define TIRETTE_GPIO_Port GPIOF
#define SPI4_SS0_Pin GPIO_PIN_10
#define SPI4_SS0_GPIO_Port GPIOE
#define LED_RED_Pin GPIO_PIN_14
#define LED_RED_GPIO_Port GPIOB
#define STLK_VCP_RX_Pin GPIO_PIN_8
#define STLK_VCP_RX_GPIO_Port GPIOD
#define STLK_VCP_TX_Pin GPIO_PIN_9
#define STLK_VCP_TX_GPIO_Port GPIOD
#define USB_FS_PWR_EN_Pin GPIO_PIN_10
#define USB_FS_PWR_EN_GPIO_Port GPIOD
#define BUZZER_Pin GPIO_PIN_12
#define BUZZER_GPIO_Port GPIOD
#define D5_Pin GPIO_PIN_4
#define D5_GPIO_Port GPIOG
#define D1_Pin GPIO_PIN_5
#define D1_GPIO_Port GPIOG
#define D0_Pin GPIO_PIN_6
#define D0_GPIO_Port GPIOG
#define USB_FS_OVCR_Pin GPIO_PIN_7
#define USB_FS_OVCR_GPIO_Port GPIOG
#define D2_Pin GPIO_PIN_8
#define D2_GPIO_Port GPIOG
#define LED_STRIP_Pin GPIO_PIN_6
#define LED_STRIP_GPIO_Port GPIOC
#define XSHUT2_Pin GPIO_PIN_7
#define XSHUT2_GPIO_Port GPIOC
#define USB_FS_VBUS_Pin GPIO_PIN_9
#define USB_FS_VBUS_GPIO_Port GPIOA
#define USB_FS_ID_Pin GPIO_PIN_10
#define USB_FS_ID_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define DIR_OPT1_Pin GPIO_PIN_12
#define DIR_OPT1_GPIO_Port GPIOC
#define ENABLE_STEPPERS_Pin GPIO_PIN_4
#define ENABLE_STEPPERS_GPIO_Port GPIOD
#define D6_Pin GPIO_PIN_14
#define D6_GPIO_Port GPIOG
#define DIR_LEFT_Pin GPIO_PIN_15
#define DIR_LEFT_GPIO_Port GPIOG
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define XSHUT1_Pin GPIO_PIN_6
#define XSHUT1_GPIO_Port GPIOB
#define D3_Pin GPIO_PIN_0
#define D3_GPIO_Port GPIOE
#define LED_YELLOW_Pin GPIO_PIN_1
#define LED_YELLOW_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
