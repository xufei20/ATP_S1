/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#include "usart.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
#define bit(n) (1 << n)

#define TOP 0
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
typedef union u16_u8
{
    uint16_t data_u16;
    uint8_t data_u8[2];
}u16_u8_t;

typedef union u32_u8
{
    uint32_t u32t;
    uint8_t u8t[4];
}u32_u8_t;

typedef union float2uint8
{
    float f;
    uint8_t u8t[4];
}f32_u8_t;

typedef union short2uint8
{
    short s;
    uint8_t u8t[2];
}s16_u8_t;
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED1_Pin GPIO_PIN_8
#define LED1_GPIO_Port GPIOI
#define RF_PWD_Pin GPIO_PIN_9
#define RF_PWD_GPIO_Port GPIOF
#define LED2_Pin GPIO_PIN_3
#define LED2_GPIO_Port GPIOG

/* USER CODE BEGIN Private defines */
int uart_printf(const char *fmt, ...);

void DelayMs(uint32_t ms);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
