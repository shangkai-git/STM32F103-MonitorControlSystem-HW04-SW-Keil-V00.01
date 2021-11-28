/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f1xx_hal.h"

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
#define DIGITAL5_IN_Pin GPIO_PIN_2
#define DIGITAL5_IN_GPIO_Port GPIOE
#define DIGITAL4_IN_Pin GPIO_PIN_3
#define DIGITAL4_IN_GPIO_Port GPIOE
#define DIGITAL3_IN_Pin GPIO_PIN_4
#define DIGITAL3_IN_GPIO_Port GPIOE
#define DIGITAL2_IN_Pin GPIO_PIN_5
#define DIGITAL2_IN_GPIO_Port GPIOE
#define DIGITAL1_IN_Pin GPIO_PIN_6
#define DIGITAL1_IN_GPIO_Port GPIOE
#define NRF_CE_Pin GPIO_PIN_13
#define NRF_CE_GPIO_Port GPIOC
#define NRF_CS_Pin GPIO_PIN_14
#define NRF_CS_GPIO_Port GPIOC
#define VBAT_SW_Pin GPIO_PIN_1
#define VBAT_SW_GPIO_Port GPIOC
#define VCC_SW_Pin GPIO_PIN_2
#define VCC_SW_GPIO_Port GPIOC
#define MCU_BAT_AN_Pin GPIO_PIN_3
#define MCU_BAT_AN_GPIO_Port GPIOC
#define WAKE_UP_IN_Pin GPIO_PIN_0
#define WAKE_UP_IN_GPIO_Port GPIOA
#define CURRENT4_AN_Pin GPIO_PIN_1
#define CURRENT4_AN_GPIO_Port GPIOA
#define CURRENT3_AN_Pin GPIO_PIN_2
#define CURRENT3_AN_GPIO_Port GPIOA
#define CURRENT2_AN_Pin GPIO_PIN_3
#define CURRENT2_AN_GPIO_Port GPIOA
#define CURRENT1_AN_Pin GPIO_PIN_4
#define CURRENT1_AN_GPIO_Port GPIOA
#define VOLTAGE4_AN_Pin GPIO_PIN_5
#define VOLTAGE4_AN_GPIO_Port GPIOA
#define VOLTAGE3_AN_Pin GPIO_PIN_6
#define VOLTAGE3_AN_GPIO_Port GPIOA
#define VOLTAGE2_AN_Pin GPIO_PIN_7
#define VOLTAGE2_AN_GPIO_Port GPIOA
#define VOLTAGE1_AN_Pin GPIO_PIN_4
#define VOLTAGE1_AN_GPIO_Port GPIOC
#define RELAY12_AN_Pin GPIO_PIN_5
#define RELAY12_AN_GPIO_Port GPIOC
#define HC4851_AN_Pin GPIO_PIN_0
#define HC4851_AN_GPIO_Port GPIOB
#define RELAY4_OUT_Pin GPIO_PIN_1
#define RELAY4_OUT_GPIO_Port GPIOB
#define RELAY3_OUT_Pin GPIO_PIN_7
#define RELAY3_OUT_GPIO_Port GPIOE
#define RELAY7_OUT_Pin GPIO_PIN_8
#define RELAY7_OUT_GPIO_Port GPIOE
#define RELAY6_OUT_Pin GPIO_PIN_9
#define RELAY6_OUT_GPIO_Port GPIOE
#define RELAY5_OUT_Pin GPIO_PIN_10
#define RELAY5_OUT_GPIO_Port GPIOE
#define RELAY1_OUT_Pin GPIO_PIN_11
#define RELAY1_OUT_GPIO_Port GPIOE
#define RELAY2_OUT_Pin GPIO_PIN_12
#define RELAY2_OUT_GPIO_Port GPIOE
#define SW1_IN_Pin GPIO_PIN_13
#define SW1_IN_GPIO_Port GPIOE
#define SW2_IN_Pin GPIO_PIN_14
#define SW2_IN_GPIO_Port GPIOE
#define SW3_IN_Pin GPIO_PIN_15
#define SW3_IN_GPIO_Port GPIOE
#define SW4_IN_Pin GPIO_PIN_10
#define SW4_IN_GPIO_Port GPIOB
#define SW5_IN_Pin GPIO_PIN_11
#define SW5_IN_GPIO_Port GPIOB
#define OLED_CS_Pin GPIO_PIN_12
#define OLED_CS_GPIO_Port GPIOB
#define OLED_DC_Pin GPIO_PIN_8
#define OLED_DC_GPIO_Port GPIOD
#define OLED_RST_Pin GPIO_PIN_9
#define OLED_RST_GPIO_Port GPIOD
#define HC4851_A_Pin GPIO_PIN_10
#define HC4851_A_GPIO_Port GPIOD
#define HC4851_B_Pin GPIO_PIN_11
#define HC4851_B_GPIO_Port GPIOD
#define HC4851_C_Pin GPIO_PIN_12
#define HC4851_C_GPIO_Port GPIOD
#define LSD3_OUT_Pin GPIO_PIN_13
#define LSD3_OUT_GPIO_Port GPIOD
#define LSD2_OUT_Pin GPIO_PIN_14
#define LSD2_OUT_GPIO_Port GPIOD
#define LSD1_OUT_Pin GPIO_PIN_15
#define LSD1_OUT_GPIO_Port GPIOD
#define PWM4_IN_Pin GPIO_PIN_7
#define PWM4_IN_GPIO_Port GPIOC
#define PWM3_IN_Pin GPIO_PIN_8
#define PWM3_IN_GPIO_Port GPIOC
#define PWM2_IN_Pin GPIO_PIN_9
#define PWM2_IN_GPIO_Port GPIOC
#define PWM1_IN_Pin GPIO_PIN_8
#define PWM1_IN_GPIO_Port GPIOA
#define DS18B20_Pin GPIO_PIN_9
#define DS18B20_GPIO_Port GPIOA
#define CAN_STB_Pin GPIO_PIN_10
#define CAN_STB_GPIO_Port GPIOA
#define HSD4_OUT_Pin GPIO_PIN_15
#define HSD4_OUT_GPIO_Port GPIOA
#define HSD3_OUT_Pin GPIO_PIN_10
#define HSD3_OUT_GPIO_Port GPIOC
#define HSD2_OUT_Pin GPIO_PIN_11
#define HSD2_OUT_GPIO_Port GPIOC
#define HSD1_OUT_Pin GPIO_PIN_12
#define HSD1_OUT_GPIO_Port GPIOC
#define UART2_TX_Pin GPIO_PIN_5
#define UART2_TX_GPIO_Port GPIOD
#define UART2_RX_Pin GPIO_PIN_6
#define UART2_RX_GPIO_Port GPIOD
#define NRF_IRQ_Pin GPIO_PIN_7
#define NRF_IRQ_GPIO_Port GPIOD
#define I2C_SCL_Pin GPIO_PIN_6
#define I2C_SCL_GPIO_Port GPIOB
#define I2C_SDA_Pin GPIO_PIN_7
#define I2C_SDA_GPIO_Port GPIOB
#define LED4_OUT_Pin GPIO_PIN_8
#define LED4_OUT_GPIO_Port GPIOB
#define LED3_OUT_Pin GPIO_PIN_9
#define LED3_OUT_GPIO_Port GPIOB
#define LED2_OUT_Pin GPIO_PIN_0
#define LED2_OUT_GPIO_Port GPIOE
#define LED1_OUT_Pin GPIO_PIN_1
#define LED1_OUT_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
