/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "stm32l0xx_hal.h"

#include "stm32l0xx_ll_iwdg.h"
#include "stm32l0xx_ll_lptim.h"
#include "stm32l0xx_ll_spi.h"
#include "stm32l0xx_ll_usart.h"
#include "stm32l0xx_ll_rcc.h"
#include "stm32l0xx_ll_system.h"
#include "stm32l0xx_ll_gpio.h"
#include "stm32l0xx_ll_exti.h"
#include "stm32l0xx_ll_bus.h"
#include "stm32l0xx_ll_cortex.h"
#include "stm32l0xx_ll_utils.h"
#include "stm32l0xx_ll_pwr.h"
#include "stm32l0xx_ll_dma.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct
{
	uint32_t deviceSerial0;
	uint32_t deviceSerial1;
	uint32_t deviceSerial2;
	uint8_t deviceCode1;
	uint8_t deviceCode2;
	uint8_t deviceCode3;
	uint8_t deviceCode4;
	uint8_t deviceCode5;
	uint8_t deviceCode6;
	uint8_t deviceCode7;
	uint8_t deviceCode8;
	uint8_t deviceCode9;
	uint8_t deviceCode10;
	uint8_t deviceCode11;
	uint8_t deviceCode12;
} device_t;

extern device_t device;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define EEPROM_START_ADDR   0x08080000   /* Start @ of user eeprom area */
#define PI 3.1415926
#define SEND_S1LENGTH    	16
#define SEND_S2LENGTH    	17
#define SEND_S3LENGTH    	24
#define SEND_S5LENGTH    	20
#define SEND_S7LENGTH    	25
#define SEND_LLENGTH     	97
#define SEND_LENGTH     	180
#define RECV_LENGTH   		24

#define	Time_Delay				500		// cc1101 tx wait time
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define _DEBUG              0        //  set device info
#define _NBIOT_DEBUG        0        //  use printf debug

#if (_NBIOT_DEBUG == 1)
#define rfid_printf(...)     			printf(__VA_ARGS__)
#else
#define rfid_printf(...)     			{};
#endif

static uint8_t SendBuffer[SEND_LENGTH] = {0};
static uint8_t RecvBuffer[RECV_LENGTH] = {0};

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SPI1_CS_Pin LL_GPIO_PIN_2
#define SPI1_CS_GPIO_Port GPIOA
#define GDO0_Pin LL_GPIO_PIN_3
#define GDO0_GPIO_Port GPIOA
#define GDO0_EXTI_IRQn EXTI2_3_IRQn
#define GDO2_Pin LL_GPIO_PIN_4
#define GDO2_GPIO_Port GPIOA
#define GDO2_EXTI_IRQn EXTI4_15_IRQn
#define INT1_Pin LL_GPIO_PIN_0
#define INT1_GPIO_Port GPIOB
#define INT1_EXTI_IRQn EXTI0_1_IRQn
#define INT2_Pin LL_GPIO_PIN_1
#define INT2_GPIO_Port GPIOB
#define INT2_EXTI_IRQn EXTI0_1_IRQn
#define SPI2_CS_Pin LL_GPIO_PIN_12
#define SPI2_CS_GPIO_Port GPIOB
#define LED_GREEN_Pin LL_GPIO_PIN_4
#define LED_GREEN_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
void System_Initial(void);
void Get_SerialNum(void);
void Show_Message(void);
uint8_t RF_RecvHandler(void);
void RF_SendPacket(uint8_t index);
void Package_Array(void);
void Set_DeviceInfo(void);
void DATAEEPROM_Program(uint32_t Address, uint32_t Data);
uint32_t DATAEEPROM_Read(uint32_t Address);
void LED_Blinking(uint32_t Period);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/