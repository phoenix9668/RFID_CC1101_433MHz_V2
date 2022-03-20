/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */
usart_t usart;

#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
/* USER CODE END 0 */

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);

  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  /**USART1 GPIO Configuration
  PA9   ------> USART1_TX
  PA10   ------> USART1_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USART1 interrupt Init */
  NVIC_SetPriority(USART1_IRQn, 3);
  NVIC_EnableIRQ(USART1_IRQn);

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART1, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART1);
  LL_USART_Enable(USART1);
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/* USER CODE BEGIN 1 */
/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
int fputc(int ch,FILE *f)
{
	USART1->TDR = ch;
	while(!(USART1->ISR&USART_ISR_TXE))
	{;}
	return ch;
}

void Activate_USART1_RXIT(void)
{
	/* Polling USART initialisation */
  while((!(LL_USART_IsActiveFlag_TEACK(USART1))) || (!(LL_USART_IsActiveFlag_REACK(USART1))))
  { 
  }

  /* Enable RXNE and Error interrupts */
  LL_USART_EnableIT_RXNE(USART1);
  LL_USART_EnableIT_ERROR(USART1);
}

/**
  * @brief  Send Txt information message on USART Tx line (to PC Com port).
  * @param  None
  * @retval None
  */
void PrintInfo(uint8_t *String, uint32_t Size)
{
  uint32_t index = 0;
  uint8_t *pchar = String;
  
  /* Send characters one per one, until last char to be sent */
  for (index = 0; index < Size; index++)
  {
    /* Wait for TXE flag to be raised */
    while (!LL_USART_IsActiveFlag_TXE(USART1))
    {
    }

    /* Write character in Transmit Data register.
       TXE flag is cleared by writing data in TDR register */
    LL_USART_TransmitData8(USART1, *pchar++);
  }

  /* Wait for TC flag to be raised for last char */
  while (!LL_USART_IsActiveFlag_TC(USART1))
  {
  }
}

/**
  * @brief  Function called from USART IRQ Handler when RXNE flag is set
  *         Function is in charge of reading character received on USART RX line.
  * @param  None
  * @retval None
  */
void USART_CharReception_Callback(void)
{
  /* Read Received character. RXNE flag is cleared by reading of RDR register */
  usart.rxBuffer[usart.rxCounter++] = LL_USART_ReceiveData8(USART1);
  /* Echo received character on TX */
//  LL_USART_TransmitData8(USART1, RxBuffer[RxCounter-1]);
	/* Check if received value is corresponding to specific one : S or s */
  if (usart.rxBuffer[usart.rxCounter-2] == 0x0D && usart.rxBuffer[usart.rxCounter-1] == 0x0A)
  {
    /* Clear RxCounter : Expected character has been received */
    usart.rxCounter = 0x00;
		usart.rxState = SET;
  }
}

/**
  * @brief  Function called in case of error detected in USART IT Handler
  * @param  None
  * @retval None
  */
void Error_Callback(void)
{
  __IO uint32_t isr_reg;

  /* Disable USARTx_IRQn */
  NVIC_DisableIRQ(USART1_IRQn);
  
  /* Error handling example :
    - Read USART ISR register to identify flag that leads to IT raising
    - Perform corresponding error handling treatment according to flag
  */
  isr_reg = LL_USART_ReadReg(USART1, ISR);
  if (isr_reg & LL_USART_ISR_NE)
  {
    /* case Noise Error flag is raised : ... */
			LL_USART_ClearFlag_NE(USART1);
			LL_USART_DeInit(USART1);
			MX_USART1_UART_Init();
			LL_USART_EnableIT_RXNE(USART1);
			LL_USART_EnableIT_ERROR(USART1);
			usart.rxCounter = 0x00;
  }
	else if (isr_reg & LL_USART_ISR_FE)
  {
    /* case Noise Error flag is raised : ... */
			LL_USART_ClearFlag_FE(USART1);
			LL_USART_DeInit(USART1);
			MX_USART1_UART_Init();
			LL_USART_EnableIT_RXNE(USART1);
			LL_USART_EnableIT_ERROR(USART1);
			usart.rxCounter = 0x00;
  }
	else if (isr_reg & LL_USART_ISR_ORE)
  {
    /* case Noise Error flag is raised : ... */
			LL_USART_ClearFlag_ORE(USART1);
			LL_USART_DeInit(USART1);
			MX_USART1_UART_Init();
			LL_USART_EnableIT_RXNE(USART1);
			LL_USART_EnableIT_ERROR(USART1);
			usart.rxCounter = 0x00;
  }
	else if (isr_reg & LL_USART_ISR_PE)
  {
    /* case Noise Error flag is raised : ... */
			LL_USART_ClearFlag_PE(USART1);
			LL_USART_DeInit(USART1);
			MX_USART1_UART_Init();
			LL_USART_EnableIT_RXNE(USART1);
			LL_USART_EnableIT_ERROR(USART1);
			usart.rxCounter = 0x00;
  }
  else
  {
    /* Unexpected IT source : Set LED to Blinking mode to indicate error occurs */
			LL_USART_DeInit(USART1);
			MX_USART1_UART_Init();
			LL_USART_EnableIT_RXNE(USART1);
			LL_USART_EnableIT_ERROR(USART1);
			usart.rxCounter = 0x00;
  }
}

void MX_USART1_UART_DeInit(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  
  /* Peripheral clock disable */
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);

  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  /**LPUART1 GPIO Configuration
  PA9   ------> USART1_TX
  PA10   ------> USART1_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	LL_USART_Disable(USART1);
  LL_USART_DeInit(USART1);
}
/* USER CODE END 1 */
