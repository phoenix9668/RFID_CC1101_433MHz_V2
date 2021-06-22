/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "iwdg.h"
#include "lptim.h"
#include "rtc.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cc1101.h"
#include "adxl362.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t Device_Serial0,Device_Serial1,Device_Serial2;
uint8_t Device_Code1,Device_Code2,Device_Code3,Device_Code4,Device_Code5,Device_Code6,Device_Code7,Device_Code8,Device_Code9,Device_Code10,Device_Code11,Device_Code12;
uint8_t addrEeprom;
uint16_t syncEeprom;
uint8_t rxIndex = 0x0;
uint8_t	ChipAddr = 0;
uint8_t	RSSI = 0;
RTC_TimeTypeDef sTime = {0};
RTC_DateTypeDef sDate = {0};
uint8_t k;
//uint16_t temp = 0;

__IO uint8_t stepStage = 0;
__IO uint16_t stepNum = 0;
uint16_t step[STEP_NUM];
uint8_t batteryLow;
extern __IO uint8_t Interval;
extern __IO uint8_t ResetCC1101;

uint8_t SendBuffer[SEND_LENGTH] = {0};
uint8_t RecvBuffer[RECV_LENGTH] = {0};

extern __IO uint8_t RxBuffer[RXBUFFERSIZE];
extern __IO FlagStatus commandState;
extern __IO ITStatus rxCatch;
extern __IO ITStatus twentyMinute;
extern __IO ITStatus fourHour;
FlagStatus recvState = RESET;
__IO FlagStatus stepState = RESET;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void SystemPower_Config(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_IWDG_Init();
  MX_RTC_Init();
  MX_LPTIM1_Init();
  /* USER CODE BEGIN 2 */
  System_Initial();
  Show_Message();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		LL_IWDG_ReloadCounter(IWDG);
		rxIndex = RF_RecvHandler();
		if(rxIndex != 0x0)
		{
			RF_SendPacket(rxIndex);
		}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if(twentyMinute == SET)
		{
			step[stepStage] = stepNum;
			#ifdef DEBUG
				for(k=0; k<STEP_NUM; k++)
				{	printf("%x ",step[k]);}
				printf("\n");
				printf("stepStage = %d\n",stepStage);
			#endif
			DATAEEPROM_Program(EEPROM_START_ADDR+4*stepStage+8, (uint32_t)step[stepStage]);
			stepNum = 0;
			if(stepStage == STEPOVERFLOW)
			{	stepStage = 0;}
			else
			{	stepStage++;}
			DATAEEPROM_Program(EEPROM_START_ADDR+152, stepStage);
			GetRTC(&sTime, &sDate);
			DATAEEPROM_Program(EEPROM_START_ADDR+160, ((uint32_t)(0xFF000000 & sDate.Year<<24)+(uint32_t)(0x00FF0000 & sDate.Month<<16)+(uint32_t)(0x0000FF00 & sDate.Date<<8)+(uint32_t)(0x000000FF & sDate.WeekDay)));
			DATAEEPROM_Program(EEPROM_START_ADDR+164, ((uint32_t)(0x00FF0000 & sTime.Hours<<16)+(uint32_t)(0x0000FF00 & sTime.Minutes<<8)+(uint32_t)(0x000000FF & sTime.Seconds)));
			twentyMinute = RESET;
		}
		if(fourHour == SET)
		{
			RFIDInitial(addrEeprom, syncEeprom, IDLE_MODE);
			fourHour = RESET;
		}
		#ifdef DEBUG
			if(commandState == SET)
			{
				Set_DeviceInfo();
				commandState = RESET;
			}
		#endif
		if(stepState == SET)
		{
			HAL_Delay(1);
			stepState = RESET;
		}
		if(commandState == RESET && recvState == RESET && stepState == RESET && twentyMinute == RESET && fourHour == RESET)
		{
			MX_SPI1_DeInit();
			MX_SPI2_DeInit();
			SystemPower_Config();
			/* Enter Stop Mode */
			HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
			SystemClock_Config();
			MX_SPI1_Init();
			Activate_SPI(SPI1);
			MX_SPI2_Init();
			Activate_SPI(SPI2);
		}
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_RTC
                              |RCC_PERIPHCLK_LPTIM1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.LptimClockSelection = RCC_LPTIM1CLKSOURCE_LSI;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**
  * @brief  System Power Configuration
  *         The system Power is configured as follow :
  *            + Regulator in LP mode
  *            + VREFINT OFF, with fast wakeup enabled
  *            + HSI as SysClk after Wake Up
  *            + No IWDG
  *            + Automatic Wakeup using RTC clocked by LSI (after ~4s)
  * @param  None
  * @retval None
  */
static void SystemPower_Config(void)
{
	/* Disable Wakeup Counter */
	HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
	/* ## Setting the Wake up time ############################################*/
	/*  RTC Wakeup Interrupt Generation:
			Wakeup Time Base = (RTC_WAKEUPCLOCK_RTCCLK_DIV /(LSE or LSI))
			Wakeup Time = Wakeup Time Base * WakeUpCounter 
			= (RTC_WAKEUPCLOCK_RTCCLK_DIV /(LSE or LSI)) * WakeUpCounter
				==> WakeUpCounter = Wakeup Time / Wakeup Time Base
    
			To configure the wake up timer to 10s the WakeUpCounter is set to 0x5000 for LSE:
			RTC_WAKEUPCLOCK_RTCCLK_DIV = RTCCLK_Div16 = 16 
			Wakeup Time Base = 16 /(32.768KHz) = 0.48828125 ms
			Wakeup Time = 10s = 0.48828125ms  * WakeUpCounter
				==> WakeUpCounter = 10s/0.48828125ms = 20480 = 0x5000 */
	HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 0xA000, RTC_WAKEUPCLOCK_RTCCLK_DIV16);
	
  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* Enable Ultra low power mode */
  HAL_PWREx_EnableUltraLowPower();

  /* Enable the fast wake up from Ultra low power mode */
  HAL_PWREx_EnableFastWakeUp();
}

/**
  * @brief System Initial
  * @retval None
*/
void System_Initial(void)
{
	uint8_t i;
	/*##-1- initial all peripheral ##*/
	#ifdef DEBUG
		Activate_USART1_RXIT();
	#endif
	#ifndef DEBUG
		MX_USART1_UART_DeInit();
	#endif
	Activate_SPI(SPI1);
	Activate_SPI(SPI2);
	LPTIM1_Counter_Start_IT();
	Get_SerialNum();
	ADXL362_Init();
	/*##-2- initial CC1101 peripheral,configure it's address and sync code ##*/
	addrEeprom = (uint8_t)(0xff & DATAEEPROM_Read(EEPROM_START_ADDR)>>16);
	syncEeprom = (uint16_t)(0xffff & DATAEEPROM_Read(EEPROM_START_ADDR));
	#ifdef DEBUG
		printf("addrEeprom = %x\n",addrEeprom);
		printf("syncEeprom = %x\n",syncEeprom);
	#endif
	RFIDInitial(addrEeprom, syncEeprom, IDLE_MODE);
	
	for(i=0; i<STEP_NUM; i++)
	{
		step[i] = (uint16_t)(0x0000FFFF & DATAEEPROM_Read(EEPROM_START_ADDR+4*i+8));
	}
	stepStage = (uint8_t)(0x000000FF & DATAEEPROM_Read(EEPROM_START_ADDR+152));
	batteryLow = (uint8_t)(0x000000FF & DATAEEPROM_Read(EEPROM_START_ADDR+156));
	InitRTC(DATAEEPROM_Read(EEPROM_START_ADDR+160), DATAEEPROM_Read(EEPROM_START_ADDR+164));
	
	#ifdef DEBUG
		printf("\n");
		for(i=0; i<STEP_NUM; i++)
		{	printf("%x ",step[i]);}
		printf("\n");
		printf("stepStage = %d\n",stepStage);
	#endif
}

/**
  * @brief Get Device Unique Code
  * @retval None
*/
void Get_SerialNum(void)
{
  Device_Serial0 = *(uint32_t*)(0x1FF80050);
  Device_Serial1 = *(uint32_t*)(0x1FF80054);
  Device_Serial2 = *(uint32_t*)(0x1FF80064);
	Device_Code1 = (uint8_t)(0x000000FF & Device_Serial0>>24);
	Device_Code2 = (uint8_t)(0x000000FF & Device_Serial0>>16);
	Device_Code3 = (uint8_t)(0x000000FF & Device_Serial0>>8);
	Device_Code4 = (uint8_t)(0x000000FF & Device_Serial0);
	Device_Code5 = (uint8_t)(0x000000FF & Device_Serial1>>24);
	Device_Code6 = (uint8_t)(0x000000FF & Device_Serial1>>16);
	Device_Code7 = (uint8_t)(0x000000FF & Device_Serial1>>8);
	Device_Code8 = (uint8_t)(0x000000FF & Device_Serial1);
	Device_Code9 = (uint8_t)(0x000000FF & Device_Serial2>>24);
	Device_Code10 = (uint8_t)(0x000000FF & Device_Serial2>>16);
	Device_Code11 = (uint8_t)(0x000000FF & Device_Serial2>>8);
	Device_Code12 = (uint8_t)(0x000000FF & Device_Serial2);
}

/**
  * @brief Show Message
  * @retval None
*/
void Show_Message(void)
{
	unsigned int  ReadValueTemp;
	#ifdef DEBUG
		printf("\r\n CC1101 chip transfer program \n");
		printf(" using USART2,configuration:%d 8-N-1 \n",9600);
		printf(" when in transfer mode,the data can exceed 60 bytes!!\r\n");
	#endif
	#ifdef DEBUG
		printf("Device_Serial0 : %x\n",(unsigned int)Device_Serial0);
		printf("Device_Serial1 : %x\n",(unsigned int)Device_Serial1);
		printf("Device_Serial2 : %x\n",(unsigned int)Device_Serial2);
	#endif
	ReadValueTemp = ADXL362RegisterRead(XL362_DEVID_AD);     	//Analog Devices device ID, 0xAD
	if(ReadValueTemp == 0xAD)
	{
		LED_GREEN_ON();
		HAL_Delay(500);
	}
	#ifdef DEBUG
		printf("Analog Devices device ID: %x\n",ReadValueTemp);	 	//send via UART
	#endif
	ReadValueTemp = ADXL362RegisterRead(XL362_DEVID_MST);    	//Analog Devices MEMS device ID, 0x1D
	if(ReadValueTemp == 0x1D)
	{
		LED_GREEN_OFF();
		HAL_Delay(500);
	}
	#ifdef DEBUG
		printf("Analog Devices MEMS device ID: %x\n",ReadValueTemp);	//send via UART
	#endif
	ReadValueTemp = ADXL362RegisterRead(XL362_PARTID);       	//part ID, 0xF2
	if(ReadValueTemp == 0xF2)
	{
		LED_GREEN_ON();
		HAL_Delay(500);
	}
	#ifdef DEBUG
		printf("Part ID: %x\n",ReadValueTemp);										//send via UART
	#endif
	ReadValueTemp = ADXL362RegisterRead(XL362_REVID);       	//version ID, 0x03
	if(ReadValueTemp == 0x02 || ReadValueTemp == 0x03)
	{
		LED_GREEN_OFF();
	}
	#ifdef DEBUG
		printf("Version ID: %x\n",ReadValueTemp);									//send via UART
	#endif
}

/**
  * @brief Receive RF Single
  * @retval index
*/
uint8_t RF_RecvHandler(void)
{
	uint8_t length=0;
	int16_t rssi_dBm;
	uint8_t i;

	if(rxCatch == SET)
		{
			recvState = SET;
			HAL_Delay(1);
			#ifdef DEBUG
				printf("interrupt occur\n");
			#endif
			for (i=0; i<RECV_LENGTH; i++)   { RecvBuffer[i] = 0; } // clear array
			length = CC1101RecPacket(RecvBuffer, &ChipAddr, &RSSI);

			#ifdef DEBUG
				rssi_dBm = CC1101CalcRSSI_dBm(RSSI);
				printf("RSSI = %ddBm, length = %d, address = %d\n",rssi_dBm,length,ChipAddr);
				for(i=0; i<RECV_LENGTH; i++)
				{
					printf("%x ",RecvBuffer[i]);
				}
			#endif
				
			/* Reset transmission flag */
			rxCatch = RESET;
				
			if(length == 0)
				{
					#ifdef DEBUG
						printf("receive error or Address Filtering fail\n");
					#endif
					return 0x01;
				}
			else
				{
					if(RecvBuffer[3] == Device_Code1 && RecvBuffer[4] == Device_Code2 && RecvBuffer[5] == Device_Code3 && RecvBuffer[6] == Device_Code4 && RecvBuffer[7] == Device_Code5 && RecvBuffer[8] == Device_Code6
						&& RecvBuffer[9] == Device_Code7 && RecvBuffer[10] == Device_Code8 && RecvBuffer[11] == Device_Code9 && RecvBuffer[12] == Device_Code10 && RecvBuffer[13] == Device_Code11 && RecvBuffer[14] == Device_Code12)
						{
						if(RecvBuffer[2] == 0xC0 || RecvBuffer[2] == 0xC2 || RecvBuffer[2] == 0xC3 || RecvBuffer[2] == 0xC5 || RecvBuffer[2] == 0xC6 || RecvBuffer[2] == 0xC7)
							{return RecvBuffer[2];}
						else
							{
							#ifdef DEBUG
								printf("receive function order error\r\n");
							#endif
							return 0x03;}
							}
					else
						{
						#ifdef DEBUG
							printf("receive RFID code error\r\n");
						#endif
						return 0x02;}
				}
		}
	else	{return 0x00;}
}
/**
  * @brief Send RF Single
  * @retval None
*/
void RF_SendPacket(uint8_t index)
{
	uint32_t data;
	uint32_t dataeeprom;// 从eeprom中读出的数
	uint8_t i;
	
	#ifdef DEBUG
		LED_GREEN_ON();
	#endif
	
	switch(index)
	{
		case 0x01://receive error or Address Filtering fail
			SendBuffer[0] = RecvBuffer[0];
			SendBuffer[1] = RecvBuffer[1];
			SendBuffer[2] = 0xE0;
			Package_Array();
			SendBuffer[15] = RSSI;
		
			for(i=0; i<2; i++)
			{
				HAL_Delay(Time_Delay);
				CC1101SendPacket(SendBuffer, SEND_S1LENGTH, ADDRESS_CHECK);
			}
			break;
		case 0x02://receive RFID code error
			SendBuffer[0] = RecvBuffer[2];
			SendBuffer[1] = RecvBuffer[3];
			SendBuffer[2] = 0xE1;
			Package_Array();
			SendBuffer[15] = RSSI;
		
			for(i=0; i<2; i++)
			{
				HAL_Delay(Time_Delay);
				CC1101SendPacket(SendBuffer, SEND_S1LENGTH, ADDRESS_CHECK);
			}
			break;
		case 0x03://receive function order error
			SendBuffer[0] = RecvBuffer[0];
			SendBuffer[1] = RecvBuffer[1];
			SendBuffer[2] = 0xE2;
			Package_Array();
			SendBuffer[15] = RSSI;
		
			for(i=0; i<2; i++)
			{
				HAL_Delay(Time_Delay);
				CC1101SendPacket(SendBuffer, SEND_S1LENGTH, ADDRESS_CHECK);
			}
			break;
			
		case 0xC0:
			SendBuffer[0] = RecvBuffer[0];
			SendBuffer[1] = RecvBuffer[1];
			SendBuffer[2] = 0xD0;
			Package_Array();
		
			for(i = 0;i < STEP_NUM; i++)
			{
				SendBuffer[15+i*2] = (uint8_t)(0x00FF & step[i]>>8);
				SendBuffer[16+i*2] = (uint8_t)(0x00FF & step[i]);
			}
//			for(i = 0;i < STEP_NUM; i++)
//			{
//				if(i == stepStage)
//				{
//					temp++;
//					SendBuffer[15+i*2] = (uint8_t)(0x00FF & temp>>8);
//					SendBuffer[16+i*2] = (uint8_t)(0x00FF & temp);
//					SendBuffer[15+i*2] = i*2;
//					SendBuffer[16+i*2] = i*2+1;
//					}
//				}
			SendBuffer[87] = stepStage;
			SendBuffer[88] = batteryLow;
			
			SendBuffer[89] = sDate.Year;
			SendBuffer[90] = sDate.Month;
			SendBuffer[91] = sDate.Date;
			SendBuffer[92] = sDate.WeekDay;
			SendBuffer[93] = sTime.Hours;
			SendBuffer[94] = sTime.Minutes;
			SendBuffer[95] = sTime.Seconds;
			SendBuffer[96] = RSSI;
			
			#ifdef DEBUG
				printf("\r\n");
				for(i=0; i<SEND_LLENGTH; i++)
				{	printf("%x ",SendBuffer[i]);}
				printf("\r\n");
			#endif
			
			for(i=0; i<2; i++)
			{
				HAL_Delay(Time_Delay);
				CC1101SendPacket(SendBuffer, SEND_LLENGTH, ADDRESS_CHECK);
			}
			break;
			
		case 0xC2:
			batteryLow = 0x00;
			DATAEEPROM_Program(EEPROM_START_ADDR+160, batteryLow);
			SendBuffer[0] = RecvBuffer[0];
			SendBuffer[1] = RecvBuffer[1];
			SendBuffer[2] = 0xD2;
			Package_Array();
			batteryLow = (uint8_t)(0x000000FF & DATAEEPROM_Read(EEPROM_START_ADDR+156));
			SendBuffer[15] = batteryLow;
			SendBuffer[16] = RSSI;
		
			for(i=0; i<2; i++)
			{
				HAL_Delay(Time_Delay);
				CC1101SendPacket(SendBuffer, SEND_S2LENGTH, ADDRESS_CHECK);
			}
			break;
			
		case 0xC3:
			ADXL362_ReInit(RecvBuffer[15], RecvBuffer[16], RecvBuffer[18], RecvBuffer[19], RecvBuffer[20], RecvBuffer[21], RecvBuffer[22]);
			SendBuffer[0] = RecvBuffer[0];
			SendBuffer[1] = RecvBuffer[1];
			SendBuffer[2] = 0xD3;
			Package_Array();
			SendBuffer[15] = ADXL362RegisterRead(XL362_THRESH_ACT_H);
			SendBuffer[16] = ADXL362RegisterRead(XL362_THRESH_ACT_L);
			SendBuffer[17] = ADXL362RegisterRead(XL362_TIME_ACT);
			SendBuffer[18] = ADXL362RegisterRead(XL362_THRESH_INACT_H);
			SendBuffer[19] = ADXL362RegisterRead(XL362_THRESH_INACT_L);
			SendBuffer[20] = ADXL362RegisterRead(XL362_TIME_INACT_H);
			SendBuffer[21] = ADXL362RegisterRead(XL362_TIME_INACT_L);
			SendBuffer[22] = ADXL362RegisterRead(XL362_FILTER_CTL);
			SendBuffer[23] = RSSI;
		
			for(i=0; i<2; i++)
			{
				HAL_Delay(Time_Delay);
				CC1101SendPacket(SendBuffer, SEND_S3LENGTH, ADDRESS_CHECK);
			}
			break;
			
		case 0xC5:
			data = ((uint32_t)(0xFF000000 & RecvBuffer[15]<<24)+(uint32_t)(0x00FF0000 & RecvBuffer[16]<<16)+(uint32_t)(0x0000FF00 & RecvBuffer[17]<<8)+(uint32_t)(0x000000FF & RecvBuffer[18]));
			DATAEEPROM_Program(EEPROM_START_ADDR, data);
			SendBuffer[0] = RecvBuffer[0];
			SendBuffer[1] = RecvBuffer[1];
			SendBuffer[2] = 0xD5;
			Package_Array();
			dataeeprom = DATAEEPROM_Read(EEPROM_START_ADDR);
			SendBuffer[15] = (uint8_t)(0x000000FF & dataeeprom>>24);
			SendBuffer[16] = (uint8_t)(0x000000FF & dataeeprom>>16);
			SendBuffer[17] = (uint8_t)(0x000000FF & dataeeprom>>8);
			SendBuffer[18] = (uint8_t)(0x000000FF & dataeeprom);
			SendBuffer[19] = RSSI;
		
			for(i=0; i<2; i++)
			{
				HAL_Delay(Time_Delay);
				CC1101SendPacket(SendBuffer, SEND_S5LENGTH, ADDRESS_CHECK);
			}
			break;
			
		case 0xC6:
			for(i = 0;i < STEP_NUM; i++)
			{
				step[i] = 0;
				DATAEEPROM_Program(EEPROM_START_ADDR+4*i+8, 0x0);
			}
			stepStage = 0;
			DATAEEPROM_Program(EEPROM_START_ADDR+152, 0x0);
			SendBuffer[0] = RecvBuffer[0];
			SendBuffer[1] = RecvBuffer[1];
			SendBuffer[2] = 0xD6;
			Package_Array();
			SendBuffer[15] = RSSI;
			
			for(i=0; i<2; i++)
			{	
				HAL_Delay(Time_Delay);
				CC1101SendPacket(SendBuffer, SEND_S1LENGTH, ADDRESS_CHECK);
			}
			break;
			
		case 0xC7:
			SendBuffer[0] = RecvBuffer[0];
			SendBuffer[1] = RecvBuffer[1];
			SendBuffer[2] = 0xD7;
			Package_Array();
			
			SetRTC(RecvBuffer);
			if(RecvBuffer[22] == 0x00 || RecvBuffer[23] == 0x00)
			{	Interval = 0x0A;
				ResetCC1101 = 0x70;}
			else
			{	Interval = RecvBuffer[22];
				ResetCC1101 = RecvBuffer[23];}
			GetRTC(&sTime, &sDate);
			SendBuffer[15] = sDate.Year;
			SendBuffer[16] = sDate.Month;
			SendBuffer[17] = sDate.Date;
			SendBuffer[18] = sDate.WeekDay;
			SendBuffer[19] = sTime.Hours;
			SendBuffer[20] = sTime.Minutes;
			SendBuffer[21] = sTime.Seconds;
			SendBuffer[22] = Interval;
			SendBuffer[23] = ResetCC1101;
			SendBuffer[24] = RSSI;
			
			for(i=0; i<2; i++)
			{	
				HAL_Delay(Time_Delay);
				CC1101SendPacket(SendBuffer, SEND_S7LENGTH, ADDRESS_CHECK);
			}
			break;			
		default : break;
	}

//	for(i=0; i<SEND_LLENGTH; i++) // clear array
//	{SendBuffer[i] = 0;}
	
	CC1101SetIdle();
	CC1101WORInit();
	CC1101SetWORMode();
	#ifdef DEBUG
		LED_GREEN_OFF();
	#endif
	recvState = RESET;
}

/**
  * @brief Package_Array
  * @retval None
*/
void Package_Array(void)
{
	SendBuffer[3] = Device_Code1;
	SendBuffer[4] = Device_Code2;
	SendBuffer[5] = Device_Code3;
	SendBuffer[6] = Device_Code4;
	
	SendBuffer[7] = Device_Code5;
	SendBuffer[8] = Device_Code6;
	SendBuffer[9] = Device_Code7;
	SendBuffer[10] = Device_Code8;
	
	SendBuffer[11] = Device_Code9;
	SendBuffer[12] = Device_Code10;
	SendBuffer[13] = Device_Code11;
	SendBuffer[14] = Device_Code12;
}
	
/**
  * @brief Set_DeviceInfo
  * @retval None
*/
void Set_DeviceInfo(void)
{
  /* Set transmission flag: trasfer complete*/
	uint32_t data;
	uint8_t uart_addr_eeprom;// 从eeprom中读出的数
	uint16_t uart_sync_eeprom;

	/*##-1- Check UART receive data whether is ‘ABCD’ begin or not ###########################*/
	if(RxBuffer[0] == 0x41 && RxBuffer[1] == 0x42 && RxBuffer[2] == 0x43 && RxBuffer[3] == 0x44)//输入‘ABCD’
	{
		data = ((uint32_t)(0xFF000000 & RxBuffer[4]<<24)+(uint32_t)(0x00FF0000 & RxBuffer[5]<<16)+(uint32_t)(0x0000FF00 & RxBuffer[6]<<8)+(uint32_t)(0x000000FF & RxBuffer[7]));
		DATAEEPROM_Program(EEPROM_START_ADDR, data);
		uart_addr_eeprom = (uint8_t)(0xff & DATAEEPROM_Read(EEPROM_START_ADDR)>>16);
		uart_sync_eeprom = (uint16_t)(0xffff & DATAEEPROM_Read(EEPROM_START_ADDR));
		#ifdef DEBUG
			printf("eeprom program end\n");
			printf("addr_eeprom = %x\n",uart_addr_eeprom);
			printf("sync_eeprom = %x\n",uart_sync_eeprom);
		#endif
	}
}

/**
  * @brief DATAEEPROM WRITE
  * @retval None
*/
void DATAEEPROM_Program(uint32_t Address, uint32_t Data)
{
	/* Unlocks the data memory and FLASH_PECR register access *************/
	if(HAL_FLASHEx_DATAEEPROM_Unlock() != HAL_OK)
	{
    Error_Handler();
	}
	/* Clear FLASH error pending bits */
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_WRPERR | FLASH_FLAG_SIZERR |
							FLASH_FLAG_OPTVERR | FLASH_FLAG_RDERR |
								FLASH_FLAG_FWWERR | FLASH_FLAG_NOTZEROERR);
	/*Erase a word in data memory *************/
	if (HAL_FLASHEx_DATAEEPROM_Erase(Address) != HAL_OK)
	{
		Error_Handler();
	}
	/*Enable DATA EEPROM fixed Time programming (2*Tprog) *************/
	HAL_FLASHEx_DATAEEPROM_EnableFixedTimeProgram();
	/* Program word at a specified address *************/
	if (HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_WORD, Address, Data) != HAL_OK)
	{
		Error_Handler();
	}
	/*Disables DATA EEPROM fixed Time programming (2*Tprog) *************/
	HAL_FLASHEx_DATAEEPROM_DisableFixedTimeProgram();

	/* Locks the Data memory and FLASH_PECR register access. (recommended
     to protect the DATA_EEPROM against possible unwanted operation) *********/
	HAL_FLASHEx_DATAEEPROM_Lock();

}

/**
  * @brief DATAEEPROM READ
  * @retval DATA
*/
uint32_t DATAEEPROM_Read(uint32_t Address)
{
	return *(__IO uint32_t*)Address;
}

void LED_Blinking(uint32_t Period)
{
  /* Toggle LED2 in an infinite loop */
  while (1)
  {
    LED_GREEN_TOG();
    HAL_Delay(Period);
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
    /* Error if LED is slowly blinking (1 sec. period) */
    LED_Blinking(1000);
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
