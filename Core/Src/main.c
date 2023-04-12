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
#include "adc.h"
#include "crc.h"
#include "iwdg.h"
#include "rtc.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cc1101.h"
#include "adxl362.h"
#include "rand_bytes_gen.h"
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
uint8_t ErrorIndex;
device_t device;
uint16_t resetCnt;
uint8_t delayRand;
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
    MX_CRC_Init();
    MX_ADC_Init();
    /* USER CODE BEGIN 2 */
    System_Initial();
    Show_Message();
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        LL_IWDG_ReloadCounter(IWDG);
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */

        if(rtc.tenSecIndex == SET)
        {
            DATAEEPROM_Program(EEPROM_START_ADDR + 16, (uint32_t) rtc.tenSecTick);
            step.restArray[step.stepStage] = action_classify.rest;
            step.ingestionArray[step.stepStage] = action_classify.ingestion;
            step.movementArray[step.stepStage] = action_classify.movement;
            step.climbArray[step.stepStage] = action_classify.climb;
            step.ruminateArray[step.stepStage] = action_classify.ruminate;
            step.otherArray[step.stepStage] = action_classify.other;
            DATAEEPROM_Program(EEPROM_START_ADDR + 8, step.stepStage);
            DATAEEPROM_Program((EEPROM_START_ADDR + 0x100 + 4 * step.stepStage), (uint32_t)step.restArray[step.stepStage]);
            DATAEEPROM_Program((EEPROM_START_ADDR + 0x130 + 4 * step.stepStage), (uint32_t)step.ingestionArray[step.stepStage]);
            DATAEEPROM_Program((EEPROM_START_ADDR + 0x160 + 4 * step.stepStage), (uint32_t)step.movementArray[step.stepStage]);
            DATAEEPROM_Program((EEPROM_START_ADDR + 0x190 + 4 * step.stepStage), (uint32_t)step.climbArray[step.stepStage]);
            DATAEEPROM_Program((EEPROM_START_ADDR + 0x1C0 + 4 * step.stepStage), (uint32_t)step.ruminateArray[step.stepStage]);
            DATAEEPROM_Program((EEPROM_START_ADDR + 0x200 + 4 * step.stepStage), (uint32_t)step.otherArray[step.stepStage]);
            rtc.tenSecIndex = RESET;
        }

        if(rtc.twentyMinIndex == SET)
        {
            delayRand = rand() % 100;
            rfid_printf("delayRand = %d\n", delayRand);
            HAL_Delay(delayRand);
            memset(&action_classify, 0, sizeof(action_classify));

            for(uint8_t i = 0; i < _STEP_LOOPNUM; i++)
            {
                rfid_printf("%x ", step.movementArray[i]);
            }

            rfid_printf("\nstepStage = %d\n", step.stepStage);

            if(step.stepStage == (_STEP_LOOPNUM - 1))
            {
                step.stepStage = 0;
            }
            else
            {
                step.stepStage++;
            }

            DATAEEPROM_Program(EEPROM_START_ADDR + 8, step.stepStage);

            MX_SPI1_Init();
            adc_detect();
            RNG_Init();
            RNG_Gen();
            MX_CRC_Init();
            CC1101SendHandler();
            HAL_CRC_DeInit(&hcrc);
            MX_SPI1_DeInit();

            rtc.twentyMinIndex = RESET;
        }

        #if (_DEBUG == 1)

        if(usart.rxState == SET)
        {
            Set_DeviceInfo();
            usart.rxState = RESET;
        }

        HAL_Delay(100);
        LED_GREEN_TOG();
        #endif

        if(step.fifoOverrun == SET)
        {
            MX_SPI2_Init();
            rfid_printf("XL362_STATUS2: %x\n", ADXL362RegisterRead(XL362_STATUS));
            memset(fifo, 0, sizeof(fifo));
            LL_IWDG_ReloadCounter(IWDG);
            #if (_Original_Data_Algorithm == 0)
            ADXL362FifoRead(1024, fifo);
            ADXL362RegisterWrite(XL362_FIFO_CONTROL, 0x00);//select fifo is disabled
            ADXL362RegisterWrite(XL362_FIFO_CONTROL, 0x03);//select fifo triggered mode
            MX_SPI2_DeInit();
            ADXL362FifoProcess();
            #elif (_Original_Data_Algorithm == 1)
            ADXL362FifoRead(900, fifo);
            MX_SPI2_DeInit();
            MX_SPI1_Init();
            MX_CRC_Init();
            ADXL362FifoProcess();
            HAL_CRC_DeInit(&hcrc);
            MX_SPI1_DeInit();
            #else
            ADXL362FifoRead(900, fifo);
            MX_SPI2_DeInit();
            ADXL362FifoProcess();
            #endif
            step.fifoOverrun = RESET;
        }

        LL_IWDG_ReloadCounter(IWDG);
        #if (_DEBUG == 0)

        if(step.fifoOverrun == RESET && rtc.twentyMinIndex == RESET && rtc.tenSecIndex == RESET)
        {
            HAL_SuspendTick();
            SystemPower_Config();
            /* Enter Stop Mode */
            HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
            SystemClock_Config();
            HAL_ResumeTick();
        }

        #endif
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

    /** Configure LSE Drive Capability
    */
    HAL_PWR_EnableBkUpAccess();
    __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_HSE
                                       | RCC_OSCILLATORTYPE_LSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.LSEState = RCC_LSE_ON;
    RCC_OscInitStruct.LSIState = RCC_LSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_8;
    RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_3;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
    {
        Error_Handler();
    }

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1 | RCC_PERIPHCLK_RTC;
    PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
    PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;

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
    /* Enable Ultra low power mode */
    HAL_PWREx_EnableUltraLowPower();

    /* Enable the fast wake up from Ultra low power mode */
    HAL_PWREx_EnableFastWakeUp();

    /* Select HSI as system clock source after Wake Up from Stop mode */
    __HAL_RCC_WAKEUPSTOP_CLK_CONFIG(RCC_STOP_WAKEUPCLOCK_HSI);
}

/**
  * @brief System Initial
  * @retval None
*/
void System_Initial(void)
{
    /*##-1- initial all peripheral ##*/
    #if (_RFID_PRINT_DEBUG == 1)
    Activate_USART1_RXIT();
    #else
    MX_USART1_UART_DeInit();
    #endif
    Get_SerialNum();
    ADXL362_Init();
    /*##-2- initial CC1101 peripheral,configure it's address and sync code ##*/
		CC1101_POWER_ON();
    RFIDInitial(0xEF, 0x1234, IDLE_MODE);
    CC1101WriteCmd(CC1101_SPWD);
		CC1101_POWER_DOWN();

    resetCnt = (uint16_t)(0x0000FFFF & DATAEEPROM_Read(EEPROM_START_ADDR + 12));
    resetCnt++;
    DATAEEPROM_Program(EEPROM_START_ADDR + 12, (uint32_t)resetCnt);

    for(uint8_t i = 0; i < _STEP_LOOPNUM; i++)
    {
        step.restArray[i] = (uint16_t)(0x0000FFFF & DATAEEPROM_Read(EEPROM_START_ADDR + 0x100 + 4 * i));
    }

    for(uint8_t i = 0; i < _STEP_LOOPNUM; i++)
    {
        step.ingestionArray[i] = (uint16_t)(0x0000FFFF & DATAEEPROM_Read(EEPROM_START_ADDR + 0x130 + 4 * i));
    }

    for(uint8_t i = 0; i < _STEP_LOOPNUM; i++)
    {
        step.movementArray[i] = (uint16_t)(0x0000FFFF & DATAEEPROM_Read(EEPROM_START_ADDR + 0x160 + 4 * i));
    }

    for(uint8_t i = 0; i < _STEP_LOOPNUM; i++)
    {
        step.climbArray[i] = (uint16_t)(0x0000FFFF & DATAEEPROM_Read(EEPROM_START_ADDR + 0x190 + 4 * i));
    }

    for(uint8_t i = 0; i < _STEP_LOOPNUM; i++)
    {
        step.ruminateArray[i] = (uint16_t)(0x0000FFFF & DATAEEPROM_Read(EEPROM_START_ADDR + 0x1C0 + 4 * i));
    }

    for(uint8_t i = 0; i < _STEP_LOOPNUM; i++)
    {
        step.otherArray[i] = (uint16_t)(0x0000FFFF & DATAEEPROM_Read(EEPROM_START_ADDR + 0x200 + 4 * i));
    }

    step.stepStage = (uint8_t)(0x000000FF & DATAEEPROM_Read(EEPROM_START_ADDR + 8));
    rtc.tenSecTick = (uint8_t)(0x000000FF & DATAEEPROM_Read(EEPROM_START_ADDR + 16));
    rtc.tenSecIndex = RESET;
    rtc.twentyMinIndex = RESET;

    rfid_printf("\nstepArray = ");

    for(uint8_t i = 0; i < _STEP_LOOPNUM; i++)
    {
        rfid_printf("%x ", step.movementArray[i]);
    }

    rfid_printf("\nstepStage = %d\n", step.stepStage);
    rfid_printf("tenSecTick = %d\n", rtc.tenSecTick);
}

/**
  * @brief Get Device Unique Code
  * @retval None
*/
void Get_SerialNum(void)
{
    memset(&device, 0, sizeof(device));
    device.deviceSerial0 = DATAEEPROM_Read(EEPROM_START_ADDR);
    device.deviceSerial1 = DATAEEPROM_Read(EEPROM_START_ADDR + 4);
//  device.deviceSerial0 = *(uint32_t*)(0x1FF80050);//HAL_GetUIDw0
//  device.deviceSerial1 = *(uint32_t*)(0x1FF80054);//HAL_GetUIDw1
//  device.deviceSerial2 = *(uint32_t*)(0x1FF80064);//HAL_GetUIDw2
    device.deviceCode1 = (uint8_t)(0x000000FF & device.deviceSerial0 >> 24);
    device.deviceCode2 = (uint8_t)(0x000000FF & device.deviceSerial0 >> 16);
    device.deviceCode3 = (uint8_t)(0x000000FF & device.deviceSerial0 >> 8);
    device.deviceCode4 = (uint8_t)(0x000000FF & device.deviceSerial0);
    device.deviceCode5 = (uint8_t)(0x000000FF & device.deviceSerial1 >> 24);
    device.deviceCode6 = (uint8_t)(0x000000FF & device.deviceSerial1 >> 16);
}

/**
  * @brief Show Message
  * @retval None
*/
void Show_Message(void)
{
    unsigned int  ReadValueTemp;
    rfid_printf("\n|***********************************************************|\n");
    rfid_printf("|*-------------------RFID collar program-------------------*|\n");
    rfid_printf("|*---------using USART1,configuration:%d 8-N-1---------*|\n", 115200);
    rfid_printf("|*-when in transfer mode,the data can't exceed 256 bytes!!-*|\n");
    rfid_printf("|*-----------deviceCode in eeprom = %08x", device.deviceSerial0);
    rfid_printf("%04x-----------*|\n", (uint16_t)(0x0000FFFF & device.deviceSerial1 >> 16));
    rfid_printf("|***********************************************************|\n");

    #if (_DEBUG == 1)
    printf("\n|*************************************************************|\n");
    printf("|*-----------please configure deviceCode in eeprom-----------*|\n");
    printf("|*------Header-------|---------device code---------|---Tail--*|\n");
    printf("|*0x41 0x42 0x43 0x44|0xXX 0xXX 0xXX 0xXX 0xXX 0xXX|0x0D 0x0A*|\n");
    printf("|*************************************************************|\n");
    #endif

    ReadValueTemp = ADXL362RegisterRead(XL362_DEVID_AD);    //Analog Devices device ID, 0xAD

    if(ReadValueTemp == 0xAD)
    {
        LED_GREEN_ON();
        HAL_Delay(500);
    }

    rfid_printf("\n|*************************************|\n");
    rfid_printf("|*---------ADXL362 CHIP INFO---------*|\n");
    rfid_printf("|*-Analog Devices device ID: %x------*|\n", ReadValueTemp);    //send via UART
    ReadValueTemp = ADXL362RegisterRead(XL362_DEVID_MST);    //Analog Devices MEMS device ID, 0x1D

    if(ReadValueTemp == 0x1D)
    {
        LED_GREEN_OFF();
        HAL_Delay(500);
    }

    rfid_printf("|*-Analog Devices MEMS device ID: %x-*|\n", ReadValueTemp);    //send via UART
    ReadValueTemp = ADXL362RegisterRead(XL362_PARTID);    //part ID, 0xF2

    if(ReadValueTemp == 0xF2)
    {
        LED_GREEN_ON();
        HAL_Delay(500);
    }

    rfid_printf("|*-Part ID: %x-----------------------*|\n", ReadValueTemp);    //send via UART
    ReadValueTemp = ADXL362RegisterRead(XL362_REVID);    //version ID, 0x03

    if(ReadValueTemp == 0x02 || ReadValueTemp == 0x03)
    {
        LED_GREEN_OFF();
    }

    rfid_printf("|*-Version ID: %x---------------------*|\n", ReadValueTemp);    //send via UART
    rfid_printf("|*************************************|\n");

    MX_SPI1_DeInit();
    MX_SPI2_DeInit();
}

/**
  * @brief Set_DeviceInfo
  * @retval None
*/
void Set_DeviceInfo(void)
{
    /* Set transmission flag: trasfer complete*/
    uint32_t data;

    /*##-1- Check UART receive data whether is °ÆABCD°Ø begin or not ###########################*/
    if(usart.rxBuffer[0] == 0x41 && usart.rxBuffer[1] == 0x42 && usart.rxBuffer[2] == 0x43 && usart.rxBuffer[3] == 0x44)// ‰»Î°ÆABCD°Ø
    {
        data = ((uint32_t)(0xFF000000 & usart.rxBuffer[4] << 24) + (uint32_t)(0x00FF0000 & usart.rxBuffer[5] << 16) + (uint32_t)(0x0000FF00 & usart.rxBuffer[6] << 8) + (uint32_t)(0x000000FF & usart.rxBuffer[7]));
        DATAEEPROM_Program(EEPROM_START_ADDR, data);
        data = ((uint32_t)(0xFF000000 & usart.rxBuffer[8] << 24) + (uint32_t)(0x00FF0000 & usart.rxBuffer[9] << 16));
        DATAEEPROM_Program(EEPROM_START_ADDR + 4, data);

        Get_SerialNum();

        #if (_DEBUG == 1)
        printf("\neeprom program end\n");
        printf("deviceCode = %08x", device.deviceSerial0);
        printf("%04x\n", (uint16_t)(0x0000FFFF & device.deviceSerial1 >> 16));
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
    if (ErrorIndex == 0x01 || ErrorIndex == 0x02)
        LED_Blinking(200);
    else
        LED_Blinking(2000);

    __disable_irq();

    while (1)
    {
    }

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
