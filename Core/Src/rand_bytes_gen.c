/**
  ******************************************************************************
  * @file    rng.c
  * @brief   This file provides code for the configuration
  *          of the rng instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "rand_bytes_gen.h"

/** @addtogroup STM32_Crypto_Examples
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Entropy String. Uniformly distributed random bit string 1*/
uint8_t entropy_data[32] =
{
    0x9d, 0x20, 0x1a, 0x18, 0x9b, 0x6d, 0x1a, 0xa7, 0x0e,
    0x79, 0x57, 0x6f, 0x36, 0xb6, 0xaa, 0x88, 0x55, 0xfd,
    0x4a, 0x7f, 0x97, 0xe9, 0x71, 0x69, 0xb6, 0x60, 0x88,
    0x78, 0xe1, 0x9c, 0x8b, 0xa5
};
/* Nonce. Non repeating sequence, such as a timestamp */
uint8_t nonce[] = {0xFE, 0xA9, 0x96, 0xD4, 0x62, 0xC5};
/* Personalization String */
uint8_t personalization_String[] = {0x1E, 0x6C, 0x7B, 0x82, 0xE5, 0xA5, 0x71, 0x8D};

/* Array that will be filled with random bytes */
uint8_t RandomString[32] = {0, };

RNGstate_stt RNGstate;

RNGinitInput_stt RNGinit_st;

int32_t status = RNG_SUCCESS;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/* RNG init function */
void RNG_Init(void)
{
    for(uint8_t i = 0; i < sizeof( nonce ); i++)
    {
        srand(HAL_GetTick());
        nonce[i] = rand() % 255;
        rfid_printf("nonce[%d] = %02x ", i, nonce[i]);
    }

    rfid_printf("\n");

    /* Enable CRC clock */
    __CRC_CLK_ENABLE();

    /* Set the values of EntropyData, Nonce, Personalization String and their sizes inside the RNGinit_st structure */
    RNGinit_st.pmEntropyData = entropy_data;
    RNGinit_st.mEntropyDataSize = sizeof(entropy_data);
    RNGinit_st.pmNonce =  nonce;
    RNGinit_st.mNonceSize = sizeof( nonce );
    RNGinit_st.pmPersData = personalization_String;
    RNGinit_st.mPersDataSize = sizeof( personalization_String );

    status = RNGinit(&RNGinit_st, &RNGstate);

    if  ( status != RNG_SUCCESS )
    {
        /* In case of randomization not success possible values of status:
         * RNG_ERR_BAD_ENTROPY_SIZE, RNG_ERR_BAD_PERS_STRING_SIZE
         */

        Error_Handler();
    }

}

/* RNG gen function */
void RNG_Gen(void)
{
    /* The Random engine has been initialized, the status is in RNGstate */

    /* Now fill the random string with random bytes */
    status = RNGgenBytes(&RNGstate, NULL, RandomString, sizeof(RandomString));

    if (status == RNG_SUCCESS)
    {
        /* Random Generated Succefully, free the state before returning */
        for(uint16_t i = 0; i < sizeof(RandomString); i++)
        {
            rfid_printf("%02x ", RandomString[i]);
        }

        rfid_printf("\n");

        status = RNGfree(&RNGstate);

        if  ( status == RNG_SUCCESS )
        {
        }
        else
        {
            Error_Handler();
        }
    }
    else
    {
        /* In case of randomization not success possible values of status:
         * RNG_ERR_BAD_PARAMETER, RNG_ERR_UNINIT_STATE
         */

        Error_Handler();
    }
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
