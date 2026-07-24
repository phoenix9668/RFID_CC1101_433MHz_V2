/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __RAND_BYTES_GEN_H__
#define __RAND_BYTES_GEN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Exported functions ------------------------------------------------------- */
void RNG_Init(void);
void RNG_Gen(void);

extern uint8_t RandomString[32];

#ifdef __cplusplus
}
#endif

#endif /* __RAND_BYTES_GEN_H__ */
