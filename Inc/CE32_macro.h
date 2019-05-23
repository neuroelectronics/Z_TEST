#include "stm32f4xx_hal.h"

#ifndef __CE32_macro

#define __CE32_macro
#define PIN_SET(__HANDLE__) (__HANDLE__##_GPIO_Port->BSRR|=__HANDLE__##_Pin)
#define PIN_RESET(__HANDLE__) (__HANDLE__##_GPIO_Port->BSRR|=__HANDLE__##_Pin<<16)

#define CE32_NOP(__x__) (for(int i=0;i<__x__;i++){__nop();})
typedef struct
{
  __IO uint32_t ISR;   /*!< DMA interrupt status register */
  __IO uint32_t Reserved0;
  __IO uint32_t IFCR;  /*!< DMA interrupt flag clear register */
} DMA_Base_Registers;


#define SET_STIM_DAC(__HANDLE__)  DAC1->DHR12L1=(__HANDLE__);
#define STIM_ON		GPIOA->BSRR|=LD1_Pin|LD2_Pin
#define STIM_OFF	GPIOA->BSRR=(LD1_Pin|LD2_Pin)<<16
#endif
