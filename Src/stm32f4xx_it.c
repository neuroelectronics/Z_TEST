/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dataMGR.h"
#include "HD32_intan_driver.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern float dataBuf[100];
extern float dataBuf1[100];
extern float   impedance[128];
extern int channel_imp_flag[MAX_CHANNEL_NUMBER];
extern uint16_t DAC_cmd[100];
extern int num_sine_values;
extern uint32_t DAC_on;
uint32_t adc_cnt;
extern uint32_t ch;
float max1,max2;
float min1,min2;

extern int test_cyc;
extern SPI_HandleTypeDef hspi3;

//SPI READER
volatile uint16_t temp_SPI;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_spi3_rx;
extern DMA_HandleTypeDef hdma_tim1_up;
extern DMA_HandleTypeDef hdma_tim4_up;
extern DMA_HandleTypeDef hdma_tim4_ch3;
extern DMA_HandleTypeDef hdma_tim4_ch2;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim14;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream0 global interrupt.
  */
void DMA1_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream0_IRQn 0 */

  /* USER CODE END DMA1_Stream0_IRQn 0 */
  /* USER CODE BEGIN DMA1_Stream0_IRQn 1 */

  /* USER CODE END DMA1_Stream0_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream3 global interrupt.
  */
void DMA1_Stream3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream3_IRQn 0 */

  /* USER CODE END DMA1_Stream3_IRQn 0 */
  /* USER CODE BEGIN DMA1_Stream3_IRQn 1 */

  /* USER CODE END DMA1_Stream3_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream6 global interrupt.
  */
void DMA1_Stream6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream6_IRQn 0 */

  /* USER CODE END DMA1_Stream6_IRQn 0 */
  /* USER CODE BEGIN DMA1_Stream6_IRQn 1 */

  /* USER CODE END DMA1_Stream6_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt and TIM10 global interrupt.
  */
void TIM1_UP_TIM10_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 0 */

  /* USER CODE END TIM1_UP_TIM10_IRQn 0 */
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 1 */

  /* USER CODE END TIM1_UP_TIM10_IRQn 1 */
}

/**
  * @brief This function handles TIM1 trigger and commutation interrupts and TIM11 global interrupt.
  */
void TIM1_TRG_COM_TIM11_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_TRG_COM_TIM11_IRQn 0 */

  /* USER CODE END TIM1_TRG_COM_TIM11_IRQn 0 */
  /* USER CODE BEGIN TIM1_TRG_COM_TIM11_IRQn 1 */

  /* USER CODE END TIM1_TRG_COM_TIM11_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */

  /* USER CODE END TIM4_IRQn 0 */
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles TIM8 trigger and commutation interrupts and TIM14 global interrupt.
  */
void TIM8_TRG_COM_TIM14_IRQHandler(void)
{
  /* USER CODE BEGIN TIM8_TRG_COM_TIM14_IRQn 0 */
	TIM14->SR&=~(TIM_FLAG_UPDATE|TIM_FLAG_CC1); //Clear interrupt flag
  /* USER CODE END TIM8_TRG_COM_TIM14_IRQn 0 */
  /* USER CODE BEGIN TIM8_TRG_COM_TIM14_IRQn 1 */

	if(DAC_on)
	{
		INTAN_CS_GPIO_Port->BSRR|=INTAN_CS_Pin<<16;//Set CS low
		//SPI3->DR=((0x80|0x06)<<8)|(uint8_t)((0.5+sin(2.0*PI*adc_cnt/system_fs*target_fs)/2.0)*255); //write to register 6 with sine wave
		SPI3->DR=DAC_cmd[adc_cnt%num_sine_values];
		SPI_WAIT_TILL_SENT(SPI3);
		INTAN_CS_GPIO_Port->BSRR|=INTAN_CS_Pin;//Set CS high
		for(int i=0;i<5;i++){__nop();}; //Need at least 154ns for the CS
	}
		//Flush SPI DRs
	while((SPI3->SR&SPI_SR_RXNE)==0);
	volatile uint16_t temp;
	temp=SPI2->DR;
	temp=SPI3->DR;
	temp=SPI4->DR;
	temp=SPI5->DR;
	
	//Send Convert Command
	INTAN_CS_GPIO_Port->BSRR|=INTAN_CS_Pin<<16;//Set CS low
	SPI3->DR=0x01|((ch&0x1f)<<8); //Send convert command 0-31
	SPI_WAIT_TILL_SENT(SPI3);
	
	INTAN_CS_GPIO_Port->BSRR|=INTAN_CS_Pin;//Set CS high
	for(int i=0;i<5;i++){__nop();}; //Need at least 154ns for the CS
	
	INTAN_CS_GPIO_Port->BSRR|=INTAN_CS_Pin<<16;//Set CS Low to trigger ADC sampling
	
//	for(int i=0;i<5;i++){__nop();}; //Need at least 154ns for the CS
//	SPI3->DR=0xc700; //Send dummy command (read register 7)
//	SPI_WAIT_TILL_SENT(SPI3);
//	INTAN_CS_GPIO_Port->BSRR|=INTAN_CS_Pin;//Set CS high
//	for(int i=0;i<5;i++){__nop();}; //Need at least 154ns for the CS
//	INTAN_CS_GPIO_Port->BSRR|=INTAN_CS_Pin<<16;
//	
//	SPI3->DR=0xc700; //Send dummy command
//	SPI_WAIT_TILL_SENT(SPI3);
//	INTAN_CS_GPIO_Port->BSRR|=INTAN_CS_Pin;//Set CS high
//	for(int i=0;i<5;i++){__nop();}; //Need at least 154ns for the CS
//	INTAN_CS_GPIO_Port->BSRR|=INTAN_CS_Pin<<16; //Pull CS low to issue command
	
	while((SPI3->SR&SPI_SR_RXNE)==0);
	if((ch&0x20)==0)
	{
		//dataBuf[adc_cnt%(fs_ratio*test_cyc)]+=(uint8_t)((0.5+sin(2.0*PI*adc_cnt/system_fs*target_fs)/2.0)*255); 
		volatile uint16_t temp_SPI=SPI2->DR;		
		dataBuf[adc_cnt%num_sine_values]+=((float)(*((int16_t*)(&temp_SPI))))/test_cyc; //MISO0 PhaseA
		temp_SPI=SPI5->DR;		
		dataBuf1[adc_cnt%num_sine_values]+=((float)(*((int16_t*)(&temp_SPI))))/test_cyc; //MISO1 PhaseA
		
//		MeasuredDataBuffer[ch][adc_cnt%num_sine_values]+=(float)(int16_t)(SPI2->DR)/test_cyc; //MISO0 PhaseA
//		MeasuredDataBuffer[ch+64][adc_cnt%num_sine_values]+=(float)(int16_t)(SPI5->DR)/test_cyc; //MISO1 PhaseA
	}
	else
	{
		volatile uint16_t temp_SPI=SPI3->DR<<1;
		dataBuf[adc_cnt%num_sine_values]+=((float)(*((int16_t*)(&temp_SPI))))/test_cyc; //MISO0 PhaseB
		temp_SPI=SPI4->DR<<1;
		dataBuf1[adc_cnt%num_sine_values]+=((float)(*((int16_t*)(&temp_SPI))))/test_cyc; //MISO1 PhaseB
		
		//MeasuredDataBuffer[ch][adc_cnt%num_sine_values]+=(float)(int16_t)(SPI3->DR<<1)/test_cyc; //MISO0 PhaseB
		//MeasuredDataBuffer[ch+64][adc_cnt%num_sine_values]+=(float)(int16_t)(SPI4->DR<<1)/test_cyc; //MISO1 PhaseB
	}
	adc_cnt++;
	
	

	if(adc_cnt>=test_cyc*num_sine_values)
	{
		TIM14->CR1&=~TIM_CR1_CEN; //DISABLE TIMER
		HAL_TIM_Base_Stop(&htim14);
		
		for(int i=0;i<num_sine_values;i++)  // Find max find min;
		{
			if(i==0)
			{
				min1 = dataBuf[i];
				max1 = dataBuf[i];
				min2 = dataBuf1[i];
				max2 = dataBuf1[i];
			}
			else
			{
				if(dataBuf[i]>max1)
				{
					max1 = dataBuf[i];
				}
				else if(dataBuf[i]<min1)
				{
					min1 = dataBuf[i];
				}
				
				if(dataBuf1[i]>max2)
				{
					max2 = dataBuf1[i];
				}
				else if(dataBuf1[i]<min2)
				{
					min2 = dataBuf1[i];
				}
			}
 			dataBuf[i]=0; //Reset buffer
			dataBuf1[i]=0; //Reset buffer
		}

		
		//max /= 38e-9;
		//min /= 38e-9;
		float DAC_current=3.8e-9;
		impedance[ch]=(max1-min1)*0.195*1e-6/DAC_current*1e-3/2; //*0.195->microVolt *1e-6->Volt /DAC_current ->Ohm *1e-3 ->KOhm 
		impedance[ch+64]=(max2-min2)*0.195*1e-6/DAC_current*1e-3/2;
		
		// check if impedance criteria is met
		if ( ( ( (int16_t)impedance[ch] ) < MAX_IMP) & (impedance[ch] > 0))
		{
			channel_imp_flag[ch] = 1;
		}
		else
		{
			channel_imp_flag[ch] = 0;
		}
		if ( ( ( (int16_t)impedance[ch+64] ) < MAX_IMP) & (impedance[ch+64] > 0))
		{
			channel_imp_flag[ch+64] = 1;
		}
		else
		{
			channel_imp_flag[ch+64] = 0;
		}
		
		adc_cnt=0;
		ch++;
		ch&=0x3F;
		
		HAL_GPIO_WritePin(INTAN_CS_GPIO_Port,INTAN_CS_Pin,GPIO_PIN_RESET);//Set CS low
		SPI3->DR=((0x80|0x07)<<8)|ch; //write to register 7 with DAC channel
		SPI_WAIT_TILL_SENT(SPI3);
		HAL_GPIO_WritePin(INTAN_CS_GPIO_Port,INTAN_CS_Pin,GPIO_PIN_SET);//Set CS high
		// Should we make it wait for 154 ns here ???
		
		TIM14->CR1|=TIM_CR1_CEN; //enABLE TIMER
	}
	
  /* USER CODE END TIM8_TRG_COM_TIM14_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream7 global interrupt.
  */
void DMA1_Stream7_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream7_IRQn 0 */

  /* USER CODE END DMA1_Stream7_IRQn 0 */
  /* USER CODE BEGIN DMA1_Stream7_IRQn 1 */

  /* USER CODE END DMA1_Stream7_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream5 global interrupt.
  */
void DMA2_Stream5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream5_IRQn 0 */

  /* USER CODE END DMA2_Stream5_IRQn 0 */
  /* USER CODE BEGIN DMA2_Stream5_IRQn 1 */

  /* USER CODE END DMA2_Stream5_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
