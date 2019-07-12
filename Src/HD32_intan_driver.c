#include "HD32_intan_driver.h"
#include "HD32_SD_driver.h"
int HD32_intan_init(SPI_HandleTypeDef* hspi,GPIO_TypeDef* CSS_port,uint16_t CSS_pin,uint32_t fs,IntanParams* param)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = INTAN_CS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = 0;
	HAL_GPIO_Init(INTAN_CS_GPIO_Port, &GPIO_InitStruct);
	//check SPI speed, should be 12MHz
	const uint8_t RHD2164_IMPTEST_20K[22] ={222,32,40,2,204,0,0,0,0x01,0x02,23,2,13,0,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff};
	const uint8_t RHD2132_20K[22] ={222,3,7,2,204,0,0,0,22,0,23,0,16,124,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff};
	const uint8_t RHD2132_10K[22] ={222,4,18,2,204,0,0,0,22,0,23,0,16,124,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff};
	const uint8_t RHD2132_5K[22]  ={222,8,40,2,204,0,0,0,22,0,23,0,16,124,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff};
	const uint8_t RHD2132_1250[22]={222,32,40,2,204,0,0,0,22,0,23,0,16,124,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff};
	//INTAN_SPI_Init(hspi);
	uint8_t intan_read[64];
	for(uint8_t i=0;i<64;i++){
		intan_read[i]=HD32_intan_readReg(hspi,CSS_port,CSS_pin,i);
	}
	HD32_intan_setup(hspi,CSS_port,CSS_pin,(uint8_t*)RHD2164_IMPTEST_20K,sizeof(RHD2164_IMPTEST_20K));
	for(uint8_t i=0;i<64;i++)
	{
		intan_read[i]=HD32_intan_readReg(hspi,CSS_port,CSS_pin,i);
	}
	
	param->ChannelNum=intan_read[62];
	param->SamplingRate=fs;
	
	HD32_intan_calibration(hspi,CSS_port,CSS_pin);
	return 0;
}

void HD32_intan_InitConvertCmd(uint16_t* cmdBuf){
	for(int i=0;i<32;i++){
		cmdBuf[i]=1+(((i+1)&0x1F)<<8);	//DMA always start at first element, and intan have 2 command lag
		//cmdBuf[i]=0x3F01; //cycling cmd
	}
	//Initialize DMA
}

void HD32_intan_DMASETUP(SPI_HandleTypeDef* hspi,uint8_t* RXaddr,uint32_t RXsize,TIM_HandleTypeDef* htim,uint8_t* TXaddr,uint32_t TXsize){ //ready SPI for RX
	//HD32_intan_halfword_Init(hspi);
	hspi->hdmarx->Instance->M0AR=(uint32_t)RXaddr;
	hspi->hdmarx->Instance->PAR = (uint32_t)&hspi->Instance->DR;
	hspi->hdmarx->Instance->NDTR=RXsize/2;
	hspi->hdmatx->Instance->CR&=~DMA_IT_TC;											//Do not trigger any interrupt
	__HAL_SPI_DISABLE(hspi);
	SET_BIT(hspi->Instance->CR2, SPI_CR2_RXDMAEN); //enable spi_DMA_request
	__HAL_SPI_ENABLE(hspi);
	__HAL_DMA_ENABLE(hspi->hdmarx);	
	
	//consigure timer DMA for transmit SPI data
	htim->hdma[TIM_DMA_ID_CC3]->Instance->M0AR=(uint32_t)TXaddr;
	htim->hdma[TIM_DMA_ID_CC3]->Instance->PAR=(uint32_t)&hspi->Instance->DR;
	htim->hdma[TIM_DMA_ID_CC3]->Instance->NDTR=(uint32_t)TXsize/2;
	htim->hdma[TIM_DMA_ID_CC3]->Instance->CR&=~DMA_IT_TC;											//Do not trigger any interrupt
	__HAL_TIM_ENABLE_DMA(htim, TIM_DMA_CC3); //enable timer_DMA_request
	__HAL_DMA_ENABLE(htim->hdma[TIM_DMA_ID_CC3]);
}

int HD32_intan_setup(SPI_HandleTypeDef* hspi,GPIO_TypeDef* CSS_port,uint16_t CSS_pin,uint8_t* setting,uint8_t regLen){
	for(uint8_t i=0;i<regLen;i++){
		HD32_intan_writeReg(hspi,CSS_port,CSS_pin,i,setting[i]);
	}
	
	return 0;
}

static int HD32_intan_halfword_Init(SPI_HandleTypeDef* hspi){
	__HAL_SPI_DISABLE(hspi);
  hspi->Init.Mode = SPI_MODE_MASTER;
  hspi->Init.Direction = SPI_DIRECTION_2LINES;
  hspi->Init.DataSize = SPI_DATASIZE_16BIT;
  hspi->Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi->Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi->Init.NSS = SPI_NSS_SOFT;
  hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;		//match 12MHz
  hspi->Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi->Init.TIMode = SPI_TIMODE_DISABLE;
  hspi->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi->Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(hspi) != HAL_OK)
  {
    return 1;
  }
	__HAL_SPI_ENABLE(hspi);
	return 0;
}

void HD32_intan_calibration(SPI_HandleTypeDef* hspi,GPIO_TypeDef* CSS_port,uint16_t CSS_pin){
	uint16_t cmd_calib=0x5500;
	HAL_GPIO_WritePin(CSS_port,CSS_pin,GPIO_PIN_RESET);			//set CS low
	HAL_Delay(1);
	HAL_SPI_Transmit(hspi,(uint8_t*)&cmd_calib,2,1000); 			//Send dummy byte to Flush data
	cmd_calib=0x00;																					//set cmd to 0 to send dummy data
	HAL_GPIO_WritePin(CSS_port,CSS_pin,GPIO_PIN_SET);				//set CS HIGH
	for(uint8_t i=0;i<9;i++){
		HAL_GPIO_WritePin(CSS_port,CSS_pin,GPIO_PIN_RESET);		//set CS low
		HAL_SPI_Transmit(hspi,(uint8_t*)&cmd_calib,2,1000); 		//Send dummy byte to Flush data
		HAL_GPIO_WritePin(CSS_port,CSS_pin,GPIO_PIN_SET);			//set CS HIGH
	}
}

void HD32_intan_writeDAC(SPI_HandleTypeDef* hspi,GPIO_TypeDef* CSS_port,uint16_t CSS_pin,uint8_t value){
	uint16_t cmd_calib=0x8600;
	cmd_calib|=value;
	HAL_GPIO_WritePin(CSS_port,CSS_pin,GPIO_PIN_RESET);			//set CS low
	HAL_Delay(1);
	HAL_SPI_Transmit(hspi,(uint8_t*)&cmd_calib,2,1000); 			//Send dummy byte to Flush data
	cmd_calib=0x00;																					//set cmd to 0 to send dummy data
	HAL_GPIO_WritePin(CSS_port,CSS_pin,GPIO_PIN_SET);				//set CS HIGH
	for(uint8_t i=0;i<9;i++){
		HAL_GPIO_WritePin(CSS_port,CSS_pin,GPIO_PIN_RESET);		//set CS low
		HAL_SPI_Transmit(hspi,(uint8_t*)&cmd_calib,2,1000); 		//Send dummy byte to Flush data
		HAL_GPIO_WritePin(CSS_port,CSS_pin,GPIO_PIN_SET);			//set CS HIGH
	}
}

int HD32_intan_readReg(SPI_HandleTypeDef* hspi,GPIO_TypeDef* CSS_port,uint16_t CSS_pin,uint8_t addr){
	uint16_t result=0;
	uint16_t cmd=0;
	addr|=0xc0;
	cmd=addr<<8;
	HAL_GPIO_WritePin(CSS_port,CSS_pin,GPIO_PIN_RESET);			//set CS low
	HAL_Delay(1);
	//HAL_SPI_Transmit(hspi,(uint8_t*)&cmd,1,1000); 					//Send command byte
	SPI3->DR=cmd;
	SPI_WAIT_TILL_SENT(SPI3);
	HAL_GPIO_WritePin(CSS_port,CSS_pin,GPIO_PIN_SET);				//set CS HIGH
	HAL_Delay(1);
	HAL_GPIO_WritePin(CSS_port,CSS_pin,GPIO_PIN_RESET);			//set CS low
	HAL_Delay(1);
	//HAL_SPI_Transmit(hspi,(uint8_t*)&cmd,1,1000); 					//Send command byte
	SPI3->DR=cmd;
	SPI_WAIT_TILL_SENT(SPI3);
	HAL_GPIO_WritePin(CSS_port,CSS_pin,GPIO_PIN_SET);				//set CS HIGH
	HAL_Delay(1);
	HAL_GPIO_WritePin(CSS_port,CSS_pin,GPIO_PIN_RESET);			//set CS low
	HAL_Delay(1);
	//HAL_SPI_TransmitReceive(hspi,(uint8_t*)&cmd,(uint8_t*)&result,1,1000); //Send command byte
	SPI3->DR=cmd;
	SPI_WAIT_TILL_SENT(SPI3);
	result=SPI2->DR;
	HAL_Delay(1);
	HAL_GPIO_WritePin(CSS_port,CSS_pin,GPIO_PIN_SET);				//set CS HIGH
	HAL_Delay(1);
	result&=0xff;
	return result;
}

int HD32_intan_writeReg(SPI_HandleTypeDef* hspi,GPIO_TypeDef* CSS_port,uint16_t CSS_pin,uint8_t addr,uint8_t data){
	uint16_t result=0;
	uint16_t cmd=0;
	addr|=0x80;
	addr&=0xBF;
	cmd=addr<<8|data;
	HAL_GPIO_WritePin(CSS_port,CSS_pin,GPIO_PIN_RESET);				//set CS low
	//HAL_Delay(1);
	//HAL_SPI_Transmit(hspi,(uint8_t*)&cmd,1,1000); 						//Send command byte
	SPI3->DR=cmd;
	SPI_WAIT_TILL_SENT(SPI3);
	HAL_GPIO_WritePin(CSS_port,CSS_pin,GPIO_PIN_SET);					//set CS HIGH
	HAL_Delay(1);
	
	HAL_GPIO_WritePin(CSS_port,CSS_pin,GPIO_PIN_RESET);				//set CS low
	//HAL_Delay(1);
	//HAL_SPI_Transmit(hspi,(uint8_t*)&cmd,1,1000); 						//Send command byte
	SPI3->DR=cmd;
	SPI_WAIT_TILL_SENT(SPI3);
	HAL_GPIO_WritePin(CSS_port,CSS_pin,GPIO_PIN_SET);					//set CS HIGH
	HAL_Delay(1);
	
	HAL_GPIO_WritePin(CSS_port,CSS_pin,GPIO_PIN_RESET);				//set CS low
	//HAL_Delay(1);
	//HAL_SPI_TransmitReceive(hspi,(uint8_t*)&cmd,(uint8_t*)&result,1,1000); //Send command byte
	SPI3->DR=cmd;
	SPI_WAIT_TILL_SENT(SPI3);
	result=SPI2->DR;
	//HAL_Delay(1);
	HAL_GPIO_WritePin(CSS_port,CSS_pin,GPIO_PIN_SET);					//set CS HIGH
	//HAL_Delay(1);
	result&=0xff;
	return result;
}

void HD32_intan_recording_start_DMA(TIM_HandleTypeDef* htim_main,TIM_HandleTypeDef* htim_proc,TIM_HandleTypeDef* htim_res){
	htim_proc->Instance->CNT=0;
	htim_res->Instance->CNT=0;
	htim_main->Instance->CNT=0;
	HAL_TIM_Base_Start_IT(htim_proc); //Enable DSP interrput
	HAL_TIM_Base_Start_IT(htim_res); //Enable wake up timer 
	//Resetting INTAN CS pin function back to timer4
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = INTAN_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
	HAL_GPIO_Init(INTAN_CS_GPIO_Port, &GPIO_InitStruct);
	//Enable PWM signal for Intan CS
	  /* Check the parameters */
  assert_param(IS_TIM_CCX_INSTANCE(htim_main->Instance, TIM_CHANNEL_1));
  /* Enable the Capture compare channel */
  TIM_CCxChannelCmd(htim_main->Instance, TIM_CHANNEL_1, TIM_CCx_ENABLE);
	  /* Check the parameters */
  assert_param(IS_TIM_CCX_INSTANCE(htim_main->Instance, TIM_CHANNEL_3));
  /* Enable the Capture compare channel */
  TIM_CCxChannelCmd(htim_main->Instance, TIM_CHANNEL_3, TIM_CCx_ENABLE);
	
	__HAL_TIM_MOE_ENABLE(htim_main);
	
	//__HAL_TIM_ENABLE_IT(htim_main,TIM_IT_CC3);
	__HAL_TIM_ENABLE(htim_main);
	
	//ADV_SD_WRITING_CONTROL
	#ifdef __SD_SPI_ADVTIMER
		/* Enable the Capture compare channel */
		TIM_CCxChannelCmd(SD_SPI_ADV_TIMER, TIM_CHANNEL_1, TIM_CCx_ENABLE);
		SD_SPI_ADV_TIMER->CNT=700; //ADJUST OFFSET TIME TO OTHER TIMERS
		SD_SPI_ADV_TIMER->BDTR|=(TIM_BDTR_MOE);
		SD_SPI_ADV_TIMER->DIER |= TIM_IT_UPDATE;
		SD_SPI_ADV_TIMER->CR1|=(TIM_CR1_CEN); //ENABLE TIMER
	#endif

}

void HD32_intan_recording_stop_DMA(TIM_HandleTypeDef* htim_main,TIM_HandleTypeDef* htim_proc,TIM_HandleTypeDef* htim_res){
	HAL_TIM_Base_Stop(htim_main); //DISABLE main up timer
	HAL_TIM_Base_Stop(htim_proc); //DISABLE DSP interrput
	HAL_TIM_Base_Stop(htim_res); //DISABLE wake up timer
	htim_main->Instance->CR1&=~TIM_CR1_CEN;
	//__HAL_TIM_MOE_DISABLE(htim_main);
//	//__HAL_TIM_ENABLE_IT(htim_main,TIM_IT_CC3);
	//__HAL_TIM_DISABLE(htim_main);
	//MAIN_TIM->CR1&=~TIM_CR1_CEN;
	#ifdef __SD_SPI_ADVTIMER
		/* Enable the Capture compare channel */
	SD_SPI_ADV_TIMER->CR1&=~(TIM_CR1_CEN); //DISABLE TIMER
	#endif
}


static int INTAN_SPI_Init(SPI_HandleTypeDef* hspi){
  //hspi->Instance = SPI3;
  hspi->Init.Mode = SPI_MODE_MASTER;
  hspi->Init.Direction = SPI_DIRECTION_2LINES;
  hspi->Init.DataSize = SPI_DATASIZE_16BIT;
  hspi->Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi->Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi->Init.NSS = SPI_NSS_SOFT;
  hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;		//Match 12MHz (MAX=24MHz)
  hspi->Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi->Init.TIMode = SPI_TIMODE_DISABLE;
  hspi->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi->Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(hspi) != HAL_OK)
  {
    return 1;
  }
	__HAL_SPI_ENABLE(hspi);
	return 0;
}
