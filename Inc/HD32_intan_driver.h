#include "stm32f4xx_hal.h"
#include "CE32_macro.h"
#include "main.h"

//set following parameter base on your project
#define INTAN_SPI SPI3
#define INTAN_SPI_TX_DMA DMA1_Stream7
#define INTAN_SPI_RX_DMA DMA1_Stream0
#define MAIN_TIM TIM4

typedef struct
{
	uint8_t ChannelNum;
	uint16_t SamplingRate;
} IntanParams;

int HD32_intan_init(SPI_HandleTypeDef* hspi,GPIO_TypeDef* CSS_port,uint16_t CSS_pin,uint32_t fs,IntanParams* param);
void HD32_intan_InitConvertCmd(uint16_t* cmdBuf);
void HD32_intan_DMASETUP(SPI_HandleTypeDef* hspi,uint8_t* RXaddr,uint32_t RXsize,TIM_HandleTypeDef* htim,uint8_t* TXaddr,uint32_t TXsize);
int HD32_intan_readReg(SPI_HandleTypeDef* hspi,GPIO_TypeDef* CSS_port,uint16_t CSS_pin,uint8_t addr);
int HD32_intan_writeReg(SPI_HandleTypeDef* hspi,GPIO_TypeDef* CSS_port,uint16_t CSS_pin,uint8_t addr,uint8_t data);
int HD32_intan_setup(SPI_HandleTypeDef* hspi,GPIO_TypeDef* CSS_port,uint16_t CSS_pin,uint8_t* setting,uint8_t regLen);
void HD32_intan_calibration(SPI_HandleTypeDef* hspi,GPIO_TypeDef* CSS_port,uint16_t CSS_pin);
void HD32_intan_writeDAC(SPI_HandleTypeDef* hspi,GPIO_TypeDef* CSS_port,uint16_t CSS_pin,uint8_t value);
void HD32_intan_recording_start_DMA(TIM_HandleTypeDef* htim_main,TIM_HandleTypeDef* htim_proc,TIM_HandleTypeDef* htim_wake);
void HD32_intan_recording_stop_DMA(TIM_HandleTypeDef* htim_main,TIM_HandleTypeDef* htim_proc,TIM_HandleTypeDef* htim_res);
static int HD32_intan_halfword_Init(SPI_HandleTypeDef* hspi);
static int INTAN_SPI_Init(SPI_HandleTypeDef* hspi);


__forceinline int16_t HD32_intan_convert(uint16_t conv_cmd){
	PIN_SET(INTAN_CS);   	//Set CS high
	//for(uint8_t idx;idx<5;idx++);//skip some time make the CS signal fulfill 154ns minimal duration
	PIN_RESET(INTAN_CS);//Set CS low to issue a command			
	return INTAN_SPI->DR;
}

#define PI 3.1415926
#define SPI_WAIT_TILL_SENT(__handle__) if(((__handle__)->SR&SPI_SR_BSY)==0){for(int i=0;i<10;i++);}while(((__handle__)->SR&SPI_SR_BSY)!=0); 
//#define SPI_WAIT_TILL_SENT for(int i=0;i<100;i++)
