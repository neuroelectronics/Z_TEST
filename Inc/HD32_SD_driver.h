#include "stm32f4xx_hal.h"
#include "CE32_macro.h"
#include "dataMGR.h"
#include "main.h"

//Configure if in timer controlled low noise mode
//#define __SD_SPI_LOWNOISE
//#define __SD_SPI_ADVTIMER
//set these macro according to your project
#define SD_SPI SPI1
#define SD_SPI_TIM TIM1
#define SD_SPI_ADV_TIMER TIM3

#define SD_SPI_LOW_TIME 1600
#define SD_SPI_HIGH_TIME 4500
#ifndef __SD_SPI_LOWNOISE
	//spi DMA stream
	#define SD_DMA_TX DMA2_Stream2
#else
	//Timer DMA_Stream
	#define SD_DMA_TX DMA2_Stream5
#endif

#ifdef __SD_SPI_ADVTIMER
	extern volatile int SD_CLK_STATE;
#endif

//
#define SD_CMD0  0x40,0x00,0x00,0x00,0x00,0x95
#define SD_CMD1  0x41,0x00,0x00,0x00,0x00,0x95
#define SD_CMD6  0x46,0x80,0x0f,0x0f,0x01,0x87
#define SD_CMD8  0x48,0x00,0x00,0x01,0xAA,0x87
#define SD_CMD9  0x49,0x00,0x00,0x00,0x00,0xff
#define SD_CMD12 0x4C,0x00,0x00,0x00,0x00,0x01
#define SD_CMD16 0x50,0x00,0x00,0x02,0x00,0x01
#define SD_CMD17 0x51,0x00,0x00,0x00,0x00,0x01
#define SD_CMD18 0x52,0x00,0x00,0x00,0x00,0x01
#define SD_CMD23 0x57,0x00,0x00,0x00,0x00,0x01
#define SD_CMD24 0x58,0x00,0x00,0x00,0x00,0x01
#define SD_CMD25 0x59,0x00,0x00,0x00,0x00,0x01
#define SD_CMD41 0x69,0x40,0x00,0x00,0x00,0xE5
#define SD_CMD55 0x77,0x00,0x00,0x00,0x00,0x01
#define SD_CMD58 0x7A,0x00,0x00,0x00,0x00,0x01
#define SD_CMD25_TOKEN_START 	0xFC
#define SD_CMD25_TOKEN_END 		0xFD
#define SD_CMD17_TOKEN_START 	0xFE
#define SD_CMD20_StartRec		 	0x00
#define SD_CMD20_CreateDIR	 	0x10
#define SD_CMD20_EndNMov		 	0x20
#define SD_CMD20_EndWMov		 	0x30
#define SD_CMD20_UpdCI		 		0x40

#define MAX_WAIT_CYC 10000;


#define SPI_GET_FLAG(__HANDLE__, __FLAG__) ((((__HANDLE__)->SR) & (__FLAG__)) == (__FLAG__))
#define SPI_ENABLE(__HANDLE__) ((__HANDLE__)->CR1 |=  SPI_CR1_SPE)
#define SPI_DISABLE(__HANDLE__) ((__HANDLE__)->CR1 &= (~SPI_CR1_SPE))
#define DMA_ENABLE(__HANDLE__)      ((__HANDLE__)->CR |=  DMA_SxCR_EN)
#define DMA_DISABLE(__HANDLE__)     ((__HANDLE__)->CR &=  ~DMA_SxCR_EN)

#define PULL_SPI_BSY while(!(SPI_GET_FLAG(SD_SPI,SPI_FLAG_TXE))){__nop();};
//#define PULL_SPI_BSY while((!(SPI_GET_FLAG(SD_SPI,SPI_FLAG_TXE)))||(SD_SPI_TIM->CNT<80));

typedef struct
{
  uint32_t upd_header; //update header indication for next write
	dataMGR *MGR;				//data manager for main sd buffer
	SPI_HandleTypeDef* hspi;	//SPI handle used by SD card
	GPIO_TypeDef* CSS_port;		//CS port of SD card
	uint16_t CSS_pin;					//CS_pin of SD card
} HD32_SD_Handle;

int HD32_SD_init(SPI_HandleTypeDef* hspi);
int HD32_SD_readCSD(SPI_HandleTypeDef* hspi,CE32_systemParam* sys);
int HD32_SD_writeBlock(uint32_t addr);
int HD32_SD_writeSingleBlock(uint32_t addr,uint8_t *data);
int HD32_SD_writeSingleBlock_DMA(uint32_t addr,uint8_t *data);
int HD32_SD_write512B(uint8_t *data);
int HD32_SD_write515B(uint8_t *data);
void HD32_SD_write515B_DMA(uint8_t *data);
void HD32_SD_write_DMA(uint8_t *data,uint16_t size);
int HD32_SD_write515B_DMA_Check(void);
int HD32_SD_writeBlock_DMA_Check(void);
int HD32_SD_dmaReady(uint8_t *data);
int HD32_SD_readSingleBlock(uint32_t addr,uint8_t *data);
int HD32_SD_read512B(uint8_t *data);
int HD32_SD_preErase(uint32_t blkNum);
int HD32_SD_stopTrans(void);
//int HD32_SD_stopTrans_fast();
int HD32_SD_sendCmd(uint8_t *buf);
int HD32_SD_sendCmdR3(uint8_t *buf,uint32_t *OCR);
int HD32_SD_sendCmdR7(uint8_t *buf);
int HD32_SD_sendACmd(uint8_t *buf);
int HD32_SD_highSpeed(void);
int HD32_SD_SpeedClassCtrl(uint8_t state);

void HD32_SD_Recording_Server(HD32_SD_Handle *handle);

static int SDSPI_lowSpeed_Init(SPI_HandleTypeDef* hspi);
static int SDSPI_midSpeed_Init(SPI_HandleTypeDef* hspi);
static int SDSPI_highSpeed_Init(SPI_HandleTypeDef* hspi);

void HD32_SD_dmaTransmit(uint32_t cnt);

extern uint8_t HD32_sd_temp;

__forceinline void SDSPI_DMA_Init(void){
	PULL_SPI_BSY
	
	/* Configure DMA Channel source address */
	SD_DMA_TX->PAR = (uint32_t)&SD_SPI->DR;
	SPI_DISABLE(SD_SPI);
#ifndef __SD_SPI_LOWNOISE
	SET_BIT(SD_SPI->CR2, SPI_CR2_TXDMAEN); //enable spi_DMA_request
	SPI_ENABLE(SD_SPI);
#else
	SPI_ENABLE(SD_SPI);
	SD_SPI_TIM->CR1&=~(TIM_CR1_CEN);
	SET_BIT(SD_SPI_TIM->DIER, TIM_DIER_UDE); //enable timer DMA
	SD_SPI_TIM->BDTR|=(TIM_BDTR_MOE);	//Enable Timer Main Output and Enable
	SD_SPI_TIM->CR1|=(TIM_CR1_CEN);
#endif
	
}

__forceinline void SDSPI_DMA_Enable(void){
	PULL_SPI_BSY
	
	/* Configure DMA Channel source address */
	//SD_DMA_TX->PAR = (uint32_t)&SD_SPI->DR;
	//SD_DMA_TX->NDTR=256;
	
#ifndef __SD_SPI_LOWNOISE
	SPI_DISABLE(SD_SPI);
	SET_BIT(SD_SPI->CR2, SPI_CR2_TXDMAEN); //enable spi_DMA_request
	DMA_ENABLE(SD_DMA_TX);
	SPI_ENABLE(SD_SPI);
	
#else
	//SD_SPI_TIM->CR1&=~(TIM_CR1_CEN);				//disable timer
	SD_SPI_TIM->CNT=0;
	SET_BIT(SD_SPI_TIM->DIER, TIM_DIER_UDE); //enable timer DMA
	SD_SPI_TIM->CR1|=(TIM_CR1_CEN);
	DMA_ENABLE(SD_DMA_TX);
#endif
}

__forceinline void SDSPI_DMA_Disable(void){
	PULL_SPI_BSY
	DMA_DISABLE(SD_DMA_TX);
#ifndef __SD_SPI_LOWNOISE
	SPI_DISABLE(SD_SPI);
	CLEAR_BIT(SD_SPI->CR2, SPI_CR2_TXDMAEN); //disable spi_DMA_request
	SPI_ENABLE(SD_SPI);
#else
	//SD_SPI_TIM->CR1&=~(TIM_CR1_CEN);					//disable timer
	CLEAR_BIT(SD_SPI_TIM->DIER, TIM_DIER_UDE); //clear timer DMA
#endif
	
}
__forceinline uint16_t SDSPI_TransmitReceive(uint16_t data){
	PULL_SPI_BSY
	SD_SPI->DR=data;
	return SD_SPI->DR;
}

__forceinline uint8_t SDSPI_TransmitReceive_Byte(uint8_t data){

	if((SD_SPI->CR1 &SPI_CR1_SPE) != SPI_CR1_SPE)
  {
    
		SD_SPI->CR1 |=  SPI_CR1_SPE;// Enable SPI peripheral 
  }

	PULL_SPI_BSY

	//uint8_t temp;
	while(SPI_GET_FLAG(SD_SPI, SPI_FLAG_RXNE)){
		HD32_sd_temp=SD_SPI->DR;							//empty current buffer
	};
//#ifdef __SD_SPI_LOWNOISE	
//	if(TIM4->CR1&TIM_CR1_CEN)
//	{
//		while(TIM4->CNT<80){};
//	}
//#endif
#ifdef __SD_SPI_ADVTIMER
	 while(((SD_SPI_ADV_TIMER->CR1&TIM_CR1_CEN)!=0)&&((SD_SPI_ADV_TIMER->CNT<SD_SPI_LOW_TIME) ))
//		while((SD_SCK_GPIO_Port->MODER&0x0c00)!=0x0800)
//	while(SD_CLK_STATE==1)
	{
		__nop();
	}//check if SCK in AF mode
#endif
	*((__IO uint8_t*)(&SD_SPI->DR))=data;		//transmit data by 8-bit
	int cnt=0;
	while((!SPI_GET_FLAG(SD_SPI, SPI_FLAG_RXNE))&&cnt<8*2*500){	//check if response data received (RXNE==1)
		__nop();
		cnt++;
	}
	return SD_SPI->DR;
}

__forceinline void SDSPI_Transmit_NBytes(uint8_t* data,int cnt){
	for(int i=0;i<cnt;i++)
	{
		PULL_SPI_BSY
		uint8_t temp;
		while(SPI_GET_FLAG(SD_SPI, SPI_FLAG_RXNE)){
			HD32_sd_temp=SD_SPI->DR;							//empty current buffer
		};
		#ifdef __SD_SPI_ADVTIMER
	//while((SD_SCK_GPIO_Port->MODER&0x0c00)!=0x0800)
	 while(((SD_SPI_ADV_TIMER->CR1&TIM_CR1_CEN)!=0)&&((SD_SPI_ADV_TIMER->CNT<SD_SPI_LOW_TIME)))
		//while(SD_CLK_STATE==1)
		{
			__nop();
		}//check if SCK in AF mode
#endif
	
		*((__IO uint8_t*)(&SD_SPI->DR))=data[i];		//transmit data by 8-bit
		while((!SPI_GET_FLAG(SD_SPI, SPI_FLAG_RXNE))){	//check if response data received (RXNE==1)
			__nop();
		}
		temp=SD_SPI->DR;	//offload RX
	}
}

__forceinline void SDSPI_Receive_NBytes(uint8_t* data,int cnt){
	for(int i=0;i<cnt;i++)
	{
		PULL_SPI_BSY
		//uint8_t temp;
		while(SPI_GET_FLAG(SD_SPI, SPI_FLAG_RXNE)){
			HD32_sd_temp=SD_SPI->DR;							//empty current buffer
		};
		*((__IO uint8_t*)(&SD_SPI->DR))=0xff;		//transmit data by 8-bit
		while((!SPI_GET_FLAG(SD_SPI, SPI_FLAG_RXNE))){	//check if response data received (RXNE==1)
			__nop();
		}
		data[i]=SD_SPI->DR;
	}
}
