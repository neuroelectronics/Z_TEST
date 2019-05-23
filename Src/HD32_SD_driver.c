#include "HD32_SD_driver.h"
#include "main.h"

int HD32_SD_init(SPI_HandleTypeDef* hspi){
	
	// set spi speed <400KHz
	SDSPI_lowSpeed_Init(hspi);
	for(int i=0;i<5000;i++){
		uint8_t sig_buff[1]={0xff};	//set DI high
		HAL_SPI_Transmit(hspi,sig_buff,1,1000);
	}
	uint8_t cmd0_buf[6]={SD_CMD0};
	int resp=0;
	int cnt=0;
	while(resp!=0x01){
		PIN_RESET(SD_CS);//Pull CS low to issue a command
		HAL_Delay(10);
		resp=HD32_SD_sendCmd(cmd0_buf);		//Reset card with CMD0
		if(cnt>50)
		{
			return 1;
		}
	}
	uint8_t cmd8_buf[6]={SD_CMD8};
	while(HD32_SD_sendCmdR7(cmd8_buf)!=0x01);		//Check voltage supply with CMD8
	uint8_t cmd41_buf[6]={SD_CMD41};
	while(HD32_SD_sendACmd(cmd41_buf)!=0x00);
	
	uint8_t cmd58_buf[6]={SD_CMD58};
	uint32_t OCR;
	while(HD32_SD_sendCmdR3(cmd58_buf,&OCR)!=0x00);		//Read OCR with cmd58

	//HD32_SD_highSpeed(hspi,CSS_port,CSS_pin);
	//uint8_t cmd16_buf[6]={SD_CMD16};
	//while(HD32_SD_sendCmd(hspi,CSS_port,CSS_pin,cmd16_buf)!=0x00);		//set block size to 512
	
	SDSPI_highSpeed_Init(hspi);
	return 0;
}

int HD32_SD_readCSD(SPI_HandleTypeDef* hspi,CE32_systemParam* sys){
	uint8_t cmd_buf=0x49;
	uint8_t data[16];
	uint8_t buf_checkRsp[1]={0xff};
	uint8_t resp=0xff;
	//SDSPI_lowSpeed_Init(hspi);
	PIN_RESET(SD_CS); //Pull CS LOW
	SDSPI_TransmitReceive_Byte(cmd_buf);
	cmd_buf=0;
	SDSPI_TransmitReceive_Byte(cmd_buf);
	SDSPI_TransmitReceive_Byte(cmd_buf);
	SDSPI_TransmitReceive_Byte(cmd_buf);
	SDSPI_TransmitReceive_Byte(cmd_buf);
	cmd_buf=0xff;
	SDSPI_TransmitReceive_Byte(cmd_buf);
	while(resp==0xff){
		resp=SDSPI_TransmitReceive_Byte(0xff);//check command response
	}
	if(resp!=0x00){
		//PIN_SET(SD_CS); //Pull CS High
		return -1;
	}
	while(resp!=0xFE){				//search for read start token
		resp=SDSPI_TransmitReceive_Byte(0xff);
	}
	for(int i=0;i<16;i++)
	{
		data[i]=SDSPI_TransmitReceive_Byte(0xff);//receive data
	}
	
	for(int i=0;i<2;i++)
	{
		SDSPI_TransmitReceive_Byte(0xff);//dummy crc
	}
	//PIN_SET(SD_CS); //Pull CS High
	sys->SD_capacity=1000*((((uint32_t)data[7])<<16)+(((uint32_t)data[8])<<8)+((uint32_t)data[9]));
	
	HD32_SD_init(hspi);								//re-init
	//SDSPI_highSpeed_Init(hspi);
	return 0;
}

int HD32_SD_readSingleBlock(uint32_t addr,uint8_t *data){
	uint8_t buf[1]={0x51};
	uint8_t buf_checkRsp[1]={0xff};
	uint8_t resp=0xff;
	PIN_RESET(SD_CS); //Pull CS LOW
	SDSPI_TransmitReceive_Byte(0x51);
	SDSPI_TransmitReceive_Byte(*(((uint8_t*)&addr+3)));
	SDSPI_TransmitReceive_Byte(*(((uint8_t*)&addr+2)));
	SDSPI_TransmitReceive_Byte(*(((uint8_t*)&addr+1)));
	SDSPI_TransmitReceive_Byte(*(((uint8_t*)&addr)));
	SDSPI_TransmitReceive_Byte(0x01);
	while(resp==0xff){
		resp=SDSPI_TransmitReceive_Byte(0xff);//check command response
	}
	if(resp!=0x00){
		//PIN_SET(SD_CS); //Pull CS High
		return -1;
	}
	//for(int i=0;i<10;i++) __nop();
	resp=HD32_SD_read512B(data);
	SDSPI_TransmitReceive_Byte(0xff);//dummy
	//PIN_SET(SD_CS); //Pull CS High
	return resp;
}

int HD32_SD_read512B(uint8_t *data){
	uint8_t buf_checkRsp[1]={0xff};
	uint8_t resp=0xff;
	//uint8_t cnt=8;

	while(resp!=SD_CMD17_TOKEN_START){				//search for read start token
		resp=SDSPI_TransmitReceive_Byte(0xff);
	}
	SDSPI_Receive_NBytes(data,512);//receive data
	uint8_t crc[2]={0xff,0xff};
	SDSPI_Receive_NBytes(crc,2);//receive CRC
 
	return 0;
}

int HD32_SD_writeBlock(uint32_t addr){

	uint8_t resp=0xff;
	//uint8_t cnt=0;
	PIN_RESET(SD_CS); //Pull CS low
	//PIN_RESET(SD_CS); //Pull CS LOW
//	HAL_SPI_Transmit(hspi,buf,1,1000);
//	HAL_SPI_Transmit(hspi,((uint8_t*)&addr)+3,1,1000);
//	HAL_SPI_Transmit(hspi,((uint8_t*)&addr)+2,1,1000);
//	HAL_SPI_Transmit(hspi,((uint8_t*)&addr)+1,1,1000);
//	HAL_SPI_Transmit(hspi,((uint8_t*)&addr),1,1000);
//	buf[0]=0x01;
//  HAL_SPI_TransmitReceive(hspi,buf,&resp,1,1000);
	SDSPI_TransmitReceive_Byte(0x59);
	SDSPI_TransmitReceive_Byte(*(((uint8_t*)&addr)+3));
	SDSPI_TransmitReceive_Byte(*(((uint8_t*)&addr)+2));
	SDSPI_TransmitReceive_Byte(*(((uint8_t*)&addr)+1));
	SDSPI_TransmitReceive_Byte(*(((uint8_t*)&addr)));
	SDSPI_TransmitReceive_Byte(0x01);
	
	while(resp==0xff){
		resp=SDSPI_TransmitReceive_Byte(0xff);
		//cnt++;
//		HAL_SPI_TransmitReceive(hspi,buf_checkRsp,&resp,1,1000);
	}
	if(resp!=0x00){
		//PIN_SET(SD_CS); //Pull CS High
		return -1;
	}
	SDSPI_TransmitReceive_Byte(0xff);
	//HAL_SPI_Transmit(hspi,buf_checkRsp,sizeof(buf_checkRsp),1000); //send extra 8 clk cycles for sequence initialization
	//PIN_SET(SD_CS);//Pull CS high
	////PIN_SET(SD_CS); //Pull CS High
	return resp;
}

int HD32_SD_writeSingleBlock(uint32_t addr,uint8_t *data){


	uint8_t resp=0xff;
	//uint8_t cnt=0;
	PIN_RESET(SD_CS); //Pull CS low
	//PIN_RESET(SD_CS); //Pull CS LOW
	SDSPI_TransmitReceive_Byte(0x58);
	SDSPI_TransmitReceive_Byte(*(((uint8_t*)&addr)+3));
	SDSPI_TransmitReceive_Byte(*(((uint8_t*)&addr)+2));
	SDSPI_TransmitReceive_Byte(*(((uint8_t*)&addr)+1));
	SDSPI_TransmitReceive_Byte(*(((uint8_t*)&addr)));
	SDSPI_TransmitReceive_Byte(0x01);

	while(resp==0xff){
		//cnt++;
		resp=SDSPI_TransmitReceive_Byte(0xff);
	}
	if(resp!=0x00){
		//PIN_SET(SD_CS); //Pull CS High
		return -1;
	}
	SDSPI_TransmitReceive_Byte(0xff);			//send extra 8 clk cycles for sequence initialization
	//PIN_SET(SD_CS); //Pull CS High
	//PIN_SET(SD_CS); //Pull CS High
	PIN_RESET(SD_CS); //Pull CS Low
	//for(int i=0;i<10;i++) __nop();
	
	SDSPI_TransmitReceive_Byte(SD_CMD17_TOKEN_START);
	SDSPI_Transmit_NBytes(data,512);//send data
	SDSPI_TransmitReceive_Byte(0xff);
	SDSPI_TransmitReceive_Byte(0xff);
	resp=SDSPI_TransmitReceive_Byte(0xff);
	//HAL_SPI_TransmitReceive(hspi,&buf_checkRsp,&resp,1,1000);   //check response token
	if(resp!=0xE5){
		return -1;
	}
	resp=0x00;
	//cnt=0;
	while(resp==0x00){
		//cnt++;
		resp=SDSPI_TransmitReceive_Byte(0xff);
		//HAL_SPI_TransmitReceive(hspi,&buf_checkRsp,&resp,1,1000);
		//resp=*((__IO uint8_t *)&hspi->Instance->DR);
	}
	SDSPI_TransmitReceive_Byte(0xff);//send extra 8 clk cycles for sequence initialization
	//PIN_SET(SD_CS); //Pull CS High
	return 0;
}

int HD32_SD_writeSingleBlock_DMA(uint32_t addr,uint8_t *data){
	//SDSPI_DMA_Disable(hspi);
	uint8_t resp=0xff;
	//uint8_t cnt=0;
	PIN_RESET(SD_CS); //Pull CS low
	//PIN_RESET(SD_CS); //Pull CS LOW
	SDSPI_TransmitReceive_Byte(0x58);
	SDSPI_TransmitReceive_Byte(*(((uint8_t*)&addr)+3));
	SDSPI_TransmitReceive_Byte(*(((uint8_t*)&addr)+2));
	SDSPI_TransmitReceive_Byte(*(((uint8_t*)&addr)+1));
	SDSPI_TransmitReceive_Byte(*(((uint8_t*)&addr)));
	SDSPI_TransmitReceive_Byte(0x01);
	
	while(resp==0xff){
		//cnt++;
		resp=SDSPI_TransmitReceive_Byte(0xff);
	}
	if(resp!=0x00){
		//PIN_SET(SD_CS); //Pull CS high
		////PIN_SET(SD_CS); //Pull CS High
		return -1;
	}
	SDSPI_TransmitReceive_Byte(0xff);//send extra 8 clk cycles for sequence initialization
	//PIN_SET(SD_CS); //Pull CS High
	//PIN_SET(SD_CS); //Pull CS High
	PIN_RESET(SD_CS); //Pull CS LOW
	SDSPI_TransmitReceive_Byte(SD_CMD17_TOKEN_START);////send block write start token
	SD_DMA_TX->M0AR=(uint32_t)data;
	SD_DMA_TX->NDTR=512;
	SD_DMA_TX->CR|=DMA_IT_TC;
	SDSPI_DMA_Enable();
	//DMA_ENABLE(SD_DMA_TX);	
	return 0;
}

int HD32_SD_write512B(uint8_t *data){
	uint8_t buf[1]={SD_CMD25_TOKEN_START};
	uint8_t buf_checkRsp[1]={0xff};
	uint8_t resp;
	SDSPI_TransmitReceive_Byte(SD_CMD25_TOKEN_START);//send start token
	SDSPI_Transmit_NBytes(data,512);//send data
	uint8_t crc[2]={0xff,0xff};
	SDSPI_Transmit_NBytes(crc,2);//send crc
	resp=SDSPI_TransmitReceive_Byte(0xff);//check response token
	if(resp!=0xE5){
		return -1;
	}
	resp=0x00;
	while(resp!=0xff){
		resp=SDSPI_TransmitReceive_Byte(0xff);//check response token
	}
	return 0;
}

int HD32_SD_write515B(uint8_t *data){
	uint8_t resp;
	uint8_t buf_checkRsp=0xff;
	PIN_RESET(SD_CS); //Pull CS LOW
	//SET_BIT(hspi->Instance->CR2, SPI_RXFIFO_THRESHOLD); //set read width to 8 bit mode
	SDSPI_Transmit_NBytes(data,515);//send data
	resp=SDSPI_TransmitReceive_Byte(0xff);//check response token
	if(resp!=0xE5){
		//PIN_SET(SD_CS); //Pull CS High
		return 1;
	}
	resp=SDSPI_TransmitReceive_Byte(0xff);//
	while(resp==0x00){
		resp=SDSPI_TransmitReceive_Byte(0xff);//
	}
	//PIN_SET(SD_CS); //Pull CS High
	return 0;
}

void HD32_SD_write515B_DMA(uint8_t *data){

	PIN_RESET(SD_CS); //Pull CS LOW
	//HAL_SPI_Transmit_DMA(hspi,data,515); //send data
	SD_DMA_TX->M0AR=(uint32_t)data;
	SD_DMA_TX->NDTR=515;
	SD_DMA_TX->CR|=DMA_IT_TC;
	DMA_ENABLE(SD_DMA_TX);	
}

void HD32_SD_write_DMA(uint8_t *data,uint16_t size){
	SDSPI_DMA_Disable();
	//PIN_RESET(SD_CS); //Pull CS LOW
	PIN_RESET(SD_CS);
	while(SDSPI_TransmitReceive_Byte(0xff)!=0xff){};//check busy
	SDSPI_TransmitReceive_Byte(SD_CMD25_TOKEN_START);////send block write start token
	//HAL_SPI_Transmit_DMA(hspi,data,515); //send data
	SD_DMA_TX->M0AR=(uint32_t)data;
	SD_DMA_TX->NDTR=size;
	SD_DMA_TX->CR|=DMA_IT_TC;
	SDSPI_DMA_Enable();
	//DMA_ENABLE(SD_DMA_TX);	
}

int HD32_SD_writeBlock_DMA_Check(){
	uint8_t resp;
	SDSPI_DMA_Disable();
	SDSPI_TransmitReceive_Byte(0xff);//send 2 crc
	SDSPI_TransmitReceive_Byte(0xff);//send 2 crc
	resp=SDSPI_TransmitReceive_Byte(0xff);//check token
//	HAL_SPI_Transmit(hspi,&buf_checkRsp,1,1000);
//	HAL_SPI_Transmit(hspi,&buf_checkRsp,1,1000);
//	HAL_SPI_TransmitReceive(hspi,&buf_checkRsp,&resp,1,1000);
	if(resp!=0xE5){
		//PIN_SET(SD_CS);//Pull CS High
		return -1;
	}
	resp=SDSPI_TransmitReceive_Byte(0xff);//check busy
	//uint32_t cnt=MAX_WAIT_CYC;
	while(resp!=0xff){
		resp=SDSPI_TransmitReceive_Byte(0xff);//check busy
		/*
		cnt--;
		if(cnt==0){
			//PIN_SET(SD_CS);//Pull CS High
			////PIN_SET(SD_CS);//Pull CS High
			return -2;
		}*/
	}
	//PIN_SET(SD_CS);
	////PIN_SET(SD_CS);//Pull CS High

	return 0;
}



int HD32_SD_write515B_DMA_Check(){
	uint8_t resp;
	uint8_t buf_checkRsp=0xff;
	resp=SDSPI_TransmitReceive_Byte(0xff);//check response token
	if(resp!=0xE5){
		//PIN_SET(SD_CS); //Pull CS High
		return -1;
	}
	resp=SDSPI_TransmitReceive_Byte(0xff);//check response token
	while(resp==0x00){
		resp=SDSPI_TransmitReceive_Byte(0xff);//check response token
	}
	//PIN_SET(SD_CS); //Pull CS High
	return resp;
}

int HD32_SD_preErase(uint32_t blkNum){
	uint8_t cmd_buf[6]={SD_CMD23};
	uint32_t *p;
	p=(uint32_t*)(cmd_buf+1);
	if(blkNum>0x0FFF){
		blkNum=0x0FFF;
	}
	*p=blkNum;
	return HD32_SD_sendACmd(cmd_buf);	//Initialize card

}
/*
int HD32_SD_stopTrans(){
	uint8_t cmd0_buf[1]={SD_CMD25_TOKEN_END};
	uint8_t resp=0;
	uint8_t buf_checkRsp[1]={0xff};
	PIN_RESET(SD_CS); //Pull CS LOW
	HAL_SPI_Transmit(hspi,cmd0_buf,1,1000); //send data
	cmd0_buf[0]=0xff;
	HAL_SPI_Transmit(hspi,cmd0_buf,1,1000); //send data
	while(resp!=0xff){
		HAL_SPI_TransmitReceive(hspi,cmd0_buf,&resp,1,1000);   //check response token
	}
	HAL_SPI_Transmit(hspi,buf_checkRsp,sizeof(buf_checkRsp),1000); //send extra 8 clk cycles for sequence initialization
	//PIN_SET(SD_CS);//Pull CS High
	//while(HD32_SD_sendCmd(hspi,CSS_port,CSS_pin,cmd0_buf)!=0x01);		//Stop continuous transaction by CMD12
	return 0;
}
*/
int HD32_SD_stopTrans(){
	uint8_t resp=0;
	PIN_RESET(SD_CS);												//Pull CS LOW
	SDSPI_TransmitReceive_Byte(SD_CMD25_TOKEN_END);
	SDSPI_TransmitReceive_Byte(0xff);
	while(resp!=0xff){
		resp=SDSPI_TransmitReceive_Byte(0xff);
	}
	SDSPI_TransmitReceive_Byte(0xff);//send extra 8 clk cycles for sequence initialization
	//PIN_SET(SD_CS); //Pull CS High
	return 0;
}

int HD32_SD_sendCmdR3(uint8_t* buf,uint32_t* OCR){
	uint8_t buf_checkRsp[4]={0xff,0xff,0xff,0xff};
	uint8_t resp=0xff;
	PIN_RESET(SD_CS); //Pull CS low
	SDSPI_Transmit_NBytes(buf,6);
	while(resp==0xff){
		resp=SDSPI_TransmitReceive_Byte(0xff);
	}
	SDSPI_TransmitReceive_Byte(*(((uint8_t*) OCR)+3));
	SDSPI_TransmitReceive_Byte(*(((uint8_t*) OCR)+2));
	SDSPI_TransmitReceive_Byte(*(((uint8_t*) OCR)+1));
	SDSPI_TransmitReceive_Byte(*(((uint8_t*) OCR)));

	SDSPI_TransmitReceive_Byte(0xff);; //send extra 8 clk cycles for sequence initialization
	//PIN_SET(SD_CS); //Pull CS high

	return resp;
}

int HD32_SD_sendCmdR7(uint8_t* buf){
	uint8_t buf_checkRsp[4]={0xff,0xff,0xff,0xff};
	uint8_t resp=0xff;
	PIN_RESET(SD_CS); //Pull CS low
	SDSPI_Transmit_NBytes(buf,6);
	while(resp==0xff){
		resp=SDSPI_TransmitReceive_Byte(0xff);
	}
	SDSPI_Transmit_NBytes(buf_checkRsp,4);
	SDSPI_TransmitReceive_Byte(0xff); //send extra 8 clk cycles for sequence initialization
	//PIN_SET(SD_CS);//Pull CS high
	return resp;
}

int HD32_SD_sendCmd(uint8_t* buf){
//	uint8_t buf_checkRsp[1]={0xff};
	uint8_t resp=0xff;
	uint32_t cnt=0;
	PIN_RESET(SD_CS);//Pull CS low
	//PIN_RESET(SD_CS); 
	SDSPI_TransmitReceive_Byte(buf[0]);
	SDSPI_TransmitReceive_Byte(buf[1]);
	SDSPI_TransmitReceive_Byte(buf[2]);
	SDSPI_TransmitReceive_Byte(buf[3]);
	SDSPI_TransmitReceive_Byte(buf[4]);
	SDSPI_TransmitReceive_Byte(buf[5]);
	//HAL_SPI_Transmit(hspi,buf,6,1000);
	while((resp==0xff)&&(cnt++<10000)){
		resp=SDSPI_TransmitReceive_Byte(0xff);
		//HAL_SPI_TransmitReceive(hspi,buf_checkRsp,&resp,1,1000);
		//cnt++;
	}
	SDSPI_TransmitReceive_Byte(0xff);
	//HAL_SPI_Transmit(hspi,buf_checkRsp,1,1000); //send extra 8 clk cycles for sequence initialization
	////PIN_SET(SD_CS);//Pull CS high
	//PIN_SET(SD_CS);//Pull CS high
	return resp;
}

int HD32_SD_sendACmd(uint8_t* buf1){
	const uint8_t buf[6]={SD_CMD55}; //send cmd0 with CRC to switch to SPI mode , add an extra 8 clocks
//	uint8_t buf_checkRsp[1]={0xff};
	uint8_t resp=0xff;
	uint32_t cnt=0;
	PIN_RESET(SD_CS);//Pull CS low

	SDSPI_TransmitReceive_Byte(buf[0]);
	SDSPI_TransmitReceive_Byte(buf[1]);
	SDSPI_TransmitReceive_Byte(buf[2]);
	SDSPI_TransmitReceive_Byte(buf[3]);
	SDSPI_TransmitReceive_Byte(buf[4]);
	SDSPI_TransmitReceive_Byte(buf[5]);
	while((resp==0xff)&&(cnt++<10000)){
		resp=SDSPI_TransmitReceive_Byte(0xff);
	}

	SDSPI_TransmitReceive_Byte(buf1[0]);
	SDSPI_TransmitReceive_Byte(buf1[1]);
	SDSPI_TransmitReceive_Byte(buf1[2]);
	SDSPI_TransmitReceive_Byte(buf1[3]);
	SDSPI_TransmitReceive_Byte(buf1[4]);
	SDSPI_TransmitReceive_Byte(buf1[5]);
	resp=0xff;

	while((resp==0xff)&&(cnt++<10000)){
		resp=SDSPI_TransmitReceive_Byte(0xff);
	}
	SDSPI_TransmitReceive_Byte(0xff);//send extra 8 clk cycles for sequence initialization

	//PIN_SET(SD_CS);//Pull CS high
	return resp;
}

static int SDSPI_lowSpeed_Init(SPI_HandleTypeDef* hspi){
  //hspi->Instance = SPI3;
  hspi->Init.Mode = SPI_MODE_MASTER;
  hspi->Init.Direction = SPI_DIRECTION_2LINES;
  hspi->Init.DataSize = SPI_DATASIZE_8BIT;
  hspi->Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi->Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi->Init.NSS = SPI_NSS_SOFT;
  hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
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

static int SDSPI_highSpeed_Init(SPI_HandleTypeDef* hspi){
  //hspi->Instance = SPI3;
  hspi->Init.Mode = SPI_MODE_MASTER;
  hspi->Init.Direction = SPI_DIRECTION_2LINES;
  hspi->Init.DataSize = SPI_DATASIZE_8BIT;
  hspi->Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi->Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi->Init.NSS = SPI_NSS_SOFT;
  hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi->Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi->Init.TIMode = SPI_TIMODE_DISABLE;
  hspi->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi->Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(hspi) != HAL_OK)
  {
    return 1;
  }
	__HAL_SPI_ENABLE(hspi);
	//SDSPI_DMA_Init(hspi);
	return 0;
}

static int SDSPI_midSpeed_Init(SPI_HandleTypeDef* hspi){
  //hspi->Instance = SPI3;
  hspi->Init.Mode = SPI_MODE_MASTER;
  hspi->Init.Direction = SPI_DIRECTION_2LINES;
  hspi->Init.DataSize = SPI_DATASIZE_8BIT;
  hspi->Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi->Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi->Init.NSS = SPI_NSS_SOFT;
  hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi->Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi->Init.TIMode = SPI_TIMODE_DISABLE;
  hspi->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi->Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(hspi) != HAL_OK)
  {
    return 1;
  }
	__HAL_SPI_ENABLE(hspi);
	//SDSPI_DMA_Init(hspi);
	return 0;
}


void HD32_SD_dmaTransmit(uint32_t cnt){
	SD_DMA_TX->NDTR=cnt;
	DMA_ENABLE(SD_DMA_TX);	
}

int HD32_SD_highSpeed(){
	uint8_t cmd6_buf[6]={SD_CMD6};
	while(HD32_SD_sendCmd(cmd6_buf)!=0x00){__NOP;};		//High speed mode on

	for(int i=0;i<64;i++){
		SDSPI_TransmitReceive_Byte(0xff);//send 512 clocks to get status out		
	}
	return 0;
}

int HD32_SD_SpeedClassCtrl(uint8_t state){
	uint8_t cmd20_buf[6]={0x54,state,0x00,0x00,0x00,0x01};
	//uint8_t resp=0xff;
	while(HD32_SD_sendCmd(cmd20_buf)!=0x00){__NOP;};	
	
	return 0;
}


void HD32_Recording_Server()
{

}


