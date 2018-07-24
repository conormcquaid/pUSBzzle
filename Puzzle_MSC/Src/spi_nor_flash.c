#include "spi_nor_flash.h"
#include "n25q256_commands.h"

SPI_HandleTypeDef hspi;

__ALIGN_BEGIN uint8_t rw_buf[516];

#define NSS_LOW GPIOB->ODR &= ~(1<<12);
#define NSS_HIGH GPIOB->ODR |= (1<<12);

void init_spi_flash(void){
   
   __HAL_RCC_SPI2_CLK_ENABLE();
   
   hspi.Instance               = SPI2;
   
   hspi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
   hspi.Init.CLKPhase          = SPI_PHASE_1EDGE;
   hspi.Init.CLKPolarity       = SPI_POLARITY_LOW;
   hspi.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
//   hspi.Init.CRCLength         = 
//   hspi.Init.CRCPolynomial     = 
   hspi.Init.DataSize          = SPI_DATASIZE_8BIT;
   hspi.Init.Direction         = SPI_DIRECTION_2LINES;
   hspi.Init.FirstBit          = SPI_FIRSTBIT_MSB;
   hspi.Init.Mode              = SPI_MODE_MASTER;
   hspi.Init.NSS               = SPI_NSS_SOFT;
   hspi.Init.NSSPMode          = SPI_NSS_PULSE_DISABLE;
   hspi.Init.TIMode            = SPI_TIMODE_DISABLE;
   
   HAL_SPI_Init( &hspi );
   
   HAL_NVIC_SetPriority(SPI2_IRQn, 2, 2);
   HAL_NVIC_EnableIRQ(SPI2_IRQn);
   
}

void SPI2_IRQHandler(void){
   
   HAL_SPI_IRQHandler( &hspi );
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi){
   SPI2->CR1 &= ~SPI_CR1_SPE;
}
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi){
   SPI2->CR1 &= ~SPI_CR1_SPE;
}
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi){
   SPI2->CR1 &= ~SPI_CR1_SPE;
}
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi){
   SPI2->CR1 &= ~SPI_CR1_SPE;
}

// must be invoked before
// program, erase or write
void write_enable(uint8_t enable){
	
	__ALIGN_BEGIN uint8_t cmd = enable ? CMD_WRITE_ENABLE : CMD_WRITE_DISABLE;
	
	NSS_LOW;
	
	HAL_SPI_Transmit( &hspi, &cmd, 1, 10);
	
	NSS_HIGH;
	//__nop();__nop();__nop();__nop();__nop();__nop();
//	NSS_LOW;
//	SPI2->CR1 |= SPI_CR1_SPE;
//	SPI2->DR = cmd;
//	while( (SPI2->SR & SPI_FLAG_RXNE) == 0){ /* wait for tx to complete */ }__nop();__nop();__nop();__nop();__nop();__nop();
//	NSS_HIGH;
//	//
//	__nop();__nop();__nop();
}

result_t read_status_register( uint8_t* reg_val ){
	
	__ALIGN_BEGIN uint8_t cmd[1] = {CMD_READ_STATUS_REGISTER};
	NSS_LOW;
	HAL_StatusTypeDef ret;
	ret = HAL_SPI_Transmit( &hspi, cmd, 1, 10);
	ret = HAL_SPI_TransmitReceive( &hspi, reg_val, reg_val, 1, 10);
	NSS_HIGH;
	if(HAL_OK == ret){
		return RESULT_OK;
	}else{
		return RESULT_ERROR;
	}
	
}

result_t read_id( uint8_t* buf ){
	
	__ALIGN_BEGIN uint8_t cmd[1] = {CMD_READ_ID};
	NSS_LOW;
	HAL_StatusTypeDef ret; 
	ret = HAL_SPI_Transmit( &hspi, cmd, 1, 10);
	ret = HAL_SPI_TransmitReceive( &hspi, buf, buf, 16, 10);
	NSS_HIGH;
	if(HAL_OK == ret){
		return RESULT_OK;
	}else{
		return RESULT_ERROR;
	}	
	
}
// read from any address,
// can potentially read out entire memory
result_t read_random(uint32_t address, uint32_t size, uint8_t* dest_buf){
	
	__ALIGN_BEGIN uint8_t cmd[4] = {  CMD_READ,
		(address & 0x00ff0000) >> 16,
		(address & 0x0000ff00) >> 8,
		(address & 0x000000ff)
	};
	
	NSS_LOW;
	HAL_StatusTypeDef ret; 
	ret = HAL_SPI_Transmit( &hspi, cmd, 4, 10);
	ret = HAL_SPI_TransmitReceive( &hspi, dest_buf, dest_buf, size, 10);
	NSS_HIGH;
	if(HAL_OK == ret){
		return RESULT_OK;
	}else{
		return RESULT_ERROR;
	}	
}


//
result_t erase_sector(uint32_t sector){
	__ALIGN_BEGIN uint8_t cmd[4] = {  CMD_SUBSECTOR_ERASE,
		((sector * ERASE_BLOCK_SIZE) & 0x00ff0000) >> 16,
		(sector & 0x0000ff00) >> 8,
		(sector & 0x000000ff)
	};
	
	//write_enable(1);	
	
	NSS_LOW;
	HAL_StatusTypeDef ret; 
	ret = HAL_SPI_Transmit( &hspi, cmd, 4, 10);
	NSS_HIGH;
	if(HAL_OK == ret){
		return RESULT_OK;
	}else{
		return RESULT_ERROR;
	}		
}

// 
result_t erase_subsector(uint32_t subsector){

	//write_enable(1);
	return RESULT_OK;
}

// page      = 256bytes
// subsector = 4k
// sector    = 64k
// program limitation is 1 page
// In order to preserve existing data in same subsector as destination page,
// subsector data must be cached
// Also, beware of writes that cross a page or subsector boundary
result_t program(uint32_t address, uint16_t size, uint8_t* source){

	//write_enable(1);
		__ALIGN_BEGIN uint8_t cmd[4] = {  CMD_PAGE_PROGRAM ,
		(address  & 0x00ff0000) >> 16,
		(address & 0x0000ff00) >> 8,
		(address & 0x000000ff)
	};
		NSS_LOW;
	HAL_StatusTypeDef ret; 
	ret = HAL_SPI_Transmit( &hspi, cmd, 4, 10);
	ret = HAL_SPI_Transmit( &hspi, source, size, 10);
	NSS_HIGH;
	if(HAL_OK == ret){
		return RESULT_OK;
	}else{
		return RESULT_ERROR;
	}		
}
result_t wait_for_idle(void){
	
	//TODO implement timeout logic
	uint8_t status;
	
	do{
		read_status_register( &status ); 
		__nop();__nop();__nop();__nop();__nop();__nop();__nop();
	}while (status & SR_WRITE_IN_PROGRESS );
	
	return RESULT_OK;	
}



