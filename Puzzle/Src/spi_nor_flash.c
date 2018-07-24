#include "spi_nor_flash.h"
#include "n25q256_commands.h"

SPI_HandleTypeDef hspi;


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
   hspi.Init.NSS               = SPI_NSS_HARD_OUTPUT;
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
	
	HAL_SPI_Transmit( &hspi, &cmd, 1, 10);
}

result_t read_status_register( uint8_t* reg_val ){
	
	__ALIGN_BEGIN uint8_t cmd[2] = {CMD_READ_STATUS_REGISTER, 0};
	if(HAL_OK == HAL_SPI_TransmitReceive( &hspi, cmd, cmd, 2, 10)){
		*reg_val = cmd[1];
		return RESULT_OK;
	}else{
		return RESULT_ERROR;
	}
	
}
// read from any address,
// can potentially read out entire memory
result_t read_random(uint32_t address, uint32_t size, uint8_t* dest_buf){
	
	return RESULT_OK;
}


//
result_t erase_sector(uint32_t sector){
	
	return RESULT_OK;
}

// 
result_t erase_subsector(uint32_t subsector){

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

		return RESULT_OK;
}

