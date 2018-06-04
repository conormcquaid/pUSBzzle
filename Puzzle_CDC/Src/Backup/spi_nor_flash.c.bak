#include "spi_nor_flash.h"


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

