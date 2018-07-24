
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"
#include "fatfs.h"
#include "usb_device.h"
#include "spi_nor_flash.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
extern SPI_HandleTypeDef hspi;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
#define NUM_COLORS 6

#define MIN_BTN_PRESS 100
#define LONG_BTN_PRESS 500
#define EXTND_BTN_PRESS 1000

volatile enum {
	NO_BUTTON,
	SHORT_BUTTON,
	LONG_BUTTON,
	EXTND_BUTTON
}button_state;
	

uint8_t led_pwm[] = {0,0,0};
uint16_t led_accum[] = {0,0,0};
uint32_t key_timer;
uint8_t mode;

uint8_t colors[NUM_COLORS][3]={
	{ 255, 0,   0   },//red
	{ 0,   255, 0   },//green
	{ 0,   0,   255 },//blue
	{ 255, 255, 0   },//yellow
	{ 255, 0,   255 },//magenta
	{ 0,   255, 255 },//cyan
	
};


uint8_t get_button(void){
	
	return GPIOB->IDR & (1<<0);
}

void led_on(uint8_t led){
	
	if(led == 0){ GPIOA->ODR &= ~(1 << 7); }
	if(led == 1){ GPIOB->ODR &= ~(1 << 3); }
	if(led == 2){ GPIOB->ODR &= ~(1 << 8); }
}

void led_off(uint8_t led){
	
	if(led == 0){ GPIOA->ODR |= (1 << 7); }
	if(led == 1){ GPIOB->ODR |= (1 << 3); }
	if(led == 2){ GPIOB->ODR |= (1 << 8); }
	
}

void cycle_color(void){

		mode++;
		
		//lightsOut = LIGHTS_OUT_AFTER_N_MS;
		
		if(mode > NUM_COLORS-1 ){
						
			mode = 0;
						
		}
		led_pwm[0] = colors[mode][0];
		led_pwm[1] = colors[mode][1];
		led_pwm[2] = colors[mode][2];
}
					
				
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  MX_FATFS_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

	printf("Dia duit!\n");

	init_spi_flash();
   
   // try some flash accesses
  // __ALIGN_BEGIN uint8_t txSpiBuf[16];
   __ALIGN_BEGIN uint8_t rxSpiBuf[256];
//   txSpiBuf[0] = CMD_READ_STATUS_REGISTER;
   //HAL_SPI_TransmitReceive( &hspi, txSpiBuf, rxSpiBuf, 2, 10);SPI2->CR1 &= ~SPI_CR1_SPE;
	 read_status_register(rxSpiBuf);
   
   printf("Status Reg:%02x\n", rxSpiBuf[1]);
   
//   txSpiBuf[0] = CMD_READ_ID;
//   HAL_SPI_TransmitReceive( &hspi, txSpiBuf, rxSpiBuf, 16, 10);SPI2->CR1 &= ~SPI_CR1_SPE;
//	 
	 
	 read_id(rxSpiBuf);
   
   printf("ID        :");
   for(int i = 0; i < 16; i++){
      
      printf("%02x",rxSpiBuf[i]);
      
   }printf("\n");SPI2->CR1 &= ~SPI_CR1_SPE;
	 

	 
	 
	 
		FATFS fs;           /* Filesystem object */
	 char fat_path[4];
    FIL fil;            /* File object */
    FRESULT res;        /* API result code */
    UINT bw;            /* Bytes written */
	 
	 if( FR_OK == f_mount( &fs, fat_path, 0)){
		 
		 if(FR_OK == f_mkfs( fat_path, 0, 0)){
			 
			 printf("mkfs success\n");
			 
		 }else{
			printf("f_mkfs failed\n");
		 }
		 
	 }else{
		 printf("f_mount failed\n");
	 } 
	 
	 read_status_register(rxSpiBuf);
   
   printf("Status Reg:%02x\n", rxSpiBuf[1]);
	 
	 // FA_CREATE_NEW will fail with FR_EXIST if file already exists
	 res = f_open ( &fil, "readme.txt", FA_WRITE | FA_CREATE_NEW);
	 
	 switch(res){
		 
		 case FR_OK:
			 printf("FR_OK \n");break;
		 case FR_DISK_ERR: 
			 printf("FR_DISK_ERR \n");break;
		 case FR_INT_ERR: 
			 printf("FR_INT_ERR \n");break;
		 case FR_NOT_READY: 
			 printf("FR_NOT_READY \n");break;
		 case FR_NO_FILE: 
			 printf("FR_NO_FILE \n");break;
		 case FR_NO_PATH: 
			 printf("FR_NO_PATH \n");break;
		 case FR_INVALID_NAME: 
			 printf("FR_INVALID_NAME \n");break;
		 case FR_DENIED: 
			 printf("FR_DENIED \n");break;
		 case FR_EXIST: 
			 printf("FR_EXIST \n");break;
		 case FR_INVALID_OBJECT: 
			 printf("FR_INVALID_OBJECT \n");break;
		 case FR_WRITE_PROTECTED: 
			 printf("ddd \n");break;
		 case FR_INVALID_DRIVE: 
			 printf("FR_WRITE_PROTECTED \n");break;
		 case FR_NOT_ENABLED: 
			 printf("FR_NOT_ENABLED \n");break;
		 case FR_NO_FILESYSTEM: 
			 printf("FR_NO_FILESYSTEM \n");break;
		 case FR_TIMEOUT: 
			 printf("FR_TIMEOUT \n");break;
		 case FR_LOCKED: 
			 printf("FR_LOCKED \n");break;
		 case FR_NOT_ENOUGH_CORE: 
			 printf("FR_NOT_ENOUGH_CORE \n");break;
		 case FR_TOO_MANY_OPEN_FILES:
			 printf("FR_TOO_MANY_OPEN_FILES \n");break;
		 
		 default:
			 printf("default \n");break;
	 }
	 
	for(int j = 0; j < 4; j++){
			 
		 read_random(j * 256, 256, rxSpiBuf);
		 printf("Sector %d :\n",j);
		 for(int i = 0; i < 256; i++){
				
				printf("%02x ",rxSpiBuf[i]);
				
		 }printf("\n");	
 }	 

	 
//    BYTE work[4096]; /* Work area (larger is better for processing time) */


//    /* Create FAT volume */
//    res = f_mkfs("",  0, 4096);
//    if (res){

//			/* Register work area */
//			f_mount(&fs, "", 0);

//			/* Create a file as new */
//			res = f_open(&fil, "hello.txt", FA_CREATE_NEW | FA_WRITE);
//			//if (res) ...

//			/* Write a message */
//			f_write(&fil, "Hello, World!\r\n", 15, &bw);
//			//if (bw != 15) ...

//			/* Close the file */
//			f_close(&fil);

//			/* Unregister work area */
//			f_mount(0, "", 0);
//		}
//	 
	 
	 
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if(button_state){
			switch(button_state){
				case SHORT_BUTTON:
				case LONG_BUTTON:
				case EXTND_BUTTON:
					
				cycle_color();
				printf("butt\n");
				break;
					
				case NO_BUTTON:
					default:
						__nop();
				
			}
			
			button_state = NO_BUTTON;
		}

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* SPI2 init function */
static void MX_SPI2_Init(void)
{

  /* SPI2 parameter configuration*/
  hspi.Instance = SPI2;
  hspi.Init.Mode = SPI_MODE_MASTER;
  hspi.Init.Direction = SPI_DIRECTION_2LINES;
  hspi.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi.Init.CRCPolynomial = 7;
  hspi.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 921600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Red_GPIO_Port, Red_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Green_Pin|Blue_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Red_Pin */
  GPIO_InitStruct.Pin = Red_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Red_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Button_Pin */
  GPIO_InitStruct.Pin = Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Green_Pin Blue_Pin */
  GPIO_InitStruct.Pin = Green_Pin|Blue_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_SYSTICK_Callback(void)
{
	static struct {
		int prev : 1;
		int curr : 1;
	}btn_state;
  
	btn_state.curr = get_button();
	
	// check for rising edge on button: end of press or debounce
	if(btn_state.curr != btn_state.prev){
		
		if(btn_state.curr){
			
			if(key_timer > EXTND_BTN_PRESS){
				
				button_state = EXTND_BUTTON;
				
			}else if( key_timer > LONG_BTN_PRESS){
				
				button_state = LONG_BUTTON;
				
			}else if(key_timer > MIN_BTN_PRESS){
				
				button_state = SHORT_BUTTON;
			}
		}
		btn_state.prev = btn_state.curr;
	}
	
	if(!btn_state.curr){
		key_timer++;
	}else{
		key_timer = 0;
	}

	
	for(int i = 0; i< 3; i++){
		
		led_accum[i] += led_pwm[i];
		if( led_accum[i] > 256 ){
			led_accum[i] -= 256;
			led_on(i);
		}else{
			led_off(i);
		}
	}	
	
	
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
