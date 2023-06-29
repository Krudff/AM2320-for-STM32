/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "ESPDataLogger.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
float t,h;
uint8_t restart_stm;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float Value_Buffer[2];

void say(char arr[]){//for debugging; can remove this useless code
	printf("Kenneth the Debugger: %s\r\n",arr);
}

//temp and humid here (from datasheet)
unsigned int CRC16(uint8_t *ptr, uint8_t length){
	unsigned int crc = 0xFFFF;
	uint8_t s = 0x00;

	while(length--){
		crc ^= *ptr++;
		for(s = 0; s < 8; s++){
			if((crc & 0x01) != 0){
				crc >>= 1;
				crc ^= 0xA001;
			}else{
				crc >>= 1;
			}
		}
	}
	return crc;
}

 void AM2320_ReadData_HAL(void){//getting data from sensor, but HAL
 	uint8_t data_t[3];
 	uint8_t buf[8];

 	data_t[0]=0x03;
 	data_t[1]=0x00;
 	data_t[2]=0x04;

 	HAL_I2C_IsDeviceReady(&hi2c1,0xb8,2,1);
 	HAL_I2C_Master_Transmit(&hi2c1,0xb8,data_t,3,1);
 	HAL_I2C_Master_Receive(&hi2c1,0xb9,buf,8,2);

 	unsigned int Rcrc = buf[7] << 8;
 	Rcrc += buf[6];
 	if(Rcrc == CRC16(buf,6)){
 		unsigned int temperature = ((buf[4] & 0x7F) << 8) + buf[5];
 		t = temperature / 10.0;
 		t = (((buf[4] & 0x80) >> 7) == 1) ? ((t) * (-1)) : t;
 		unsigned int humidity = (buf[2] << 8) + buf[3];
 		h = humidity / 10.0;
 	}
 }

//void start_sequence(uint8_t dir){
//	I2C1->CR1 |= (1<<8); //Repeated start bit generation
//	while (!(I2C1->SR1 & (1<<0))){}//wait for start bit generation
//	(void) I2C1->SR1;//read Status Register 1 to reset SB (start bit)
//	I2C1->DR = dir ==0? 0xb8 : 0xb9;//send slave address and indicate whether tx or rx functionality
//	while(!(I2C1->SR1 & (1<<1)));//wait till address sent
//	(void) I2C1->SR1;//read Status Register 1 to reset ADDR (address sent)
//	(void) I2C1->SR2;//read and clear the SR2 register (to go back to initial/fresh state for the next transmission)
//}
//void AM2320_ReadCommand(void){//getting data from sensor, but Register level
//	I2C1->DR = 0x03;//function code
//	while(!(I2C1->SR1 & (1<<7))){};//wait till transmit mode DR empty
//	I2C1->DR = 0x00; //internal register address to read from
//	while(!(I2C1->SR1 & (1<<7))){};//wait
//	I2C1->DR = 0x04; //register length
//	while(!(I2C1->SR1 & (1<<7))){};//wait
//	I2C1->CR1 |= (1<<9); //stop bit generation
//
//}

void AM2320_ReadData_Register(){
	uint8_t buf[8];
	uint8_t i;

	/// Wake sensor ///
	I2C1->CR1 |= (1<<8); //Repeated Start Generation
	while (!(I2C1->SR1 & (1<<0))){}//wait for start bit generation
	(void) I2C1->SR1;//read Status Register 1 to reset SB (actually it resets whole SR1 but ehhh)
	I2C1->DR = 0xB8;//send sensor address to SDA (0x5C shifted left by one)
	HAL_Delay(1);
	I2C1->CR1 |= (1<<9);// stop bit generation
	// send read command //
	I2C1->CR1 |= (1<<8); //Repeated start bit generation
	while (!(I2C1->SR1 & (1<<0))){}//wait for start bit generation
	(void) I2C1->SR1;//read Status Register 1 to reset SB (start bit)
	I2C1->DR = 0xb8;//send slave address and indicate whether tx or rx functionality
	while(!(I2C1->SR1 & (1<<1)));//wait till address sent
	(void) I2C1->SR1;//read Status Register 1 to reset ADDR (address sent)
	(void) I2C1->SR2;//read and clear the SR2 register (to go back to initial/fresh state for the next transmission)

	I2C1->DR = 0x03;//function code
	while(!(I2C1->SR1 & (1<<7))){};//wait till transmit mode DR empty
	I2C1->DR = 0x00; //internal register address to read from
	while(!(I2C1->SR1 & (1<<7))){};//wait
	I2C1->DR = 0x04; //register length
	while(!(I2C1->SR1 & (1<<7))){};//wait
	I2C1->CR1 |= (1<<9); //stop bit generation
	HAL_Delay(1);
	// read data from sensor //
	I2C1->CR1 |= (1<<8); //Repeated start bit generation
	while (!(I2C1->SR1 & (1<<0))){}//wait for start bit generation
	(void) I2C1->SR1;//read Status Register 1 to reset SB (start bit)
	I2C1->DR = 0xb9;//send slave address and indicate whether tx or rx functionality
	while(!(I2C1->SR1 & (1<<1)));//wait till address sent
	(void) I2C1->SR1;//read Status Register 1 to reset ADDR (address sent)
	(void) I2C1->SR2;//read and clear the SR2 register (to go back to initial/fresh state for the next transmission)

	for(i=0; i<8; i++){
		while(!(I2C1->SR1 & (1<<6))){}//wait till receiver mode DR contains something
		buf[i]=I2C1->DR;//store data from sensor to buf array
	}
	I2C1->CR1 |= (1<<9);//stop bit generation
	// crc check //
	unsigned int Rcrc = buf[7] << 8;
	Rcrc += buf[6];
	if(Rcrc == CRC16(buf, 6)){
		unsigned int temperature = ((buf[4] & 0x7F) << 8) + buf[5];
		t = temperature /10.0;
		t = (((buf[4] & 0x80) >> 7)== 1) ? ((t) * (-1)) : t;
	unsigned int humidity = (buf[2] << 8) + buf[3];
	h = humidity / 10.0;
	}
}
void TIM2_Init(void){
	say("Timer initialized");
	RCC->APB1ENR |= (1<<0); // Enable clock for TIM2
	TIM2->PSC = 42000-1;    // Set PSC+1 = 16000 such that Feff = 1/0.001 Hz

	TIM2->ARR = 10000;        // Set timer to reset after CNT = 166, essentially after 166ms which is half the period.

	TIM2->DIER |= (1<<0);   // Enable timer interrupt generation

	NVIC->IP[TIM2_IRQn] =  (2 << 4); // Set priority to 2
	NVIC->ISER[TIM2_IRQn >> 5] |= (1 << (TIM2_IRQn % 32)); // Enable interrupt
	TIM2->SR &= ~(1<<0);

	TIM2->EGR |= (1<<0);
	TIM2->CR1 &= ~(1<<0);    // Disable timer, for now
}

void TIM2_IRQHandler(void){
	say("Interrupt handler activated");
	if(restart_stm){
		NVIC_SystemReset();
	}
	TIM2->SR &= ~(1<<0); // Clear UIF update interrupt flag
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  restart_stm = 0;

  printf("\n\n\n\n\n\n\n");
  TIM2_Init();
  say("ESP is initializing...");
  restart_stm = 1;
  TIM2->CR1 |= (1<<0);
  ESP_Init("EEE192-308","EEE192_Room308");
  restart_stm = 0;
  TIM2->CR1 &= ~(1<<0);    // Disable timer, for now
  say("ESP is ready!");
  int minutes = 1;//number of minutes between each transmission

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  restart_stm = 1;
	  TIM2->CR1 |= (1<<0);
	  say("Retrieving sensor data...");
//	  AM2320_ReadData_Register();
	  AM2320_ReadData_HAL();
	  say("Sensor data received!");
	  Value_Buffer[0] = t;
	  Value_Buffer[1] = h;
	  say("Preparing to send data...");
	  HAL_Delay(3000*minutes);
	  ESP_Send_Multi("CI4OHK76MHG5N7JL",2,Value_Buffer);
	  restart_stm = 0;
	  TIM2->CR1 &= ~(1<<0);    // Disable timer, for now
	  printf("Temperature: %f ; Humidity: %f\r\n",Value_Buffer[0],Value_Buffer[1]);
	  say("Data sent!");
	  //ESP_Init("sss","12345678");
	  HAL_Delay(57000*minutes);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */
//	RCC->AHB1ENR |= (1<<1);//GPIOB clock enable
//
//	GPIOB->MODER |= (2<<16) | (2<<18);// PB8 and PB9 as alternate funct
//	GPIOB->OTYPER |= (1<<8) | (1<<9); // PB8 PB9 as output open drain
//
//	GPIOB->OSPEEDR |= (3<<16) | (3<<18);// PB8 PB9 as high speed
//
//	GPIOB->AFR[1] |= (4<<0) | (4<<4); // PB8 PB9 as alt func 4 (I2C1)
//
//	RCC->APB1ENR |= (1<<21); // i2c clock enable
//
//	I2C1->CR1 |= (1<<15);//reset the I2C
//	I2C1->CR1 &= ~(1<<15);
//
//	I2C1->CR2 |= (16<<0); // input peripheral freq in MHz (16 MHz)
//
//	I2C1->CCR |= (80<<0);// (1000ns + 4000ns)/(1/16MHz)
//
//	I2C1->TRISE |= (17<<0); // configure rise time register
//
//	I2C1->CR1 |= (1<<0);// enable I2C

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
