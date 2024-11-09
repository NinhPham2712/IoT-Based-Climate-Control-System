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
#include "LoRa.h"
#include "stdio.h"
#include "string.h"
#include "stdint.h"
#include "DHT22.h"
#include "LCD_I2C.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NORMAL						0
#define TEMPERATURE_SETPOINT_LOW	1
#define TEMPERATURE_SETPOINT_HIGH	2
#define HUMIDITY_SETPOINT_LOW		3
#define HUMIDITY_SETPOINT_HIGH		4
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
LoRa myLoRa;								// LoRa handle
char lora_buffer[40];						// buffer containing the transmitted data to esp32
char received_data[20];						// buffer containing the received data from esp32
const char* srcId = "10";					// stm32 address
const char* desId = "20";					// esp32 address
char lcd_data[16];							// buffer containing LCD data
uint8_t mode = NORMAL;						// variable containing current mode of the program
DHT22_Data dht22Data = { 0 };				// dht22 received data
double temp_setpoint[2] = {22.0, 35.0};		// array containing lowest and highest temperature points
double humid_setpoint[2] = {60.0, 80.0};	// array containing lowest and highest humidity points
uint8_t dht22Status = 0;					// variable to check whether the dht22 data is received successfully
uint8_t loraStatus = 0;						// variable to check whether the LoRa data is transmitted successfully
int cmdStatus = 0;							// variable to check whether the command is processed successfully
int alarmStatus = 0;						// alarm mode status
int bulbStatus = 0;							// bulb status
int safety = 0;								// safety status

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
void Data_Packet_Encapsulation(char* buffer, int src_id, int des_id, int sendingType, double temp, double humid, int safety);
void Status_Packet_Encapsulation(char* buffer, int src_id, int des_id, int sendingType, int cmd_status, int alarm_status, int bulb_status);
void Lcd_Sytem_State_Print(uint8_t mode);
void Long_Pressed_Button(void);
int Alarm_Check(double temp, double humid);
void LoRa_Receive_Handle();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_SPI2_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
	// Initialize Timer 2 / Timer 3
	HAL_TIM_Base_Start(&htim1);
	HAL_TIM_Base_Start(&htim3);

	// Initialize DHT22
	while(!DHT22_Get_Data(&dht22Data))
	{
		// DHT22 doesn't response
		// Re-Initilize again
		Delay_Ms(2000);
	}

	// Initialize LCD
	lcd_init();

	// Initialize LoRa
	/* Consider three factors: transmission power, bandwidth and spreading factor.
	 * If you lower the transmission power, you will save the battery, but the range of the signal will obviously be shorter.
	 * If you increase the data rate (make the bandwidth wider or the spreading factor lower), you can transmit those bytes in a shorter time.
	 * The calculation is approximately as follows:
	 * Making the bandwidth 2x wider (from BW125 to BW250) allows you to send 2x more bytes in the same time.
	 * For a LoRa® receiver to detect (capture) a packet, the transmitter and receiver frequencies need to be within 25% of the bandwidth of each other, that’s according to the data sheet.
	 * Making the spreading factor 1 step lower (from SF10 to SF9) allows you to send 2x more bytes in the same time.
	 * Lowering the spreading factor makes it more difficult for the gateway to receive a transmission, as it will be more sensitive to noise.
	 * You could compare this to two people taking in a noisy place (a bar for example). If you’re far from each other, you have to talk slow (SF10), but if you’re close, you can talk faster (SF7) */
	myLoRa = newLoRa();
	myLoRa.CS_port = NSS_GPIO_Port;
	myLoRa.CS_pin = NSS_Pin;
	myLoRa.reset_port = RST_GPIO_Port;
	myLoRa.reset_pin = RST_Pin;
	myLoRa.DIO0_port = DIO0_GPIO_Port;
	myLoRa.DIO0_pin = DIO0_Pin;
	myLoRa.hSPIx = &hspi2;

	// Other parameters
//	myLoRa.frequency             = 433;             // default = 433 MHz
//	myLoRa.spredingFactor        = SF_7;           	// default = SF_7
	myLoRa.bandWidth             = BW_250KHz;       // default = BW_125KHz
//	myLoRa.power                 = POWER_17db;      // default = 20db

	if (LoRa_init(&myLoRa) == 200) {
		lcd_clear();
		lcd_put_cursor(0, 5);
		lcd_send_string("Hello");
		lcd_put_cursor(1, 2);
		lcd_send_string("LoRa started");
		HAL_Delay(2000);
		lcd_clear();
	} else {
		lcd_put_cursor(0, 0);
		lcd_send_string("Fail to initialize LoRa");
		HAL_Delay(3000);
		lcd_clear();
		lcd_put_cursor(0, 0);
		lcd_send_string("Please try again!");
	}
	LoRa_startReceiving(&myLoRa); // Start LoRa receive mode

	/* LoRa status encapsulation */
	Status_Packet_Encapsulation(lora_buffer, 10, 20, 1, 2, alarmStatus, bulbStatus);

	/* LoRa sending */
	LoRa_transmit(&myLoRa, (uint8_t*) lora_buffer, strlen(lora_buffer), 500);

	// Turn off the alarm mode
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

	//SCB->SCR |= ( 1 << 1);
		HAL_PWR_EnableSleepOnExit();

	/* lets start with fresh Status register of Timer to avoid any spurious interrupts */
	    TIM4->SR = 0;

	// Start Timer 4
	HAL_TIM_Base_Start_IT(&htim4);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 36000-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 36000-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 4000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DHT22_Pin|BUZZER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RST_Pin|NSS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BUZZER_BUTTON_Pin INC_BUTTON_Pin DES_BUTTON_Pin RELAY_BUTTON_Pin
                           MODE_BUTTON_Pin */
  GPIO_InitStruct.Pin = BUZZER_BUTTON_Pin|INC_BUTTON_Pin|DES_BUTTON_Pin|RELAY_BUTTON_Pin
                          |MODE_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DHT22_Pin BUZZER_Pin */
  GPIO_InitStruct.Pin = DHT22_Pin|BUZZER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : RELAY_Pin RST_Pin NSS_Pin */
  GPIO_InitStruct.Pin = RELAY_Pin|RST_Pin|NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : DIO0_Pin */
  GPIO_InitStruct.Pin = DIO0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(DIO0_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 7, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 7, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 7, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// Delay micro-second function
void Delay_Us(uint32_t us)
{
	__HAL_TIM_SET_COUNTER(&htim1, 0); // set counter value to 0
	while (__HAL_TIM_GET_COUNTER(&htim1) < us)
		; // wait for the counter to reach the us input in the parameter
}

// Delay mili-second function
void Delay_Ms(uint32_t ms)
{
	__HAL_TIM_SET_COUNTER(&htim3, 0); // set counter value to 0
	while (__HAL_TIM_GET_COUNTER(&htim3) < ms*2)
		; // wait for the counter to reach the us input in the parameter
}

// Trigger Buzzer function
void Buzzer_Trigger(void) {
	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
	Delay_Ms(130);
	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
}

// Buzzer button callback
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch (GPIO_Pin) {
			// MODE_BUTTON pressed
			case MODE_BUTTON_Pin:
				if(HAL_GPIO_ReadPin(MODE_BUTTON_GPIO_Port, MODE_BUTTON_Pin) == 0)
				{
					Buzzer_Trigger();
					if(mode == NORMAL)
					{
						lcd_clear();
						mode = TEMPERATURE_SETPOINT_LOW;
					}
					else if(mode == TEMPERATURE_SETPOINT_LOW)
					{
						lcd_clear();
						mode = TEMPERATURE_SETPOINT_HIGH;
					}
					else if(mode == TEMPERATURE_SETPOINT_HIGH)
					{
						lcd_clear();
						mode = HUMIDITY_SETPOINT_LOW;
					}
					else if(mode == HUMIDITY_SETPOINT_LOW)
					{
						lcd_clear();
						mode = HUMIDITY_SETPOINT_HIGH;
					}
					else
					{
						lcd_clear();
						mode = NORMAL;
					}
				}
				break;

			// BUZZER_BUTTON pressed
			case BUZZER_BUTTON_Pin:
				if(HAL_GPIO_ReadPin(BUZZER_BUTTON_GPIO_Port, BUZZER_BUTTON_Pin) == 0)
				{
					// Stop timer 4
					HAL_TIM_Base_Stop_IT(&htim4);
					Buzzer_Trigger();
					// Toggle the built-in LED, changing the alarm status
					HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
					// Check if the alarm is turned OFF
					if(HAL_GPIO_ReadPin(LED_GPIO_Port, LED_Pin))
					{
						alarmStatus = 0;
					}
					else
					{
						alarmStatus = 1;
					}

					/* LoRa status encapsulation */
					Status_Packet_Encapsulation(lora_buffer, 10, 20, 1, 2, alarmStatus, bulbStatus);
					Delay_Ms(100);
					/* LoRa sending data */
					while(!(LoRa_transmit(&myLoRa, (uint8_t*) lora_buffer, strlen(lora_buffer), 500)))
					{
						Delay_Ms(200);
					}
					// Start timer 4 again
					__HAL_TIM_SET_COUNTER(&htim4, 0);
					HAL_TIM_Base_Start_IT(&htim4);
				}
				break;

				// RELAY_BUTTON pressed
			case RELAY_BUTTON_Pin:
				// Toggle the bulb status
				if(HAL_GPIO_ReadPin(RELAY_BUTTON_GPIO_Port, RELAY_BUTTON_Pin) == 0)
				{
					// Stop timer 4
					HAL_TIM_Base_Stop_IT(&htim4);
					Buzzer_Trigger();
					HAL_GPIO_TogglePin(RELAY_GPIO_Port, RELAY_Pin);
					// Check if the bulb status
					if(HAL_GPIO_ReadPin(RELAY_GPIO_Port, RELAY_Pin))
					{
						bulbStatus = 1;
					}
					else
					{
						bulbStatus = 0;
					}

					/* LoRa status encapsulation */
					Status_Packet_Encapsulation(lora_buffer, 10, 20, 1, 2, alarmStatus, bulbStatus);
					Delay_Ms(100);
					/* LoRa sending data */
					while(!(LoRa_transmit(&myLoRa, (uint8_t*) lora_buffer, strlen(lora_buffer), 500)))
					{
						Delay_Ms(200);
					}
					// Start timer 4 again
					__HAL_TIM_SET_COUNTER(&htim4, 0);
					HAL_TIM_Base_Start_IT(&htim4);
				}
				break;

			// INC_BUTTON pressed
			case INC_BUTTON_Pin:
				if(HAL_GPIO_ReadPin(INC_BUTTON_GPIO_Port, INC_BUTTON_Pin) == 0)
				{
					Buzzer_Trigger();
					if(mode == TEMPERATURE_SETPOINT_LOW)
					{
						temp_setpoint[0] = temp_setpoint[0] + 0.1;
					}
					else if(mode == TEMPERATURE_SETPOINT_HIGH)
					{
						temp_setpoint[1] = temp_setpoint[1] + 0.1;
					}
					else if(mode == HUMIDITY_SETPOINT_LOW)
					{
						humid_setpoint[0] = humid_setpoint[0] + 0.1;
					}
					else if(mode == HUMIDITY_SETPOINT_HIGH)
					{
						humid_setpoint[1] = humid_setpoint[1] + 0.1;
					}
					else
					{
						mode = NORMAL;
					}
				}
				break;

			// DES_BUTTON pressed
			case DES_BUTTON_Pin:
				if(HAL_GPIO_ReadPin(DES_BUTTON_GPIO_Port, DES_BUTTON_Pin) == 0)
				{
					Buzzer_Trigger();
					if(mode == TEMPERATURE_SETPOINT_LOW)
					{
						temp_setpoint[0] = temp_setpoint[0] - 0.1;
					}
					else if(mode == TEMPERATURE_SETPOINT_HIGH)
					{
						temp_setpoint[1] = temp_setpoint[1] - 0.1;
					}
					else if(mode == HUMIDITY_SETPOINT_LOW)
					{
						humid_setpoint[0] = humid_setpoint[0] - 0.1;
					}
					else if(mode == HUMIDITY_SETPOINT_HIGH)
					{
						humid_setpoint[1] = humid_setpoint[1] - 0.1;
					}
					else
					{
						mode = NORMAL;
					}
				}
				break;

			case DIO0_Pin:
				/* LoRa receiving */
				LoRa_receive(&myLoRa, (uint8_t*) received_data, 20);
				LoRa_Receive_Handle();
				break;

			// Something went wrong, turn back to NORMAL MODE
			default:
				mode = NORMAL;
				break;
		}
}

// Encapsulate the data into a packet containing the string: "source_id, destination_id, sendingType, temperature, humidity, safety"
void Data_Packet_Encapsulation(char* buffer ,int src_id, int des_id, int sendingType, double temp, double humid, int safety)
{
	memset(buffer, 0, strlen(buffer));
	sprintf(buffer, "%d,%d,%d,%0.1lf,%0.1lf,%d", src_id, des_id, sendingType, temp, humid, safety);
}

// Encapsulate the status into a packet containing the string: "source_id,destination_id, sendingType, cmd_status, alarm_status"
void Status_Packet_Encapsulation(char* buffer ,int src_id, int des_id, int sendingType, int cmd_status, int alarm_status, int bulb_status)
{
	memset(buffer, 0, strlen(buffer));
	sprintf(buffer, "%d,%d,%d,%d,%d,%d", src_id, des_id, sendingType, cmd_status, alarm_status, bulb_status);
}

// System state LCD print
void Lcd_Sytem_State_Print(uint8_t mode) {
	switch (mode) {

	// NORMAL MODE LCD print
	case NORMAL:
		lcd_put_cursor(0, 0);
		if(dht22Data.temperature <= 9.9 && dht22Data.temperature >= 0.0)
		{
			sprintf(lcd_data, "TEMP:  %0.1lf C", dht22Data.temperature);
		}
		else
		{
			sprintf(lcd_data, "TEMP: %0.1lf C", dht22Data.temperature);
		}
		lcd_send_string(lcd_data);
		lcd_put_cursor(1, 0);
		if(dht22Data.humidity <= 9.9 && dht22Data.humidity >= 0)
		{
			sprintf(lcd_data, "HUMID:   %0.1lf %%", dht22Data.humidity);
		}
		else if(dht22Data.humidity == 100.0)
		{
			sprintf(lcd_data, "HUMID: %0.1lf %%", dht22Data.humidity);
		}
		else
		{
			sprintf(lcd_data, "HUMID:  %0.1lf %%", dht22Data.humidity);
		}
		lcd_send_string(lcd_data);
		break;

	//  TEMPERATURE_SETPOINT_LOW MODE LCD print
	case TEMPERATURE_SETPOINT_LOW:
		lcd_put_cursor(0, 0);
		lcd_send_string("TempSetpoint(L)");
		lcd_put_cursor(1, 0);
		if(temp_setpoint[0] <= 9.9 && temp_setpoint[0] >= 0.0)
		{
			sprintf(lcd_data, " %0.1lf C", temp_setpoint[0]);
		}
		else
		{
			sprintf(lcd_data, "%0.1lf C", temp_setpoint[0]);
		}
		lcd_send_string(lcd_data);
		lcd_put_cursor(1, 9);
		if(temp_setpoint[1] <= 9.9 && temp_setpoint[1] >= 0.0)
		{
			sprintf(lcd_data, " %0.1lf C", temp_setpoint[1]);
		}
		else
		{
			sprintf(lcd_data, "%0.1lf C", temp_setpoint[1]);
		}
		lcd_send_string(lcd_data);
		break;

	// TEMPERATURE_SETPOINT_HIGH MODE LCD print
	case TEMPERATURE_SETPOINT_HIGH:
		lcd_put_cursor(0, 0);
		lcd_send_string("TempSetpoint(H)");
		lcd_put_cursor(1, 0);
		if(temp_setpoint[0] <= 9.9 && temp_setpoint[0] >= 0.0)
		{
			sprintf(lcd_data, " %0.1lf C", temp_setpoint[0]);
		}
		else
		{
			sprintf(lcd_data, "%0.1lf C", temp_setpoint[0]);
		}
		lcd_send_string(lcd_data);
		lcd_put_cursor(1, 9);
		if(temp_setpoint[1] <= 9.9 && temp_setpoint[1] >= 0.0)
		{
			sprintf(lcd_data, " %0.1lf C", temp_setpoint[1]);
		}
		else
		{
			sprintf(lcd_data, "%0.1lf C", temp_setpoint[1]);
		}
		lcd_send_string(lcd_data);
		break;

	// HUMIDITY_SETPOINT_LOW MODE LCD print
	case HUMIDITY_SETPOINT_LOW:
		lcd_put_cursor(0, 0);
		lcd_send_string("HumidSetpoint(L)");
		lcd_put_cursor(1, 0);
		if(humid_setpoint[0] <= 9.9 && humid_setpoint[0] >= 0.0)
		{
			sprintf(lcd_data, "  %0.1lf %%", humid_setpoint[0]);
		}
		else if (humid_setpoint[0] == 100.0)
		{
			sprintf(lcd_data, "%0.1lf %%", humid_setpoint[0]);
		}
		else
		{
			sprintf(lcd_data, " %0.1lf %%", humid_setpoint[0]);
		}
		lcd_send_string(lcd_data);
		lcd_put_cursor(1, 8);
		if(humid_setpoint[1] <= 9.9 && humid_setpoint[1] >= 0.0)
		{
			sprintf(lcd_data, "  %0.1lf %%", humid_setpoint[1]);
		}
		else if (humid_setpoint[1] == 100.0)
		{
			sprintf(lcd_data, "%0.1lf %%", humid_setpoint[1]);
		}
		else
		{
			sprintf(lcd_data, " %0.1lf %%", humid_setpoint[1]);
		}
		lcd_send_string(lcd_data);
		break;

	// HUMIDITY_SETPOINT_HIGH MODE LCD print
	case HUMIDITY_SETPOINT_HIGH:
		lcd_put_cursor(0, 0);
		lcd_send_string("HumidSetpoint(H)");
		lcd_put_cursor(1, 0);
		if(humid_setpoint[0] < 10.0 && humid_setpoint[0] >= 0.0)
		{
			sprintf(lcd_data, "  %0.1lf %%", humid_setpoint[0]);
		}
		else if (humid_setpoint[0] == 100.0)
		{
			sprintf(lcd_data, "%0.1lf %%", humid_setpoint[0]);
		}
		else
		{
			sprintf(lcd_data, " %0.1lf %%", humid_setpoint[0]);
		}
		lcd_send_string(lcd_data);
		lcd_put_cursor(1, 8);
		if(humid_setpoint[1] < 10.0 && humid_setpoint[1] >= 0.0)
		{
			sprintf(lcd_data, "  %0.1lf %%", humid_setpoint[1]);
		}
		else if (humid_setpoint[1] == 100.0)
		{
			sprintf(lcd_data, "%0.1lf %%", humid_setpoint[1]);
		}
		else
		{
			sprintf(lcd_data, " %0.1lf %%", humid_setpoint[1]);
		}
		lcd_send_string(lcd_data);
		break;

	// Some issues occur, back to NORMAL MODE
	default:
		mode = NORMAL;
		break;
	}
}

// Long pressed button handle
void Long_Pressed_Button(void)
{
	HAL_TIM_Base_Stop_IT(&htim4);
	// Temperature set point increase/decrease when the button is pressed a long time
	while(!HAL_GPIO_ReadPin(DES_BUTTON_GPIO_Port, DES_BUTTON_Pin) && (mode == TEMPERATURE_SETPOINT_LOW))
	{
		Delay_Ms(150);
		if(!HAL_GPIO_ReadPin(DES_BUTTON_GPIO_Port, DES_BUTTON_Pin) && (mode == TEMPERATURE_SETPOINT_LOW))
		{
			temp_setpoint[0] = temp_setpoint[0] - 0.1;
			Lcd_Sytem_State_Print(mode);
		}
	}
	while(!HAL_GPIO_ReadPin(INC_BUTTON_GPIO_Port, INC_BUTTON_Pin) && (mode == TEMPERATURE_SETPOINT_LOW))
	{
		Delay_Ms(150);
		if(!HAL_GPIO_ReadPin(INC_BUTTON_GPIO_Port, INC_BUTTON_Pin) && (mode == TEMPERATURE_SETPOINT_LOW))
		{
			temp_setpoint[0] = temp_setpoint[0] + 0.1;
			Lcd_Sytem_State_Print(mode);
		}
	}
	while(!HAL_GPIO_ReadPin(DES_BUTTON_GPIO_Port, DES_BUTTON_Pin) && (mode == TEMPERATURE_SETPOINT_HIGH))
	{
		Delay_Ms(150);
		if(!HAL_GPIO_ReadPin(DES_BUTTON_GPIO_Port, DES_BUTTON_Pin) && (mode == TEMPERATURE_SETPOINT_HIGH))
		{
			temp_setpoint[1] = temp_setpoint[1] - 0.1;
			Lcd_Sytem_State_Print(mode);
		}
	}
	while(!HAL_GPIO_ReadPin(INC_BUTTON_GPIO_Port, INC_BUTTON_Pin) && (mode == TEMPERATURE_SETPOINT_HIGH))
	{
		Delay_Ms(150);
		if(!HAL_GPIO_ReadPin(INC_BUTTON_GPIO_Port, INC_BUTTON_Pin) && (mode == TEMPERATURE_SETPOINT_HIGH))
		{
			temp_setpoint[1] = temp_setpoint[1] + 0.1;
			Lcd_Sytem_State_Print(mode);
		}

	}

	// Humidity set point increase/decrease when the button is pressed a long time
	while(!HAL_GPIO_ReadPin(DES_BUTTON_GPIO_Port, DES_BUTTON_Pin) && (mode == HUMIDITY_SETPOINT_LOW))
	{
		Delay_Ms(150);
		if(!HAL_GPIO_ReadPin(DES_BUTTON_GPIO_Port, DES_BUTTON_Pin) && (mode == HUMIDITY_SETPOINT_LOW))
		{
			humid_setpoint[0] = humid_setpoint[0] - 0.1;
			Lcd_Sytem_State_Print(mode);
		}
	}
	while(!HAL_GPIO_ReadPin(INC_BUTTON_GPIO_Port, INC_BUTTON_Pin) && (mode == HUMIDITY_SETPOINT_LOW))
	{
		Delay_Ms(150);
		if(!HAL_GPIO_ReadPin(INC_BUTTON_GPIO_Port, INC_BUTTON_Pin) && (mode == HUMIDITY_SETPOINT_LOW))
		{
			humid_setpoint[0] = humid_setpoint[0] + 0.1;
			Lcd_Sytem_State_Print(mode);
		}
	}
	while(!HAL_GPIO_ReadPin(DES_BUTTON_GPIO_Port, DES_BUTTON_Pin) && (mode == HUMIDITY_SETPOINT_HIGH))
	{
		Delay_Ms(150);
		if(!HAL_GPIO_ReadPin(DES_BUTTON_GPIO_Port, DES_BUTTON_Pin) && (mode == HUMIDITY_SETPOINT_HIGH))
		{
			humid_setpoint[1] = humid_setpoint[1] - 0.1;
			Lcd_Sytem_State_Print(mode);
		}
	}
	while(!HAL_GPIO_ReadPin(INC_BUTTON_GPIO_Port, INC_BUTTON_Pin) && (mode == HUMIDITY_SETPOINT_HIGH))
	{
		Delay_Ms(150);
		if(!HAL_GPIO_ReadPin(INC_BUTTON_GPIO_Port, INC_BUTTON_Pin) && (mode == HUMIDITY_SETPOINT_HIGH))
		{
			humid_setpoint[1] = humid_setpoint[1] + 0.1;
			Lcd_Sytem_State_Print(mode);
		}
	}
	HAL_TIM_Base_Start_IT(&htim4);
}

// Alarm checking
int Alarm_Check(double temp, double humid)
{
	// Check if the alarm is turned ON
	if(!HAL_GPIO_ReadPin(LED_GPIO_Port, LED_Pin))
	{
		// Check the temperature value
		if(temp > temp_setpoint[1])
		{
			// The temperature is too HIGH
			Buzzer_Trigger();
			lcd_clear();
			lcd_put_cursor(0, 4);
			lcd_send_string("Too high");
			lcd_put_cursor(1, 2);
			lcd_send_string("Temperature");
			return 1;
		}
		if(temp < temp_setpoint[0])
		{
			// The temperature is too LOW
			Buzzer_Trigger();
			lcd_clear();
			lcd_put_cursor(0, 4);
			lcd_send_string("Too low");
			lcd_put_cursor(1, 2);
			lcd_send_string("Temperature");
			return 1;
		}

		// Check the humidity value
		if(humid > humid_setpoint[1])
		{
			// The humidity is too HIGH
			Buzzer_Trigger();
			lcd_clear();
			lcd_put_cursor(0, 4);
			lcd_send_string("Too high");
			lcd_put_cursor(1, 4);
			lcd_send_string("Humidity");
			return 1;
		}
		if(humid < humid_setpoint[0])
		{
			// The humidity is too LOW
			Buzzer_Trigger();
			lcd_clear();
			lcd_put_cursor(0, 4);
			lcd_send_string("Too low");
			lcd_put_cursor(1, 4);
			lcd_send_string("Humidity");
			return 1;
		}
	}
	return 0;
}

// LoRa receive data handle
void LoRa_Receive_Handle()
{
	// Stop timer 4
	HAL_TIM_Base_Stop_IT(&htim4);
	char *rcvSrcId;		// pointer to source ID
	char *rcvDesId;		// pointer to destination ID
	char *cmdType;			// pointer to received message

	rcvSrcId = strtok(received_data, ",");            	// return the pointer to the source_id
	rcvDesId = strtok(NULL, ",");                     	// continue to return the pointer to the destination_id
	cmdType = strtok(NULL, ","); 						// continue to return the pointer to the received command type

	// Check if the destination ID is correct
	if(!strcmp(rcvDesId, srcId))
	{
		if (!strcmp(cmdType,"1"))
		{
			char *status; // pointer to the received buzzer status
			status = strtok(NULL, ","); // continue to return the pointer to the received buzzer status
			if (!strcmp(status, "ON"))
			{
				HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
				cmdStatus = 1;
				alarmStatus = 1;
			}
			else if (!strcmp(status, "OFF"))
			{
				HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
				cmdStatus = 1;
				alarmStatus = 0;
			}
		}
		else if (!strcmp(cmdType, "2"))
		{
			char *tempLowest; 	// pointer to the received lowest temperature level
			char *tempHighest;	// pointer to the received highest temperature level
			tempLowest = strtok(NULL, ","); // continue to return the pointer to the received lowest temperature level
			tempHighest = strtok(NULL, ","); // continue to return the pointer to the received highest temperature level
			double d;
			sscanf(tempLowest, "%lf", &d);
			temp_setpoint[0] = d;
			sscanf(tempHighest, "%lf", &d);
			temp_setpoint[1] = d;
			cmdStatus = 1;
		}
		else if (!strcmp(cmdType, "3"))
		{
			char *humidLowest; 	// pointer to the received lowest humidity level
			char *humidHighest;	// pointer to the received highest humidity level
			humidLowest = strtok(NULL, ","); // continue to return the pointer to the received lowest humidity level
			humidHighest = strtok(NULL, ","); // continue to return the pointer to the received highest humdity level
			double d;
			sscanf(humidLowest, "%lf", &d);
			humid_setpoint[0] = d;
			sscanf(humidHighest, "%lf", &d);
			humid_setpoint[1] = d;
			cmdStatus = 1;
		}
		else if (!strcmp(cmdType,"4"))
		{
			char *status; // pointer to the received motor status
			status = strtok(NULL, ","); // continue to return the pointer to the received buzzer status
			if (!strcmp(status, "ON"))
			{
				HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, GPIO_PIN_SET);
				cmdStatus = 1;
				bulbStatus = 1;
			}
			else if (!strcmp(status, "OFF"))
			{
				HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, GPIO_PIN_RESET);
				cmdStatus = 1;
				bulbStatus = 0;
			}
		}
		else if (!strcmp(cmdType,"5"))
		{
			cmdStatus = 1;
		}

		/* LoRa status encapsulation */
		Status_Packet_Encapsulation(lora_buffer, 10, 20, 1, cmdStatus, alarmStatus, bulbStatus);
		Delay_Ms(100);

		/* LoRa sending data */
		while(!(LoRa_transmit(&myLoRa, (uint8_t*) lora_buffer, strlen(lora_buffer), 500)))
		{
			Delay_Ms(200);
		}

		cmdStatus = 0;
	}
	// Start timer 4 again
	__HAL_TIM_SET_COUNTER(&htim4, 0);
	HAL_TIM_Base_Start_IT(&htim4);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim4)
	{
		/* DHT22 get data */
		dht22Status = DHT22_Get_Data(&dht22Data);

		if(dht22Status == 1)
		{
			/* Check the temperature and humidity level */
			safety = Alarm_Check(dht22Data.temperature, dht22Data.humidity); // 0: safe, 1: unsafe

			/* LoRa data encapsulation */
			Data_Packet_Encapsulation(lora_buffer, 10, 20, 0, dht22Data.temperature, dht22Data.humidity, safety);

			/* LoRa sending data */
			LoRa_transmit(&myLoRa, (uint8_t*) lora_buffer, strlen(lora_buffer), 100);
		}

		if (safety == 0)
		{
			/* Print DHT22 data onto LCD */
			Lcd_Sytem_State_Print(mode);

			/* Handle when the increase/decrease button is pressed a long time */
			Long_Pressed_Button();
		}
	}
}

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
