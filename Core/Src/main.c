/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include <string.h>
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
void speelToon(double toon, int tijd);
void speelBrandAlarm();
void leesTemp(float* temp, float* humid);
void sendTemp(float temp);
void sendHumid(float humid);
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define KLOK_FREQ 32000000
#define PRESCALER 2
#define toon1 1000
#define toon2 600
#define TijdsDuur100 100

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//canbus
uint32_t mb;
CAN_TxHeaderTypeDef msg;
uint8_t data[] = {1};

CAN_RxHeaderTypeDef msg2;
uint8_t data2[8];

int brandAlarm = 0;

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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_CAN1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */






  /* Config:
   * I2C1_SDA = PB7 (D4) PULL-UP, very high
   * I2C1_SCL = PB6 (D5) PULL-UP, very high
   * GND & V5
   *
   * standard mode 100 kpbs
   * primary slave address = 1
   */

  float temperature, humidity;
  char temperatuurTekst[50];
  char luchtvochtigheidTekst[50];
  char newLine[6] = "\n\r";

  uint32_t lastTempTime = 0;
  uint32_t lastHumidTime = 0;


  uint8_t welkom[50] = "Temperatuur & luchtvochtigheid sensor:\n\r\n\r";
  HAL_UART_Transmit(&huart2, welkom, sizeof(welkom), HAL_MAX_DELAY);

 // HAL_CAN_Start(&hcan1);

  HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_1);
  __HAL_TIM_SET_AUTORELOAD(&htim1, 0);

  HAL_TIM_Base_Start(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	    if(brandAlarm == 1){
	    	speelBrandAlarm();
	    }

	    leesTemp(&temperature, &humidity);

	    //zet alles tekst en data in één variabele
	    sprintf(temperatuurTekst, "Temperatuur: %.2f°C\n\r", temperature);
	    sprintf(luchtvochtigheidTekst, "Luchtvochtigheid: %.2f%%\n\r", humidity);

	    // print de temperatuur en luchtvochtigheid
	    HAL_UART_Transmit(&huart2, (uint8_t*)temperatuurTekst, strlen(temperatuurTekst), 100);
	    HAL_UART_Transmit(&huart2, (uint8_t*)luchtvochtigheidTekst, strlen(luchtvochtigheidTekst), 100);
	    HAL_UART_Transmit(&huart2, (uint8_t*)newLine, strlen(newLine), HAL_MAX_DELAY);

	    /* 1 keer per seconde
	     * feq = klokf / (pre * arr)
	     * 1 = 80.000.000 / (8000 * 10.000)
	     */
	    uint32_t currentTime = __HAL_TIM_GET_COUNTER(&htim2);

        if ((currentTime - lastTempTime) >= 10000) { // elke seconde
            sendTemp(temperature);
            lastTempTime = currentTime;
        }

        if ((currentTime - lastHumidTime) >= 20000) { // elke twee seconden
            sendHumid(humidity);
            lastHumidTime = currentTime;
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */
	  hcan1.Instance = CAN1;
	  hcan1.Init.Prescaler = 4;
	  hcan1.Init.Mode = CAN_MODE_NORMAL;
	  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
	  hcan1.Init.TimeSeg1 = CAN_BS1_15TQ;
	  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
	  hcan1.Init.TimeTriggeredMode = DISABLE;
	  hcan1.Init.AutoBusOff = DISABLE;
	  hcan1.Init.AutoWakeUp = ENABLE;
	  hcan1.Init.AutoRetransmission = DISABLE;
	  hcan1.Init.ReceiveFifoLocked = DISABLE;
	  hcan1.Init.TransmitFifoPriority = DISABLE;

	  if (HAL_CAN_Init(&hcan1) != HAL_OK)
	  {
	    Error_Handler();
	  }

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 4;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_15TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = ENABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
	  CAN_FilterTypeDef sFilterConfig;

	  sFilterConfig.FilterBank = 0;
	  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	  sFilterConfig.FilterIdHigh = 0x0000;
	  sFilterConfig.FilterIdLow = 0x0000;
	  sFilterConfig.FilterMaskIdHigh = 0x0000;
	  sFilterConfig.FilterMaskIdLow = 0x0000;
	  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	  sFilterConfig.FilterActivation = ENABLE;
	  sFilterConfig.SlaveStartFilterBank = 12;


	     if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) {
	       Error_Handler();
	     }


	     if (HAL_CAN_Start(&hcan1) != HAL_OK) {
	       Error_Handler();
	     }

	     if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK)
	     {
	     /* Notification Error */
	     Error_Handler();
	     }
  /* USER CODE END CAN1_Init 2 */

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
  hi2c1.Init.Timing = 0x00707CBB;
  hi2c1.Init.OwnAddress1 = 2;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 3 - 1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
 * Functie voor het zenden van de temperatuur via CAN.
 */
void sendTemp(float temp){
	uint32_t mb;
    CAN_TxHeaderTypeDef msg;
    uint8_t data[4] = {0};

    memcpy(data, &temp, sizeof(float)); //4 bytes

    // Configureer het CAN bericht
    msg.StdId = 5;
    msg.IDE = CAN_ID_STD;
    msg.RTR = CAN_RTR_DATA;
    msg.DLC = 4;
    msg.TransmitGlobalTime = DISABLE;
	if (HAL_CAN_AddTxMessage(&hcan1, &msg, data, &mb) != HAL_OK) {
		Error_Handler();
	}
}
/**
 * Functie voor het zenden van de luchtvochtigheid via CAN.
 */
void sendHumid(float humid){
	uint32_t mb;
    CAN_TxHeaderTypeDef msg;
    uint8_t data[4] = {0};

    memcpy(data, &humid, sizeof(float)); //4 bytes

    // Configureer het CAN bericht
    msg.StdId = 6;
    msg.IDE = CAN_ID_STD;
    msg.RTR = CAN_RTR_DATA;
    msg.DLC = 4;
    msg.TransmitGlobalTime = DISABLE;
	if (HAL_CAN_AddTxMessage(&hcan1, &msg, data, &mb) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * Interrupt functie wanneer er een bericht via CAN binnenkomt
 * Een bericht met ID 2 is voor het aansturen van de buzzer.
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
	if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &msg2, data2) == HAL_OK){
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);	//test led
		if(msg2.StdId == 2){
			if(data2[0] == 1){	//bij data '1' ontvangen moet het brandalarm aan
				brandAlarm = 1;
			}
			else if(data2[0] == 0){ //bij data '0' ontvangen moet het brandalarm uit
				brandAlarm = 0;
			}
		}
	}
}

/**
 * Interrupt functie om te testen dat er iets verzonden is met CANbus
 */
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
}

/**
 * Functie om temperatuur en luchtvochtigheid uit te lezen en om te zetten naar Celsius
 */
void leesTemp(float* temp, float* humid){
    uint8_t data[6] = {};
    int16_t tempRaw = 0;
    uint16_t humidityRaw = 0;
    uint8_t SHT3X_ADDR = 0x44 << 1; // SHT3x I2C address shifted left by 1 bit, omdat het I2C 7 bits is.

    //clock stetching enabled:
    uint8_t cmdCS[] = {0x2C, 0x06};

    // stuur commando om temperatuur uit te lezen
    HAL_I2C_Master_Transmit(&hi2c1, SHT3X_ADDR, cmdCS, sizeof(cmdCS), HAL_MAX_DELAY);
    HAL_Delay(100);


    // lees de temperatuur
    HAL_I2C_Master_Receive(&hi2c1, SHT3X_ADDR  | 0x01, data, 6, HAL_MAX_DELAY);

    // zet de ruwe data om naar reeële waardes
    //[0] & [1] zijn temperatuur data
    //[2] is CRC
    //[3] & [4] zijn luchtvochtigheids data
    //[5] is CRC

    tempRaw = (data[0] << 8) | data[1];
    *temp = -45 + 175 * ((float)tempRaw/65535);

    humidityRaw = (data[3] << 8) | data[4];
    *humid = 100 * ((float)humidityRaw / 65535);
}
/**
 * Functie om een toon af te laten spelen op de buzzer
 */
void speelToon(double toon, int tijd) {

	int ARR = KLOK_FREQ / (((2 * (PRESCALER + 1) * toon)) - 1);
	__HAL_TIM_SET_AUTORELOAD(&htim1, ARR);
	HAL_Delay(tijd);
	__HAL_TIM_SET_AUTORELOAD(&htim1, 0);			//naar 0 zetten voor rust tussen de noten in
}

/*
ARR = klokfreq / (((2 * (prescaler + 1) * gewenstefreq)) - 1)

Voor een brandalarm wordt vaak een pulserend geluid gebruikt met een frequentie tussen 800 Hz en 1200 Hz.
6.668 = 32.000.000 / ((2 * (2 + 1) * 800) - 1) //800hz
4.445 =	32.000.000 / ((2 * (2 + 1) * 1200 - 1) //1200hz
*/

/**
 * Functie om het geluid van een brandalarm te laten spelen
 */
void speelBrandAlarm(){
	speelToon(toon1, TijdsDuur100);
	HAL_Delay(TijdsDuur100);
	speelToon(toon2, TijdsDuur100);
	HAL_Delay(TijdsDuur100);
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
