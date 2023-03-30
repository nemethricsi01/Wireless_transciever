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
#include "si4463.h"
#include <stdlib.h>
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
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */
#define APP_PACKET_LEN		((uint16_t) 10)

si4463_t si4463;
uint8_t incomingBuffer[APP_PACKET_LEN];
uint8_t outgoingBuffer[APP_PACKET_LEN];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void refresh_watchdog(void);
uint8_t SI4463_IsCTS(void);
void SI4463_WriteRead(const uint8_t * pTxData, uint8_t * pRxData, const uint16_t sizeTxData);
void SI4463_SetShutdown(void);
void SI4463_ClearShutdown(void);
void SI4463_Select(void);
void SI4463_Deselect(void);
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
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_USB_PCD_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
#ifdef DEMOFEST
  /* Debug message on uart */
  HAL_UART_Transmit(&huart1, "START\n", 6, 10);
#endif

  /* Assign functions */
  si4463.IsClearToSend = SI4463_IsCTS;
  si4463.WriteRead = SI4463_WriteRead;
  si4463.Select = SI4463_Select;
  si4463.Deselect = SI4463_Deselect;
  si4463.SetShutdown = SI4463_SetShutdown;
  si4463.ClearShutdown = SI4463_ClearShutdown;
  si4463.DelayMs = HAL_Delay;

  /* Disable interrupt pin for init Si4463 */
  HAL_NVIC_DisableIRQ(EXTI0_IRQn);

  /* Init Si4463 with structure */
  SI4463_Init(&si4463);

  /* Clear RX FIFO before starting RX packets */
  SI4463_ClearRxFifo(&si4463);
  /* Start RX mode.
   * SI4463_StartRx() put a chip in non-armed mode in cases:
   * - successfully receive a packet;
   * - invoked RX_TIMEOUT;
   * - invalid receive.
   * For receiveing next packet you have to invoke SI4463_StartRx() again!*/
  SI4463_StartRx(&si4463, APP_PACKET_LEN, false, false, false);

#ifdef DEMOFEST
  /* Debug message on UART */
  HAL_UART_Transmit(&huart1, "INIT\n", 5, 10);
#endif

  /* Enable interrupt pin and */
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
  /* Clear interrupts after enabling interrupt pin.
   * Without it may be situation when interrupt is asserted but pin not cleared.*/
  SI4463_ClearInterrupts(&si4463);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		/* Send test packet */
	  outgoingBuffer[0] = rand() & 0xFF;
	  outgoingBuffer[1] = rand() & 0xFF;
	  outgoingBuffer[2] = rand() & 0xFF;
	  outgoingBuffer[3] = rand() & 0xFF;
	  outgoingBuffer[4] = rand() & 0xFF;
	  outgoingBuffer[5] = rand() & 0xFF;
	  outgoingBuffer[6] = rand() & 0xFF;
	  SI4463_Transmit(&si4463, outgoingBuffer, APP_PACKET_LEN);

	  uint32_t newDelay = 500 + ((rand() & 0xF) * 100);
	  HAL_Delay(newDelay);
	  /* End of send of test packet */
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
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
  hi2c1.Init.Timing = 0x10909CEC;
  hi2c1.Init.OwnAddress1 = 0;
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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x10909CEC;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, TX_SDN_Pin|WDO_Pin|WDEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, TX_nSEL_Pin|SMPS_EN_Pin|SMPS_V1_Pin|SMPS_SW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : B1_Pin PC1 */
  GPIO_InitStruct.Pin = B1_Pin|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : TX_SDN_Pin WDO_Pin WDEN_Pin */
  GPIO_InitStruct.Pin = TX_SDN_Pin|WDO_Pin|WDEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : TX_nSEL_Pin SMPS_EN_Pin SMPS_V1_Pin SMPS_SW_Pin */
  GPIO_InitStruct.Pin = TX_nSEL_Pin|SMPS_EN_Pin|SMPS_V1_Pin|SMPS_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SMPS_PG_Pin */
  GPIO_InitStruct.Pin = SMPS_PG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SMPS_PG_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD4_Pin */
  GPIO_InitStruct.Pin = LD4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CTS_Pin */
  GPIO_InitStruct.Pin = CTS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CTS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(GPIO_Pin);

  /* Clear incoming buffer */
  memset(incomingBuffer, 0x00, APP_PACKET_LEN);

  /* Get interrupts and work with it */
  SI4463_GetInterrupts(&si4463);

  /* Handling PH interrupts */
  if (si4463.interrupts.filterMatch)
  {
	  /* Handling this interrupt here */
	  /* Following instruction only for add breakpoints. May be deleted */
	  si4463.interrupts.filterMatch = false;
  }
  if (si4463.interrupts.filterMiss)
  {
	  /* Handling this interrupt here */
	  /* Following instruction only for add breakpoints. May be deleted */
	  si4463.interrupts.filterMiss = false;
  }
  if (si4463.interrupts.packetSent)
  {
	  /* Handling this interrupt here */
	  /* Clear TX FIFO */
	  SI4463_ClearTxFifo(&si4463);
#ifdef DEMOFEST
	  HAL_UART_Transmit(&huart1, "OUT >", 5, 10);
	  HAL_UART_Transmit(&huart1, outgoingBuffer, APP_PACKET_LEN, 10);
	  HAL_UART_Transmit(&huart1, "\n", 1, 10);
#endif /* DEMOFEST */
	  /* Re-arm StartRX */
	  SI4463_StartRx(&si4463, APP_PACKET_LEN, false, false, false);

	  /*Toggle led for indication*/
	  HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
	  /* Following instruction only for add breakpoints. May be deleted */
	  si4463.interrupts.packetSent = false;
  }
  if (si4463.interrupts.packetRx)
  {
	  /* Handling this interrupt here */
	  /* Get FIFO data */
	  SI4463_ReadRxFifo(&si4463, incomingBuffer, APP_PACKET_LEN);
	  /* Clear RX FIFO */
	  SI4463_ClearRxFifo(&si4463);
#ifdef DEMOFEST
	  HAL_UART_Transmit(&huart1, "IN >", 4, 10);
	  HAL_UART_Transmit(&huart1, incomingBuffer, APP_PACKET_LEN, 10);
	  HAL_UART_Transmit(&huart1, "\n", 1, 10);
#endif /* DEMOFEST */

	  /* Start RX again.
	   * It need because after successful receive a packet the chip change
	   * state to READY.
	   * There is re-armed mode for StartRx but it not correctly working */
	  SI4463_StartRx(&si4463, APP_PACKET_LEN, false, false, false);

	  /*Toggle led for indication*/
	  HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);

	  /* Following instruction only for add breakpoints. May be deleted */
	  si4463.interrupts.packetRx = false;
  }
  if (si4463.interrupts.crcError)
  {
	  /* Handling this interrupt here */

	  /* Following instruction only for add breakpoints. May be deleted */
	  si4463.interrupts.crcError = false;
  }
  if (si4463.interrupts.txFifoAlmostEmpty)
  {
	  /* Handling this interrupt here */

	  /* Following instruction only for add breakpoints. May be deleted */
	  si4463.interrupts.txFifoAlmostEmpty = false;
  }
  if (si4463.interrupts.rxFifoAlmostFull)
  {
	  /* Handling this interrupt here */

	  /* Following instruction only for add breakpoints. May be deleted */
	  si4463.interrupts.rxFifoAlmostFull = false;
  }

  /* Handling Modem interrupts */
  if (si4463.interrupts.postambleDetect)
  {
	  /* Handling this interrupt here */
	  /* Following instruction only for add breakpoints. May be deleted */
	  si4463.interrupts.postambleDetect = false;
  }
  if (si4463.interrupts.invalidSync)
  {
	  /* Handling this interrupt here */
	  /* Following instruction only for add breakpoints. May be deleted */
	  si4463.interrupts.invalidSync = false;
  }
  if (si4463.interrupts.rssiJump)
  {
	  /* Handling this interrupt here */
	  /* Following instruction only for add breakpoints. May be deleted */
	  si4463.interrupts.rssiJump = false;
  }
  if (si4463.interrupts.rssi)
  {
	  /* Handling this interrupt here */
	  /* Following instruction only for add breakpoints. May be deleted */
	  si4463.interrupts.rssi = false;
  }
  if (si4463.interrupts.invalidPreamble)
  {
	  /* Handling this interrupt here */
	  /* Following instruction only for add breakpoints. May be deleted */
	  si4463.interrupts.invalidPreamble = false;
  }
  if (si4463.interrupts.preambleDetect)
  {
	  /* Handling this interrupt here */
	  /* Following instruction only for add breakpoints. May be deleted */
	  si4463.interrupts.preambleDetect = false;
  }
  if (si4463.interrupts.syncDetect)
  {
	  /* Handling this interrupt here */
	  /* Following instruction only for add breakpoints. May be deleted */
	  si4463.interrupts.syncDetect = false;
  }

  /* Handling Chip interrupts */
  if (si4463.interrupts.cal)
  {
	  /* Handling this interrupt here */
	  SI4463_GetChipStatus(&si4463);
	  /* Following instruction only for add breakpoints. May be deleted */
	  si4463.interrupts.cal = false;
  }
  if (si4463.interrupts.fifoUnderflowOverflowError)
  {
	  /* Handling this interrupt here */
	  /* Clear RX FIFO */
	  SI4463_ClearRxFifo(&si4463);
	  /* Claer Chip Status errors if exists */
	  SI4463_GetChipStatus(&si4463);
	  /* Following instruction only for add breakpoints. May be deleted */
	  si4463.interrupts.fifoUnderflowOverflowError = false;
  }
  if (si4463.interrupts.stateChange)
  {
	  /* Handling this interrupt here */
	  SI4463_GetChipStatus(&si4463);
	  /* Following instruction only for add breakpoints. May be deleted */
	  si4463.interrupts.stateChange = false;
  }
  if (si4463.interrupts.cmdError)
  {
	  /* Handling this interrupt here */
	  SI4463_GetChipStatus(&si4463);
	  /* Following instruction only for add breakpoints. May be deleted */
	  si4463.interrupts.stateChange = false;
  }
  if (si4463.interrupts.chipReady)
  {
	  /* Handling this interrupt here */
	  SI4463_GetChipStatus(&si4463);
	  /* Following instruction only for add breakpoints. May be deleted */
	  si4463.interrupts.chipReady = false;
  }
  if (si4463.interrupts.lowBatt)
  {
	  /* Handling this interrupt here */
	  SI4463_GetChipStatus(&si4463);
	  /* Following instruction only for add breakpoints. May be deleted */
	  si4463.interrupts.lowBatt = false;
  }
  if (si4463.interrupts.wut)
  {
	  /* Handling this interrupt here */
	  SI4463_GetChipStatus(&si4463);
	  /* Following instruction only for add breakpoints. May be deleted */
	  si4463.interrupts.wut = false;
  }

  /* Clear All interrupts before exit */
  SI4463_ClearAllInterrupts(&si4463);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
}

uint8_t SI4463_IsCTS(void)
{
	if(HAL_GPIO_ReadPin(CTS_GPIO_Port, CTS_Pin) == GPIO_PIN_SET)
	{
		return true;
	}
	else
	{
		return false;
	}
}

void SI4463_WriteRead(const uint8_t * pTxData, uint8_t * pRxData, const uint16_t sizeTxData)
{
	HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)pTxData, pRxData, sizeTxData, 100);
}

void SI4463_SetShutdown(void)
{
	HAL_GPIO_WritePin(TX_SDN_GPIO_Port, TX_SDN_Pin, GPIO_PIN_SET);
}

void SI4463_ClearShutdown(void)
{
	HAL_GPIO_WritePin(TX_SDN_GPIO_Port, TX_SDN_Pin, GPIO_PIN_RESET);
}

void SI4463_Select(void)
{
	HAL_GPIO_WritePin(TX_nSEL_GPIO_Port, TX_nSEL_Pin, GPIO_PIN_RESET);
}

void SI4463_Deselect(void)
{
	HAL_GPIO_WritePin(TX_nSEL_GPIO_Port, TX_nSEL_Pin, GPIO_PIN_SET);
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

