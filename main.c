/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
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
CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* CAN  - Private variables ---------------------------------------------------------*/
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint8_t				TxData[8];
uint8_t				RxData[8];
uint32_t			TxMailbox;
CAN_FilterTypeDef 	sFilterConfig;

/* I2C  - Private variables ---------------------------------------------------------*/
/*
 * Master Variable
 */
uint8_t master_rx[10];
uint8_t master_tx[10] = "12345678";
uint8_t master_NumOfByte = 5; //default
/*
 * Slave Variable
 */
uint8_t slaveAddr = 0x08; //default
uint8_t slave_rx[10];
uint8_t slave_tx[10] = "abcdefg";
uint8_t slave_size;
uint8_t slave_NumOfByte = 6;
/*
 * FSM - I2C Control Definition
 */
#define FSM_I2C_MasterSetWrite		 		0x01
#define FSM_I2C_MasterWriteData				0x02
#define FSM_I2C_MasterRead					0x03
#define FSM_I2C_MasterSel					0x04
#define FSM_I2C_SlaveSel					0x05
#define FSM_I2C_SlaveRespSet				0x06
#define FSM_I2C_SlaveResp					0x07
#define FSM_I2C_SetSlaveAddr				0x08

/* UART - Private variables ---------------------------------------------------------*/
uint8_t serial_rxChar[32];
uint8_t rxSize = 5;	//default
uint16_t delay_period = 1000; //default = 1000
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN1_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_USART2_UART_Init();
  MX_CAN1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  printf("STM32 - Nucleo L476RG - Program start... \n\r");
  while (1)
  {
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
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

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 10;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  uint32_t filter_id = 0x00000000;
  uint32_t filter_mask = 0x1FFFFFFF8;

  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = ((filter_id << 5)  | (filter_id >> (32 - 5))) & 0xFFFF; // STID[10:0] & EXTID[17:13]
  sFilterConfig.FilterIdLow = (filter_id >> (11 - 3)) & 0xFFF8; // EXID[12:5] & 3 Reserved bits
  sFilterConfig.FilterMaskIdHigh = ((filter_mask << 5)  | (filter_mask >> (32 - 5))) & 0xFFFF;
  sFilterConfig.FilterMaskIdLow = (filter_mask >> (11 - 3)) & 0xFFF8;

  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;

  if(HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)	Error_Handler();

  // Step 2: Start CAN
  if(HAL_CAN_Start(&hcan1) != HAL_OK)
	  Error_Handler();
  // Step 3: Enable Notification
  if(HAL_CAN_ActivateNotification(&hcan1, CAN_IT_TX_MAILBOX_EMPTY | CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
	  Error_Handler();
  // Step 4: Initialize/Fill the TX message
  TxHeader.StdId = 0x323;	//CAN Standard ID
  TxHeader.ExtId = 0x00;	//CAN Extend ID
  TxHeader.RTR = CAN_RTR_DATA; //
  TxHeader.IDE = CAN_ID_STD;	//
  TxHeader.DLC = 8;
  TxHeader.TransmitGlobalTime = DISABLE;


  TxData[0] = 1;
  TxData[1] = 2;
  TxData[2] = 3;
  TxData[3] = 4;
  TxData[4] = 5;
  TxData[5] = 6;
  TxData[6] = 7;
  TxData[7] = 8;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* UART - Private user code ---------------------------------------------------------*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	// HAL_UART_Transmit(huart, &serial_rxChar, 5, 100); //echo receive data back to serial port.

	uint8_t index = 0;

	uint8_t sizeNext = 5; //default

	switch (serial_rxChar[0])
	{
		case FSM_I2C_MasterSetWrite:
			slaveAddr = serial_rxChar[1] + 0x30 ;

			master_NumOfByte =  serial_rxChar[2] - 1;

			rxSize = serial_rxChar[2] + 0x30;

			sizeNext =  serial_rxChar[2];

			delay_period = 100;

			break;

		case FSM_I2C_MasterRead:
			slaveAddr = serial_rxChar[1] + 0x30 ;

			master_NumOfByte =  serial_rxChar[2] - 1;

			rxSize = serial_rxChar[2] + 0x30;

			HAL_I2C_Master_Receive_IT(&hi2c1, (slaveAddr - 0x30) << 1, master_rx, master_NumOfByte);

			break;

		case FSM_I2C_MasterSel:

			HAL_I2C_DisableListen_IT(&hi2c1);

			delay_period = 5000;
			break;
		case FSM_I2C_SlaveSel:
			HAL_I2C_EnableListen_IT(&hi2c1);

			delay_period = 5000;

			break;

		case FSM_I2C_MasterWriteData:

			for(index = 0; index < master_NumOfByte;index++)
			{
				master_tx[index] = serial_rxChar[index + 1];
			}

			HAL_I2C_Master_Transmit_IT(&hi2c1, (slaveAddr - 0x30) << 1, master_tx, master_NumOfByte);
			break;

		case FSM_I2C_SlaveRespSet:

			slaveAddr = serial_rxChar[1] + 0x30 ;

			slave_NumOfByte =  serial_rxChar[2] - 1;

			sizeNext =  serial_rxChar[2];

			break;

		case FSM_I2C_SlaveResp:

			memset(slave_tx,0,sizeof slave_tx); // flush slave tx buffer before set a new data
			for(index = 0 ; index < slave_NumOfByte; index++)
			{
				slave_tx[index] = serial_rxChar[index + 1];
			}

			delay_period = 1000;

			break;

		case FSM_I2C_SetSlaveAddr:
			slaveAddr = serial_rxChar[1] + 0x30 ;

			HAL_I2C_DisableListen_IT(&hi2c1);

			HAL_I2C_DeInit(&hi2c1);

			hi2c1.Init.OwnAddress1 = (slaveAddr - 0x30) << 1;	//re-initial device own slave address

			if (HAL_I2C_Init(&hi2c1) != HAL_OK)	Error_Handler();

			HAL_I2C_EnableListen_IT(&hi2c1);
		default:
			break;
	}

	printComMsg(serial_rxChar[0], slaveAddr, sizeNext);
	/* Re-initialize uart receive depend on the command */
	memset(serial_rxChar,0,sizeof serial_rxChar);
	HAL_UART_Receive_IT(huart, (uint8_t *) &serial_rxChar, sizeNext);
}

/* I2C	- Private user code ---------------------------------------------------------*/
void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
	//printf("Address Match %d\n\r",TransferDirection);

	if(TransferDirection ==  I2C_DIRECTION_RECEIVE)
	{
		slave_tx[5] += 1;
		HAL_I2C_Slave_Seq_Transmit_IT(hi2c, slave_tx, 6, I2C_FIRST_AND_LAST_FRAME);
	}

	if(TransferDirection == I2C_DIRECTION_TRANSMIT )
	{
		HAL_I2C_Slave_Seq_Receive_IT(hi2c, slave_rx, 6, I2C_FIRST_AND_LAST_FRAME);
	}
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	printf("Slave reception completed - Data: %s \n\r", slave_rx);
	memset(slave_rx,0, sizeof slave_rx);
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	printf("Slave transmission completed - Data: %s \n\r", slave_tx);
}

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
	HAL_I2C_EnableListen_IT(hi2c);
	// printf("Completed\n\r");
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	// printf("Master received: %s \n\r",master_rx);
	memset(master_rx,0, sizeof master_rx);
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	memset(master_tx,0,sizeof master_tx);
}

/* CAN	- Private user code ---------------------------------------------------------*/
void HAL_CAN_RxFifo0MsgPendingCallBack(CAN_HandleTypeDef *hcan)
{
	HAL_UART_Transmit(&huart2, (uint8_t *) "Message Receive\n\r", 17, 100) ;
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
	HAL_UART_Transmit(&huart2, (uint8_t *) "Message Broadcast\n\r", 20, 100) ;
}


/* Fix - Private user code ---------------------------------------------------------*/
int __io_putchar(int ch)
{
	uint8_t c[1];
	c[0] = ch & 0x00FF;
	HAL_UART_Transmit(&huart2, &*c, 1, 10);
	return ch;
}

int _write(int file,char *ptr, int len)
{
	int DataIdx;
	for(DataIdx= 0; DataIdx< len; DataIdx++)
	{
		__io_putchar(*ptr++);
	}
	return len;
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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
