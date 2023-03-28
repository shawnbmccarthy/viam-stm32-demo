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
#include <pb.h>
#include <pb_encode.h>
#include <pb_decode.h>
#include <sensor.pb.h>
#include <motor.pb.h>
#include <wrapper.pb.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//
// viam defines
//
#define V_OK   0
#define V_ERR  1
#define V_SENT 2
#define V_RCV  3
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t clientMsgSz[1] = {0};
uint8_t clientMsg[60] = {0};
uint8_t retMsg[60] = {0};

float counter = 0;
float temperatureSensor = 0;
float batteryLevel = 100;

bool isMoving = false;
bool isPowered = false;
float position = 0.0;
float currentPower = 0.0;
float currentPosition = 0.0;

//start from false
bool gotSizePacket = true;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *);
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
  MX_USART1_UART_Init();
  MX_TIM2_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  __HAL_UART_CLEAR_OREFLAG(&huart1);
  if (HAL_UART_Receive_IT(&huart1, clientMsgSz, 1) != HAL_OK) {
	  Error_Handler();
  }

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_TIM2;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Tim2ClockSelection = RCC_TIM2CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* EXTI15_10_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
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
  htim2.Init.Prescaler = 0;
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, Left_Encoder_Pin|Right_Encoder_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Left_Encoder_Pin Right_Encoder_Pin */
  GPIO_InitStruct.Pin = Left_Encoder_Pin|Right_Encoder_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Right_Motor_Pin Right_MotorB1_Pin Right_MotorB2_Pin */
  GPIO_InitStruct.Pin = Right_Motor_Pin|Right_MotorB1_Pin|Right_MotorB2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Left_Motor_Pin Left_MotorC8_Pin Left_MotorC9_Pin */
  GPIO_InitStruct.Pin = Left_Motor_Pin|Left_MotorC8_Pin|Left_MotorC9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/*
 * Entry point to our message passing
 * TODO: Cleanup code
 * - move to sensor.h/.c for sensor processing
 * - move to motor.h/.c for motor processing
 * - pin management/motor controller management
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	/*
	 *  on first run we should always get the size, we use this
	 *  size from the clientMsgSz[0] to get populate the client message
	 */
	if (gotSizePacket)
	{
		// prepare for client message in the next szRecv bytes
		gotSizePacket = false;
		uint8_t szRecv;
		szRecv = clientMsgSz[0];
		__HAL_UART_CLEAR_OREFLAG(huart);
		HAL_UART_Receive_IT(huart, clientMsg, szRecv);
	} else {
		// process client message
		// need to gather more requirements
		uint8_t szBuffer[1] = {0};
		WrappedRequest request = WrappedRequest_init_zero;
		WrappedResponse response = WrappedResponse_init_zero;
		SensorRequest sensorRequest;
		MotorRequest motorRequest;
		pb_istream_t requestStream = pb_istream_from_buffer(clientMsg, clientMsgSz[0]);
		pb_ostream_t responseStream = pb_ostream_from_buffer(retMsg, sizeof(retMsg));

		if (!pb_decode(&requestStream, WrappedRequest_fields, &request)) {
			Error_Handler();
		}
		switch(request.which_msg){
		case WrappedRequest_sensorRequest_tag:
			sensorRequest = request.msg.sensorRequest;
			response.which_msg = WrappedResponse_sensorResponse_tag;
			if (sensorRequest.sensor == SensorType_Temperature) {
				response.msg.sensorResponse.value = temperatureSensor++;
				response.status = true;
				if (temperatureSensor >= 100) {temperatureSensor = 0;}
			} else if (sensorRequest.sensor == SensorType_Battery) {
				response.msg.sensorResponse.value = batteryLevel--;
				response.status = true;
				if (batteryLevel <= 0) {batteryLevel = 100;}
			} else if (sensorRequest.sensor == SensorType_Counter) {
				response.msg.sensorResponse.value = counter++;
				response.status = true;
			} else {
				Error_Handler();
			}
			break;
		case WrappedRequest_motorRequest_tag:
			motorRequest = request.msg.motorRequest;
			response.which_msg = WrappedResponse_motorResponse_tag;
			response.msg.motorResponse.action = motorRequest.action;
			switch(motorRequest.action){
			case Action_NoAction:
				break;
			case Action_SetPower:
				if (motorRequest.has_power) {
					if (motorRequest.power == 0.0) {
						isMoving = false;
						isPowered = false;
					} else {
						isMoving = true;
						isPowered = true;
					}
					currentPower = motorRequest.power;
					response.msg.motorResponse.has_val_f = true;
					response.msg.motorResponse.val_f = currentPower;
					response.status = true;
				} else {
					Error_Handler();
				}
				break;
			case Action_GoFor:
				response.status = true;
				break;
			case Action_GoTo:
				response.status = true;
				break;
			case Action_resetZeroPosition:
				response.status = true;
				break;
			case Action_getPosition:
				response.msg.motorResponse.has_val_f = true;
				response.msg.motorResponse.val_f = currentPosition;
				response.status = true;
				break;
			case Action_Stop:
				response.status = true;
				break;
			case Action_IsPowered:
				response.msg.motorResponse.has_val_b = true;
				response.msg.motorResponse.has_val_f = true;
				response.msg.motorResponse.val_b = isPowered;
				response.msg.motorResponse.val_f = currentPower;
				response.status = true;
				break;
			case Action_IsMoving:
				response.msg.motorResponse.has_val_b = true;
				response.msg.motorResponse.val_b = isMoving;
				response.status = true;
				break;
			default:
				Error_Handler();
			}
		}
		if (!pb_encode(&responseStream, WrappedResponse_fields, &response)) {
			Error_Handler();
		}

		szBuffer[0] = responseStream.bytes_written;

		HAL_UART_Transmit(huart, szBuffer, 1, 1000);
		HAL_UART_Transmit(huart, retMsg, responseStream.bytes_written, 1000);

		gotSizePacket = true;
		__HAL_UART_CLEAR_OREFLAG(huart);
		HAL_UART_Receive_IT(huart, clientMsgSz, 1);
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
