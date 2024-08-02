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
#include <stdio.h>
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
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
volatile uint32_t cnt = 0;
char str[20] = {0};
uint8_t rvData;
uint8_t speakON = 0;
uint8_t btn[3] = {0, 0, 0};
uint8_t outpin[4] = {0, 0, 0, 0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void switchBTN(uint8_t* btn);
int readBTN(GPIO_TypeDef* port, uint16_t pin);
void setOut(void);
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
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart1, &rvData, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //HAL_GPIO_WritePin(LED_PWR_GPIO_Port, LED_PWR_Pin, PinState)

	  if(readBTN(BTN_3P_GPIO_Port, BTN_3P_Pin)) switchBTN(&btn[0]);
	  if(readBTN(BTN_PS_GPIO_Port, BTN_PS_Pin)) switchBTN(&btn[1]);
	  if(readBTN(BTN_SW_GPIO_Port, BTN_SW_Pin)) switchBTN(&btn[2]);

	  setOut();
	  HAL_Delay(50);
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

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
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
  huart1.Init.BaudRate = 9600;
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
  HAL_GPIO_WritePin(LED13_GPIO_Port, LED13_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, OUT_3P_Pin|OUT_PS_Pin|OUT_SW0_Pin|OUT_SW1_Pin
                          |LED_3P_Pin|LED_PS_Pin|LED_SW0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_PWR_GPIO_Port, LED_PWR_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_SW1_Pin|SPEAKER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED13_Pin */
  GPIO_InitStruct.Pin = LED13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED13_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OUT_3P_Pin OUT_PS_Pin OUT_SW0_Pin OUT_SW1_Pin
                           LED_PWR_Pin LED_3P_Pin LED_PS_Pin LED_SW0_Pin */
  GPIO_InitStruct.Pin = OUT_3P_Pin|OUT_PS_Pin|OUT_SW0_Pin|OUT_SW1_Pin
                          |LED_PWR_Pin|LED_3P_Pin|LED_PS_Pin|LED_SW0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_SW1_Pin SPEAKER_Pin */
  GPIO_InitStruct.Pin = LED_SW1_Pin|SPEAKER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : BTN_3P_Pin BTN_PS_Pin BTN_SW_Pin */
  GPIO_InitStruct.Pin = BTN_3P_Pin|BTN_PS_Pin|BTN_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int readBTN(GPIO_TypeDef* port, uint16_t pin) // считывает значение кнопки с здаержкой 1сек.// возвр 1 если прожата, 0 если нет
{
	cnt = 0;
	while(HAL_GPIO_ReadPin(port, pin) == 1 && cnt < 850){
		if(pin == BTN_SW_Pin){
			if(btn[0] || btn[1]){
				//speaker
				if(speakON == 0){
					HAL_GPIO_WritePin(SPEAKER_GPIO_Port, SPEAKER_Pin, GPIO_PIN_SET);
					speakON = 1;
				}

				char* msg = "tak nizya\r\n";
				HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
				HAL_Delay(100);
				return 0;
			}
		}
		cnt++;
		HAL_Delay(1);
	}
	if(speakON){
		HAL_GPIO_WritePin(SPEAKER_GPIO_Port, SPEAKER_Pin, GPIO_PIN_RESET);
		speakON = 0;
	}

	if(cnt > 800){
		HAL_GPIO_TogglePin(LED13_GPIO_Port, LED13_Pin);
		return 1;
	}
	return 0;
}

void setOut(void)  //ФОРМ�?РОВАН�?Е ВЫХОДНОГО С�?ГНАЛА
{
	if(btn[0]){
		HAL_GPIO_WritePin(OUT_3P_GPIO_Port, OUT_3P_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED_3P_GPIO_Port, LED_3P_Pin, GPIO_PIN_SET);
		outpin[0] = 1;
	}

	else{
		HAL_GPIO_WritePin(OUT_3P_GPIO_Port, OUT_3P_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_3P_GPIO_Port, LED_3P_Pin, GPIO_PIN_RESET);
		outpin[0] = 0;
	}
	if(btn[1]){
		HAL_GPIO_WritePin(OUT_PS_GPIO_Port, OUT_PS_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED_PS_GPIO_Port, LED_PS_Pin, GPIO_PIN_SET);
		outpin[1] = 1;
	}
	else{
		HAL_GPIO_WritePin(OUT_PS_GPIO_Port, OUT_PS_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_PS_GPIO_Port, LED_PS_Pin, GPIO_PIN_RESET);
		outpin[1] = 0;
	}
	if(btn[2]){
		HAL_GPIO_WritePin(OUT_SW0_GPIO_Port, OUT_SW0_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(OUT_SW1_GPIO_Port, OUT_SW1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED_SW0_GPIO_Port, LED_SW0_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_SW1_GPIO_Port, LED_SW1_Pin, GPIO_PIN_SET);
		outpin[2] = 1;
		outpin[3] = 1;
	}
	else{
		HAL_GPIO_WritePin(OUT_SW0_GPIO_Port, OUT_SW0_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(OUT_SW1_GPIO_Port, OUT_SW1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_SW0_GPIO_Port, LED_SW0_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED_SW1_GPIO_Port, LED_SW1_Pin, GPIO_PIN_SET);
		outpin[2] = 0;
		outpin[3] = 0;
	}
}

void switchBTN(uint8_t* btn)// МЕНЯЕТ ЗНАЧЕН�?Е КНОПК�? НА ПРОТ�?ВОПОЛОЖНОЕ 1->0 0->1
{
	if(*btn)
		*btn = 0;
	else
		*btn = 1;
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) { //ВЫВОД В КОНСОЛЬ ДО ЗАПРОСУ 1, 2
	if (huart->Instance == USART1) {
		char* sep = "-----------------------\r\n";
		if (rvData == '1') {
			HAL_UART_Transmit(&huart1, (uint8_t*)sep, strlen(sep), HAL_MAX_DELAY);
			for(int i = 0; i < 3; i++){
				sprintf(str, "BTN%d VAL = %d\r\n", i+1, btn[i]);
				HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
			}
			HAL_UART_Transmit(&huart1, (uint8_t*)sep, strlen(sep), HAL_MAX_DELAY);
		}
		else if (rvData == '2') {
			HAL_UART_Transmit(&huart1, (uint8_t*)sep, strlen(sep), HAL_MAX_DELAY);
			for(int i = 0; i < 4; i++){
				sprintf(str, "OUT%d VAL = %d\r\n", i+1, outpin[i]);
				HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
			}
			HAL_UART_Transmit(&huart1, (uint8_t*)sep, strlen(sep), HAL_MAX_DELAY);
		}
		HAL_UART_Receive_IT(huart, &rvData, 1);
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
