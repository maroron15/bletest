

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
	uint8_t rec_from_BLE[500];
	uint8_t tmp_from_BLE[1];
	uint8_t rec_from_PC[500];
	uint8_t tmp_from_PC[1];
	uint8_t DISCON[17]="TTM:DISCONNECT\r\n\0";
	uint8_t Ble_Cnt,Pc_Cnt;
	
	
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{ 
			/*
		BRTS
			As the data sending requests (for module wake-up)
			0: Host has data to send, and module will wait for data transmission from the host so will not sleep
			1: Host has no data to send, or data has been sent. So the value of the signal should be set at "1".
		BCTS
			Data input signal (for host wake-up, optional)
			0: Module has data to send, and the host will receive the data.
			1: Module has no data to send, or data has been sent, and the value of the signal will be set at "1".
		
		HAL_GPIO_WritePin(GPIOF,GPIO_PIN_13,GPIO_PIN_RESET); //BCTS 0
	  HAL_GPIO_WritePin(GPIOF,GPIO_PIN_12,GPIO_PIN_SET); //BRTS 1 
		*/
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET); //red led on when interrupted 
	if((huart->Instance)==USART2){ //usart - ble.  receive from ble module //to mcu
		uint8_t TmpBle[12],TmpBleCase;
		TmpBleCase=rec_from_BLE[0];
		if(TmpBleCase>='0'&&TmpBleCase<='9'){
				sprintf((char *)TmpBle,"BLE:I'm%c\r\n",TmpBleCase);
				HAL_UART_Transmit(&huart3,TmpBle,10,100);

		}
			HAL_UART_Receive_IT(&huart2,rec_from_BLE,1);	
			HAL_UART_Receive_IT(&huart3,rec_from_PC,1);	
					
		}//USART2 End

	if((huart->Instance)==USART3){//uart - pc. receive from pc //to mcu
		
		uint8_t TmpPc[12],TmpPcCase;
		TmpPcCase=rec_from_PC[0];
		switch(TmpPcCase){
			case '0':
				sprintf((char *)TmpPc,"PC:I'm%d\r\n",0);
				HAL_UART_Transmit(&huart2,TmpPc,9,100);
				break;
			case '1':
				sprintf((char *)TmpPc,"PC:I'm%d\r\n",1);
				HAL_UART_Transmit(&huart2,TmpPc,9,100);
				break;
			case '2':
				sprintf((char *)TmpPc,"PC:I'm%d\r\n",2);
				HAL_UART_Transmit(&huart2,TmpPc,9,100);
				break;
			case '3':
				sprintf((char *)TmpPc,"PC:I'm%d\r\n",3);
				HAL_UART_Transmit(&huart2,TmpPc,9,100);
				break;
			case '4':
				sprintf((char *)TmpPc,"PC:I'm%d\r\n",4);
				HAL_UART_Transmit(&huart2,TmpPc,9,100);
				break;
			case '5':
				sprintf((char *)TmpPc,"PC:I'm%d\r\n",5);
				HAL_UART_Transmit(&huart2,TmpPc,9,100);
				break;
		}

		HAL_UART_Receive_IT(&huart3,rec_from_PC,1);//receive from PC
		HAL_UART_Receive_IT(&huart2,rec_from_BLE,1);//receive from PC
		
	}//end uart3
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);//red led off
}//end callback func

/* USER CODE END 0 */

int main(void)
{
  /* USER CODE BEGIN 1 */
	
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
//	ringbuffer_initialize(&rb,(uint8_t)RBUFSIZE);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
//	uint8_t arr[5]="Abcd";
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_7,GPIO_PIN_RESET);//0: ENABLE 
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET); //LD3 OFF   LED using for check

		HAL_UART_Receive(&huart2,rec_from_BLE,14,30000);
		HAL_UART_Transmit(&huart3,rec_from_BLE,14,100);

		HAL_UART_Receive_IT(&huart2,tmp_from_BLE,1);//receive from ble module
		HAL_UART_Receive_IT(&huart3,tmp_from_PC,1);//receive from PC
		
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		/*
		BRTS
			As the data sending requests (for module wake-up)
			0: Host has data to send, and module will wait for data transmission from the host so will not sleep
			1: Host has no data to send, or data has been sent. So the value of the signal should be set at "1".
		BCTS
			Data input signal (for host wake-up, optional)
			0: Module has data to send, and the host will receive the data.
			1: Module has no data to send, or data has been sent, and the value of the signal will be set at "1".
		
		HAL_GPIO_WritePin(GPIOF,GPIO_PIN_13,GPIO_PIN_RESET); //BCTS 0
	  HAL_GPIO_WritePin(GPIOF,GPIO_PIN_12,GPIO_PIN_SET); //BRTS 1 
		*/
	
		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_7);
		HAL_Delay(500);
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

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD_RED_Pin|LD_BLUE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Connection_status_indicator_Pin Sleep_mode_indicator_Pin */
  GPIO_InitStruct.Pin = Connection_status_indicator_Pin|Sleep_mode_indicator_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : LD_RED_Pin LD_BLUE_Pin */
  GPIO_InitStruct.Pin = LD_RED_Pin|LD_BLUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : EN_Pin */
  GPIO_InitStruct.Pin = EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(EN_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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
