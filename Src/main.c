
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
//#include "ringbuffer.h"
#define RBUFSIZE 500
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
	
//	ringbuffer_t rb;//ringbuffer variable
	uint8_t tbrawbuffer[RBUFSIZE];//ring buffer's raw buffer
	uint8_t userbuffer[RBUFSIZE];//the buffer for get data from ring buffer
	
	
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
	if((huart->Instance)==USART2){ //uart - ble.  receive from ble module //to mcu
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET);
		//HAL_GPIO_WritePin(GPIOF,GPIO_PIN_13,GPIO_PIN_RESET); //BCTS 0
		//HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_13);
	  //HAL_GPIO_WritePin(GPIOF,GPIO_PIN_12,GPIO_PIN_SET); //BRTS 1 
		/* 
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET);//LD3 ON
		while(tmp_from_BLE[0]!=4 && tmp_from_BLE[0]!='\0' &&tmp_from_BLE[0]!='\n' &&tmp_from_BLE[0]!='\r' ){
			rec_from_BLE[Ble_Cnt]=tmp_from_BLE[0];Ble_Cnt++;
		}
		
		if(tmp_from_BLE[0]==4 || tmp_from_BLE[0]!='\0' || tmp_from_BLE[0]!='\n'  ){
			rec_from_BLE[Ble_Cnt]=tmp_from_BLE[0];
			if(!strncmp((const char *)rec_from_BLE,(const char *)DISCON,(uint8_t)14)){
				HAL_UART_Transmit(&huart3,DISCON,sizeof(DISCON),200);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET); //LD3 OFF
			}//Print "TTM:DISCONNECT"
			else{			
				uint8_t *tmp;
				uint8_t cnt=0;
				HAL_UART_Transmit(&huart3,rec_from_BLE,Ble_Cnt,200);
			//	while(rec_from_BLE[cnt]!='\0'){
				//	cnt++;
				//}
				//tmp=(uint8_t *)calloc(Ble_Cnt-1,sizeof(uint8_t));
				//for(int i=0;i<Ble_Cnt;i++){
					//tmp[i]=rec_from_BLE[i];
				//}
				//HAL_UART_Transmit(&huart3,tmp,sizeof(tmp)-4,200);
				//free(tmp);
			}//else 
			Ble_Cnt=0; //re-init
		}//end if
		*/ 
		uint8_t TmpBle[12],TmpBleCase;
		TmpBleCase=rec_from_BLE[0];
		if(TmpBleCase>='0'&&TmpBleCase<='9'){
				sprintf((char *)TmpBle,"BLE:I'm%c\r\n",TmpBleCase);
				HAL_UART_Transmit(&huart3,TmpBle,10,100);

		}
			//HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET); //LD3 OFF
			//HAL_GPIO_WritePin(GPIOF,GPIO_PIN_13,GPIO_PIN_SET); //BCTS 1
			HAL_UART_Receive_IT(&huart2,rec_from_BLE,1);	
			HAL_UART_Receive_IT(&huart3,rec_from_PC,1);	
					
		}//USART2 End

	if((huart->Instance)==USART3){//uart - pc. receive from pc //to mcu
		//HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_SET);//LD2 ON
		
		//HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_SET);
		//HAL_GPIO_WritePin(GPIOF,GPIO_PIN_13,GPIO_PIN_SET); //BCTS 1
		//HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_13);
	 // HAL_GPIO_WritePin(GPIOF,GPIO_PIN_12,GPIO_PIN_RESET); //BRTS 0
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
		/*uart3
		if(!strncmp((const char *)rec_from_PC,(const char *)"TTM:",4)){
			HAL_UART_Transmit(&huart3,tmp,4,10);
			HAL_UART_Transmit(&huart6,tmp,4,10);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_SET);//LD2 ON
			uint8_t AT[30],tmp[20],i=4,cnt=0,at_size;
			for(int j=0;j<4;j++){
				AT[j]=rec_from_PC[j];
			}
			HAL_UART_Receive(&huart3,tmp,sizeof(tmp),2000);
			while(tmp[cnt]!='\n' &&tmp[cnt]!='\r'&&tmp[cnt]!='\0'){
				AT[i]=tmp[cnt];
				cnt++;
				i++;
			}
		*/
/*
			while(AT[i]!='\0'){
				HAL_UART_Receive(&huart3,tmp,1,5000);
				AT[i]=tmp[0];
				i++;
			}//while
*/
/*
			AT[i]='\r';
			AT[i+1]='\n';
			AT[i+2]='\0';
		HAL_UART_Transmit(&huart6,AT,i+3,100); //transmit pc to ble
		HAL_UART_Transmit(&huart3,AT,i+3,100); //print. echo
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_SET);//LD2 ON
		HAL_UART_Receive_IT(&huart3,rec_from_PC,4);//receive from PC
	}//end strncmp
	*/
	
		//HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_14);

		HAL_UART_Receive_IT(&huart3,rec_from_PC,1);//receive from PC
		HAL_UART_Receive_IT(&huart2,rec_from_BLE,1);//receive from PC
		//HAL_Delay(10);
		//HAL_GPIO_WritePin(GPIOF,GPIO_PIN_12,GPIO_PIN_SET); //BRTS 1
		
	}//end uart3
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);
}//end callback func

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
		
		//HAL_GPIO_WritePin(GPIOF,GPIO_PIN_12,GPIO_PIN_SET); //BRTS 1: Host has no data to send, or data has been sent. So the value of the signal should be set at ?1?.
		//HAL_GPIO_WritePin(GPIOF,GPIO_PIN_13,GPIO_PIN_RESET); //BCTS 0 :Module has data to send, and the host will receive the data.
		//HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_13);
		uint8_t test[5]="hihi";
		HAL_Delay(100);
		HAL_UART_Receive(&huart2,rec_from_BLE,14,30000);
		//HAL_UART_Transmit(&huart3,rec_from_BLE,14,100);
		HAL_UART_Transmit(&huart3,rec_from_BLE,14,100);
		//HAL_GPIO_WritePin(GPIOF,GPIO_PIN_12,GPIO_PIN_SET); //BRTS 1: Host has no data to send, or data has been sent. So the value of the signal should be set at "1"
		//uint8_t arrtest[100],i=0;
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
		
		/* MCU -> Mobile test working 
  	sprintf((char *)arrtest,"arr test=%d\n",i);
		HAL_UART_Transmit(&huart6,arrtest,11,100);
		HAL_UART_Transmit(&huart3,arrtest,11,100);
		i++;
		*/
		
	//	HAL_UART_Transmit(&huart3,arr,5,10);//uart to pc 
		//HAL_UART_Transmit(&huart6,arr,5,10);//uart to BLE
		//HAL_GPIO_WritePin(GPIOF,GPIO_PIN_12,GPIO_PIN_SET); //BRTS 1
		//HAL_GPIO_WritePin(GPIOF,GPIO_PIN_13,GPIO_PIN_SET); //BCTS 1
		//HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_13);
		//HAL_UART_Transmit(&huart2,test,4,10);
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
