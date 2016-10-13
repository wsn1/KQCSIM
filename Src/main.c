/**
******************************************************************************
* File Name          : main.c
* Description        : Main program body
******************************************************************************
*
* Copyright (c) 2016 STMicroelectronics International N.V. 
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
#include "stm32l1xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include "string.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart3_rx;

osThreadId defaultTaskHandle;
osThreadId recvTaskHandle;
osThreadId sendTaskHandle;
osMessageQId recvQueueHandle;

/* USER CODE BEGIN PV */
#define SI_DMA_BUFSZ 32
typedef  enum _Uart_Reason
{ Uart1_Reason_TxCplt,Uart2_Reason_TxCplt, Uart_Reason_RxCplt, Uart_Reason_TxHalfCplt, 
Uart_Reason_RxHalfCplt,Uart1_Reason_Error, Uart2_Reason_Error }
Uart_Reason;
typedef  enum _cmd_recv_state
{
	CMD0,CMD1,CMD2,CMD3,CMD4,CMD5
}cmd_recv_state;
/* Private variables ---------------------------------------------------------*/
typedef struct  _global_info
{
	uint32_t cycles;
	uint8_t  status;
	uint8_t  recvCode;
	uint8_t  errorCount;
	GPIO_PinState led1State;
	uint8_t  uart3dma_cndtr_handled;
	uint16_t uart3recvbuff[SI_DMA_BUFSZ] ;
	uint32_t uart3ReadTaskCount;
	uint8_t  errRecvCnt[4];
	uint8_t  uart3errCountLast;
	uint8_t  uart3errCount;
	uint8_t  uart3recvCpltCount;
	uint8_t  uart3recvHalfCpltCount;
	uint32_t recvDatCount;
	int32_t  weight;
	int32_t  w2;
	uint8_t  dat[4];
}global_info;
global_info g_info;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM7_Init(void);
void StartDefaultTask(void const * argument);
void StartRecvTask(void const * argument);
void StartSendTask(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void processword(uint16_t word)
{
 uint16_t bit9=(word>>8)&1;
	uint16_t byte=word&0xff;
	
	g_info.dat[2]=g_info.dat[1];
	g_info.dat[1]=g_info.dat[0];
	g_info.dat[0]=byte;


	switch (g_info.recvCode)
	{
	case CMD0:
		if(bit9==0)
		{
			g_info.recvCode=CMD1;
		} else {
			g_info.errRecvCnt[0]++;
		}
		break;
	case CMD1:
		if(bit9==0)
		{
			g_info.recvCode=CMD2;
		} else {
			g_info.errRecvCnt[1]++;
		}
		break;
	case CMD2:
		if(bit9==1)
		{
			int32_t w2=g_info.dat[0]+((uint32_t)g_info.dat[1]<<8)+(((int32_t)(g_info.dat[2]>>6)&0x3)<<16);
			if((g_info.dat[2]>>3)==1)
			{
				w2=-w2;
			}
			if(g_info.w2+1!=w2)
			{
				g_info.errorCount++;
				g_info.errRecvCnt[3]++;
			}
			g_info.w2=w2;
			g_info.recvDatCount++;
		} else {
			g_info.errRecvCnt[2]++;
		}
		g_info.recvCode=CMD0;
		break;
	}
	
}
void OnRecvCmd(const uint16_t* first, const uint16_t* last)
{
	while (first < last) {
		processword(*first);
		first++;		
	}
}

//------------------------------------------------------------------------------
//{ UART Callbacks
//------------------------------------------------------------------------------
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart==&huart1)//
	{
		//g_info.uart1sentCount++;
		//uint32_t info=Uart1_Reason_TxCplt;
		//osMessagePut(g_info.m_uartTxCpltmsgQ,info,0);
	}
}

//------------------------------------------------------------------------------
void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart==&huart1)//
	{
		//g_info.uart1sentCount++;
	}
}

//------------------------------------------------------------------------------
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart==&huart3)//
	{
		uint32_t info=Uart_Reason_RxCplt;
		g_info.uart3recvCpltCount++;
		osMessagePut(recvQueueHandle,info,0);
	}  
}

//------------------------------------------------------------------------------
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart==&huart3)//
	{
		uint32_t info=Uart_Reason_RxHalfCplt;
		osMessagePut(recvQueueHandle,info,0);
		g_info.uart3recvHalfCpltCount++;	
	} 
}

//------------------------------------------------------------------------------
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if(huart==&huart3)//
	{
		//uint32_t info=Uart1_Reason_Error;
		g_info.uart3errCount++;
	}
}
/* USER CODE END 0 */

int main(void)
{

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_USART3_UART_Init();
	MX_USART1_UART_Init();
	MX_TIM7_Init();

	/* USER CODE BEGIN 2 */
	memset(&g_info,0,sizeof(global_info));
	/* USER CODE END 2 */

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* Create the thread(s) */
	/* definition and creation of defaultTask */
	osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 64);
	defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

	/* definition and creation of recvTask */
	osThreadDef(recvTask, StartRecvTask, osPriorityNormal, 0, 64);
	recvTaskHandle = osThreadCreate(osThread(recvTask), NULL);

	/* definition and creation of sendTask */
	osThreadDef(sendTask, StartSendTask, osPriorityNormal, 0, 64);
	sendTaskHandle = osThreadCreate(osThread(sendTask), NULL);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* Create the queue(s) */
	/* definition and creation of recvQueue */
	osMessageQDef(recvQueue, 8, uint16_t);
	recvQueueHandle = osMessageCreate(osMessageQ(recvQueue), NULL);

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */


	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

	}
	/* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;

	/**Configure the main internal regulator output voltage 
	*/
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/**Initializes the CPU, AHB and APB busses clocks 
	*/
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/**Initializes the CPU, AHB and APB busses clocks 
	*/
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
		|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV16;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
	{
		Error_Handler();
	}

	/**Configure the Systick interrupt time 
	*/
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

	/**Configure the Systick 
	*/
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* TIM7 init function */
static void MX_TIM7_Init(void)
{

	TIM_MasterConfigTypeDef sMasterConfig;

	htim7.Instance = TIM7;
	htim7.Init.Prescaler = 0;
	htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim7.Init.Period = 0;
	if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
	{
		Error_Handler();
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

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

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

	huart3.Instance = USART3;
	huart3.Init.BaudRate = 600;
	huart3.Init.WordLength = UART_WORDLENGTH_9B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart3) != HAL_OK)
	{
		Error_Handler();
	}

}

/** 
* Enable DMA controller clock
*/
static void MX_DMA_Init(void) 
{
	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel3_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
	/* DMA1_Channel5_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : LED1_Pin */
	GPIO_InitStruct.Pin = LED1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

	/* USER CODE BEGIN 5 */
	/* Infinite loop */
	for(;;)
	{
		
		g_info.cycles++;
		if(g_info.led1State==GPIO_PIN_RESET)
		{
			g_info.led1State=GPIO_PIN_SET;
			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, g_info.led1State);
		} else 
			if((g_info.cycles%20)==0)
			{
				g_info.led1State=GPIO_PIN_RESET;
				HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, g_info.led1State);
				g_info.weight++;
				if(g_info.weight>1000)
				{
					g_info.weight=-1000;
				}
			}
			if((g_info.cycles%20)==0)
			{
				uint16_t dat[3];
				int32_t w=g_info.weight;
				if(w<0)
				{
					w=-w;
				} 
				dat[2]=(w&0xff)|0x100;
				dat[1]=(w>>8)&0xff;
				dat[0]=((w>>16)&0x3)<<6;
				if(g_info.weight<0)
					dat[0]|=0x8;
				dat[0]|=1;

				HAL_UART_Transmit(&huart3,(uint8_t*)dat,3,1000);
			}
			osDelay(50);
	}
	/* USER CODE END 5 */ 
}

/* StartRecvTask function */
void StartRecvTask(void const * argument)
{
	/* USER CODE BEGIN StartRecvTask */
	/* Infinite loop */
	g_info.uart3dma_cndtr_handled = SI_DMA_BUFSZ; // Note: CNDTR counts backwards starting from BUFSZ

	HAL_UART_Receive_DMA(&huart3,(uint8_t*)g_info.uart3recvbuff, SI_DMA_BUFSZ);

	for(;;)
	{
		osEvent event=osMessageGet(recvQueueHandle,100);
		if ((event.status == osEventTimeout) ||(event.status == osOK) ||( event.status == osEventMessage))
		{

			uint32_t cndtr = huart3.hdmarx->Instance->CNDTR;
			if (cndtr == g_info.uart3dma_cndtr_handled)
			{
				osThreadYield();
				continue;
			}
			g_info.uart3ReadTaskCount++;
			if(g_info.uart3errCountLast!=g_info.uart3errCount)
			{
				g_info.uart3errCountLast=g_info.uart3errCount;
			}
			uint16_t* begin = &g_info.uart3recvbuff[SI_DMA_BUFSZ - g_info.uart3dma_cndtr_handled];
			uint16_t* end   = &g_info.uart3recvbuff[SI_DMA_BUFSZ - cndtr];

			if (end > begin) {
				OnRecvCmd(begin,end);
			}
			else {
				OnRecvCmd(begin,&g_info.uart3recvbuff[SI_DMA_BUFSZ]);
				OnRecvCmd(g_info.uart3recvbuff,end);
			}
			g_info.uart3dma_cndtr_handled = cndtr;
		}
	}
	/* USER CODE END StartRecvTask */
}

/* StartSendTask function */
void StartSendTask(void const * argument)
{
	/* USER CODE BEGIN StartSendTask */
	/* Infinite loop */
	for(;;)
	{
		osDelay(100);
	}
	/* USER CODE END StartSendTask */
}

/**
* @brief  Period elapsed callback in non blocking mode
* @note   This function is called  when TIM6 interrupt took place, inside
* HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
* a global variable "uwTick" used as application time base.
* @param  htim : TIM handle
* @retval None
*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM6) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

/**
* @brief  This function is executed in case of error occurrence.
* @param  None
* @retval None
*/
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler */
	/* User can add his own implementation to report the HAL error return state */
	while(1) 
	{
	}
	/* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
* @brief Reports the name of the source file and the source line number
* where the assert_param error has occurred.
* @param file: pointer to the source file name
* @param line: assert_param error line source number
* @retval None
*/
void assert_failed(uint8_t* file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */

}

#endif

/**
* @}
*/ 

/**
* @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
