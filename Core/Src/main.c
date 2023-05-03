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
#include "cmsis_os.h"
#include "lwip.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */
#include "lwip/sockets.h"
#include "lwip/inet.h"
/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SERVER_PORT 50000 // Настройка номера порта сервера


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */
uint8_t data_buffer [100];
char tcp_server_recvbuf [300];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
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
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  /* USER CODE BEGIN 2 */
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

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 13;
  RCC_OscInitStruct.PLL.PLLN = 195;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
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
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 4;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* обработка переключения состояния диода по кнопке добавлена для проверки функции отображения состояния светодиодов*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == USER_Btn_Pin){
		HAL_GPIO_TogglePin(GPIOB,LD3_Pin);
	}
	else{
		__NOP();
	}

}

static void server_thread (void * p_arg)
{
	struct sockaddr_in server_addr; // адрес сервера
	struct sockaddr_in conn_addr; // адрес подключения
	int sock_fd; // Сервер подключения
	int sock_conn; // запрошенный socket
	socklen_t addr_len; // длина адреса
	int err;
	int length;
	int num;



	 sock_fd = socket (AF_INET, SOCK_STREAM, IPPROTO_TCP); // установить новое соединение с сокетом
	 memset (& server_addr, 0, sizeof (server_addr)); // Очистить адрес сервера
	 server_addr.sin_family = AF_INET; // семейство адресов
	 server_addr.sin_addr.s_addr = htonl (INADDR_ANY);
	 server_addr.sin_port = htons (SERVER_PORT);

	 err = bind (sock_fd, (struct sockaddr *) & server_addr, sizeof (server_addr));
	 if (err <0) // Закрыть сокет, если привязка не удалась
	 {
			 closesocket (sock_fd);
	 }

	 err = listen (sock_fd, 1); // прослушивание запросов на подключение
	 if (err <0) // Закрыть сокет, если прослушивание не удалось
	 {
			 closesocket (sock_fd);
	 }
	 addr_len = sizeof (struct sockaddr_in);

	 sock_conn = accept (sock_fd, (struct sockaddr *) & conn_addr, & addr_len); // Подключиться к прослушиваемому запросу и присвоить статус sock_conn

	 if (sock_conn <0) // Если состояние меньше 0, это указывает на то, что соединение неисправно
	 {
		closesocket(sock_fd);
	 }
	 else send (sock_conn, "connect success! \n \r", 22, 0);

	while (1)//TODO:исправить, то что сервер читает escape последовательности
	{
		memset (data_buffer, 0, sizeof (data_buffer)); // Очистить приемный буфер

		length = recv (sock_conn, (unsigned int *) data_buffer, 100, 0); // помещаем полученные данные в приемный буфер
        if(length == 0){
        	continue;
        }

		for (num = 0; num <100; num ++)
		{
			tcp_server_recvbuf[num]=data_buffer[num];
		}

		if(tcp_server_recvbuf[0] == NULL){
			send (sock_conn, "empty message \n ", strlen ("empty message \n"), 1);
		}
		else if(tcp_server_recvbuf[0]=='L' & tcp_server_recvbuf[1]=='E' & tcp_server_recvbuf[2]=='D' & tcp_server_recvbuf[3]=='2' & tcp_server_recvbuf[4]=='O' & tcp_server_recvbuf[5]=='F' & tcp_server_recvbuf[5]=='F')
		{
			HAL_GPIO_WritePin (GPIOB, LD2_Pin, GPIO_PIN_RESET);
			send (sock_conn, "LED2 выключен \n \r", strlen ("LED2 выключен \n \r"), 1);
	    }
		else if(tcp_server_recvbuf[0]=='L' & tcp_server_recvbuf[1]=='E' & tcp_server_recvbuf[2]=='D' & tcp_server_recvbuf[3]=='2' & tcp_server_recvbuf[4]=='O' & tcp_server_recvbuf[5]=='N')
		{
			HAL_GPIO_WritePin (GPIOB, LD2_Pin, GPIO_PIN_SET);
			send (sock_conn, "LED2 включен \n \r",  strlen("LED2 включен \n \r"),1);
		}
		else if(tcp_server_recvbuf[0]=='L' & tcp_server_recvbuf[1]=='E' & tcp_server_recvbuf[2]=='D' & tcp_server_recvbuf[3]=='3' & tcp_server_recvbuf[4]=='O' & tcp_server_recvbuf[5]=='F' & tcp_server_recvbuf[5]=='F')
		{
			HAL_GPIO_WritePin (GPIOB, LD3_Pin, GPIO_PIN_RESET);
			send (sock_conn, "LED3 выключен \n \r", strlen("LED3 выключен \n \r"), 1);
		}
		else if(tcp_server_recvbuf[0]=='L' & tcp_server_recvbuf[1]=='E'&tcp_server_recvbuf[2]=='D' & tcp_server_recvbuf[3]=='3'&tcp_server_recvbuf[4]=='O'&tcp_server_recvbuf[5]=='N')
		{
			HAL_GPIO_WritePin (GPIOB, LD3_Pin, GPIO_PIN_SET);
			send (sock_conn, "LED3 включен \n \r", strlen("LED3 включен \n \r"), 1);
		}
		else if(tcp_server_recvbuf[0] =='S' & tcp_server_recvbuf[1]=='T' & tcp_server_recvbuf[2]=='A' & tcp_server_recvbuf[3]=='T'){
			if(HAL_GPIO_ReadPin(GPIOB, LD2_Pin) == GPIO_PIN_SET & HAL_GPIO_ReadPin(GPIOB, LD2_Pin) ==GPIO_PIN_SET){
				send (sock_conn, "LED2 и LED3 включен \n \r", strlen("LED2 и LED3 включен \n \r"), 1);
			}
			if(HAL_GPIO_ReadPin(GPIOB, LD2_Pin) == GPIO_PIN_RESET & HAL_GPIO_ReadPin(GPIOB, LD2_Pin) ==GPIO_PIN_SET){
				send (sock_conn, "LED2 включен и LED3 включен \n \r", strlen("LED2 включен и LED3 включен \n \r"), 1);
			}
			if(HAL_GPIO_ReadPin(GPIOB, LD2_Pin) == GPIO_PIN_RESET & HAL_GPIO_ReadPin(GPIOB, LD2_Pin) ==GPIO_PIN_RESET){
				send (sock_conn, "LED2 и LED3 выключен \n \r", strlen ("LED2 и LED3 выключен \n \r"), 1);
			}
			if(HAL_GPIO_ReadPin(GPIOB, LD2_Pin) == GPIO_PIN_SET & HAL_GPIO_ReadPin(GPIOB, LD2_Pin) ==GPIO_PIN_RESET){
				send (sock_conn, "LED2 включен и LED3 выключен \n \r", strlen ("LED2 включен и LED3 выключен \n \r"), 1);
			}
		}
		else if(tcp_server_recvbuf[0] =='E' & tcp_server_recvbuf[1]=='X' & tcp_server_recvbuf[2]=='T'){
			closesocket(sock_conn);
		}
		else if(tcp_server_recvbuf[0] =='H' & tcp_server_recvbuf[1]=='E' & tcp_server_recvbuf[2]=='L' & tcp_server_recvbuf[3]=='P'){
			send (sock_conn, "--------------------- \n \r", strlen ("--------------------- \n \r"), 1);
			send (sock_conn, "LED2ON/LED3ON-команды включения светодиодов \n \r", strlen ("LED2ON/LED3ON-команды включения светодиодов \n \r"), 1);
			send (sock_conn, "LED2OFF/LED3OFF-команды выключения светодиодов \n \r", strlen ("LED2OFF/LED3OFF-команды выключения светодиодов \n \r"), 1);
			send (sock_conn, "STAT-команда для вывода состояния светодиодов \n \r", strlen ("STAT-команда для вывода состояния светодиодов \n \r"), 1);
			send (sock_conn, "EXT-завершить сеанс \n \r", strlen ("EXT-завершить сеанс \n \r"), 1);
			send (sock_conn, "--------------------- \n \r", strlen ("--------------------- \n \r"), 1);
		}
		/*
		else{
			send (sock_conn, "неправильная команда для вывод списка команд введите HELP \n \r", strlen ("неправильная команда для вывод списка команд введите HELP \n \r"), 1);
		}*/
	}
}

 void tcp_serv_init (void) // инициализация TCP-сервера
{
	sys_thread_new("tcp_server_thread",  server_thread, NULL, DEFAULT_THREAD_STACKSIZE, DEFAULT_THREAD_PRIO - 1);
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for LWIP */
  MX_LWIP_Init();
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	tcp_serv_init();
    osDelay(100);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
