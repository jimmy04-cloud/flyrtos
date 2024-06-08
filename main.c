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
#include "cmsis_os.h"
#include "can.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

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

CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint32_t TxMailbox;
uint8_t TxData[8];
uint8_t RxData[8];
// 定义队列和互斥锁
#define QUEUE_LENGTH 10
#define QUEUE_ITEM_SIZE sizeof(uint8_t)

#define ESC_INITIAL_PULSE 1100
#define ESC_UNLOCK_PULSE 2000
#define ESC_MIN_PULSE 1000
#define ESC_MAX_PULSE 2000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// 定义脉宽变量和队列句柄
uint16_t pulse_width_1 = ESC_INITIAL_PULSE;  // 通道1当前脉宽
uint16_t pulse_width_4 = ESC_INITIAL_PULSE;  // 通道4当前脉宽
uint16_t rotor_angle = 0;                    // 旋翼角度

QueueHandle_t queue;
osMutexId queueMutexHandle;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
void MX_USART1_UART_Init(void);


void StartDefaultTask(void *argument);
void StartPWMTask(void *argument);
void StartUartTask(void *argument);
void StartCanTask(void *argument);
void reset_pulse_widths(void); // 函数声明
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//#define COMMAND_A1 1
//#define COMMAND_A4 2
//#define COMMAND_B1 3
//#define COMMAND_B4 4
//#define COMMAND_A   5
//#define COMMAND_B   6
//#define COMMAND_S   7
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
  MX_TIM4_Init();
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */
    // 创建队列
    queue = xQueueCreate(QUEUE_LENGTH, QUEUE_ITEM_SIZE);
		
		 // 创建任务锁
    osMutexDef(queueMutex);
    queueMutexHandle = osMutexCreate(osMutex(queueMutex));
		
    // 创建FreeRTOS任务
    xTaskCreate(StartPWMTask, "PWM Task", 128, NULL, osPriorityBelowNormal, NULL);
    xTaskCreate(StartUartTask, "UART Task", 128, NULL,osPriorityNormal , NULL);
		 xTaskCreate(StartCanTask, "CAN Task", 128, NULL, osPriorityNormal, NULL);  // 创建CAN任务

    // 启动调度器
    vTaskStartScheduler();

    // 主循环（通常不会运行到这里）
    while (1)
    {
    }

	
}
void StartPWMTask(void *argument)
{
    // 启动定时器PWM
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);

    // 解锁电调过程
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, ESC_UNLOCK_PULSE);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, ESC_UNLOCK_PULSE);
    vTaskDelay(pdMS_TO_TICKS(500));
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, ESC_MIN_PULSE);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, ESC_MIN_PULSE);
 

    // 设置初始脉宽
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pulse_width_1);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, pulse_width_4);

    uint8_t command;
    while (1)
    {
      // 从队列接收数据
        if (xQueueReceive(queue, &command, portMAX_DELAY) == pdPASS)
        {
					 // 调试输出接收到的命令
            char buffer[50];
            snprintf(buffer, sizeof(buffer), "Command received: %c\n", command);
            HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
					
            // 用任务锁保护队列的写入操作
            if (osMutexWait(queueMutexHandle, osWaitForever) == osOK)
            {
      // 用条件语句替代 switch 语句
            if (command == 'A') {
                pulse_width_1 += 10;
            } else if (command == 'B') {
                pulse_width_4 += 10;
            } else if (command == 'C') {
                pulse_width_1 -= 10;
            } else if (command == 'D') {
                pulse_width_4 -= 10;
            } else if (command == 'E') {
                pulse_width_1 += 10;
                pulse_width_4 += 10;
            } else if (command == 'F') {
                if (pulse_width_1 > ESC_MIN_PULSE + 10)
                    pulse_width_1 -= 10;
                if (pulse_width_4 > ESC_MIN_PULSE + 10)
                    pulse_width_4 -= 10;
            } else if (command == 'S') {
                pulse_width_1 = ESC_INITIAL_PULSE;
                pulse_width_4 = ESC_INITIAL_PULSE;
            }

               

                // 更新通道1和通道4的脉宽
                __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pulse_width_1);
                __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, pulse_width_4);
								
								 // 调试输出
                char buffer[50];
                snprintf(buffer, sizeof(buffer), "PWM updated: Ch1=%d, Ch4=%d\n", pulse_width_1, pulse_width_4);
                HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);


                // 释放任务锁
                osMutexRelease(queueMutexHandle);
								
//            // 将当前脉宽值发送到串口
//            char buffer[50];
//            int len = snprintf(buffer, sizeof(buffer), "%d %d\r\n", pulse_width_1, pulse_width_4);
//            if (len > 0 && len < sizeof(buffer))
//            {
//                HAL_UART_Transmit(&huart1, (uint8_t *)buffer, len, HAL_MAX_DELAY);
//            }
            }
        }
    }
}
           

     
	
		void StartUartTask(void *argument)
{
    uint8_t rx_data;
    while (1)
    {
        // 接收串口数据
        if (HAL_UART_Receive(&huart1, &rx_data, 1, HAL_MAX_DELAY) == HAL_OK)
        {
            // 将接收到的数据发送到队列
            if (xQueueSend(queue, &rx_data, portMAX_DELAY) == pdPASS)
            {
                // 调试输出
                char buffer[50];
                snprintf(buffer, sizeof(buffer), "UART received: %c\n", rx_data);
                HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
            }
            else
            {
                // 调试输出
                char buffer[50];
                snprintf(buffer, sizeof(buffer), "Failed to send to queue: %c\n", rx_data);
                HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
            }
//					// 将接收到的数据发送到队列
//            xQueueSend(queue, &rx_data, portMAX_DELAY);
        }
    }
}

void StartCanTask(void *argument)
{
    while (1)
    {
        // 等待接收到CAN消息
        if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
        {
            // 处理接收到的CAN消息
            uint8_t command = RxData[0];

            // 调试输出接收到的命令
            char buffer[50];
            snprintf(buffer, sizeof(buffer), "CAN Command received: %c\n", command);
            HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);

            // 用任务锁保护共享资源
            if (osMutexWait(queueMutexHandle, osWaitForever) == osOK)
            {
                if (command == 'H')
                {
                    if (rotor_angle + 10 <= 180) {
                        rotor_angle += 10;
                    }
                }
                else if (command == 'J')
                {
                    if (rotor_angle - 10 >= 0) {
                        rotor_angle -= 10;
                    }
                }
                else
                {
                    // 未识别的命令
                    snprintf(buffer, sizeof(buffer), "Unknown CAN command: %c\n", command);
                    HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
                }

                // 调试输出更新后的角度
                snprintf(buffer, sizeof(buffer), "Rotor angle updated: %d\n", rotor_angle);
                HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);

								
			
                // 释放任务锁
                osMutexRelease(queueMutexHandle);
            }
        }
        // 添加一个适当的延迟
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        static uint8_t received_char;
        HAL_UART_Receive_IT(huart, &received_char, 1); // 设置为接收1个字节

        if (xQueueSendFromISR(queue, &received_char, NULL) == pdPASS)
        {
            // 调试输出
            char buffer[50];
            snprintf(buffer, sizeof(buffer), "ISR received: %c\n", received_char);
            HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
        }
        else
        {
            // 调试输出
            char buffer[50];
            snprintf(buffer, sizeof(buffer), "ISR failed to send to queue: %c\n", received_char);
            HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
        }
//			// 将接收到的数据发送到队列
//        xQueueSendFromISR(queue, &received_char, NULL);
    }
	
		

  /* USER CODE END 2 */

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM3 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM3) {
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
