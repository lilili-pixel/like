/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  * Author					: PENGPENG 2024.6.26
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>//自行包含头文件，为了使用prinrf()函数
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint8_t USART3_buff[17]={0};//定义一个接收缓存区
uint8_t USART3_RXbuff[17]={0};//定义一个正真的接收数组
uint8_t rx3_flag = 0;
uint8_t Y_flag = 0;//Y轴运动标志位
uint8_t Z_flag = 0;//Z轴运动标志位
uint8_t motor_flag = 0;

uint16_t AIM_AXIS_X = 0x0160;//X轴的目标坐标
uint16_t AIM_AXIS_Y = 0x0160;//Y轴的目标坐标
uint16_t AIM_AXIS_Z = 0x1800;//Z轴的目标坐标

uint16_t AXIS_X =0;//X轴的实时坐标
uint16_t AXIS_Y =0;//Y轴的实时坐标
uint16_t AXIS_Z =0;//Z轴的实时坐标

int16_t ERROR_X = 0;//X轴的误差
int16_t ERROR_Y = 0;//Y轴的误差
int16_t ERROR_Z = 0;//Z轴的误差

uint8_t z_flag = 0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//回调函数接收相机发送的数据
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if(huart->Instance == USART3)
	{
	
    // 再次开启，否则只会接收一次
		HAL_UARTEx_ReceiveToIdle_IT(huart, USART3_buff, sizeof(USART3_buff));

		rx3_flag = 1;
	}
}


//限位开关触发
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_4)
	{
		z_flag = 1;
	}
}


 
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
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */
  HAL_UARTEx_ReceiveToIdle_IT(&huart3, USART3_buff, sizeof(USART3_buff));//接收数据
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

		if(rx3_flag == 1)
		{
			//获取X、Y、Z轴的实时坐标
			AXIS_X = (uint16_t)(USART3_buff[3] << 8) | USART3_buff[4];
			AXIS_Y = (uint16_t)(USART3_buff[5] << 8) | USART3_buff[6];
			AXIS_Z = (uint16_t)(USART3_buff[9] << 8) | USART3_buff[10];
			
			//计算误差坐标
			ERROR_X = AIM_AXIS_X - AXIS_X;
			ERROR_Y = AIM_AXIS_Y - AXIS_Y;
			ERROR_Z = AIM_AXIS_Z - AXIS_Z;
			
			//输出给原边主控信号
			if((AXIS_X == 252) &&(AXIS_Y == 5144) && (AXIS_Z == 0)) HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_RESET);
			else HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_SET);
			
			//X轴控制
			//if(ERROR_X != 0x0000 /*&& AXIS_X != 252*/)
			if(AXIS_X != 252)
			{			
				if(ERROR_X < 0x0000 || ERROR_X > 0x0010)
				{
					if(ERROR_X < 0)
					{
						HAL_GPIO_WritePin(GPIOC,GPIO_PIN_10,GPIO_PIN_RESET);//X电机反转
						HAL_GPIO_WritePin(GPIOC,GPIO_PIN_11,GPIO_PIN_RESET);//Y电机反转
						HAL_TIM_PWM_Start_IT(&htim8,TIM_CHANNEL_1);
						HAL_TIM_PWM_Start_IT(&htim8,TIM_CHANNEL_2);
					}
					else if(ERROR_X > 0x0010)
					{
						HAL_GPIO_WritePin(GPIOC,GPIO_PIN_10,GPIO_PIN_SET);//X电机正转
						HAL_GPIO_WritePin(GPIOC,GPIO_PIN_11,GPIO_PIN_SET);//Y电机正转
						HAL_TIM_PWM_Start_IT(&htim8,TIM_CHANNEL_1);
						HAL_TIM_PWM_Start_IT(&htim8,TIM_CHANNEL_2);     
					}
				}
				else if(0x0000 < ERROR_X < 0x0010)
				{
					Y_flag = 1;
					HAL_TIM_PWM_Stop_IT(&htim8,TIM_CHANNEL_1);
					HAL_TIM_PWM_Stop_IT(&htim8,TIM_CHANNEL_2);
				}
			}
			else if(AXIS_X == 252)
			{
				HAL_TIM_PWM_Stop_IT(&htim8,TIM_CHANNEL_1);
				HAL_TIM_PWM_Stop_IT(&htim8,TIM_CHANNEL_2);
				Y_flag = 1;		
			}
			
			//Y轴控制
			if(Y_flag == 1)
			{
				if(AXIS_Y != 5144)
				{				
					if(ERROR_Y < 0x0000 || ERROR_Y > 0x0010)
					{
						if(ERROR_Y < 0)
						{
							HAL_GPIO_WritePin(GPIOC,GPIO_PIN_10,GPIO_PIN_RESET);//X电机正转
							HAL_GPIO_WritePin(GPIOC,GPIO_PIN_11,GPIO_PIN_SET);  //Y电机反转
							HAL_TIM_PWM_Start_IT(&htim8,TIM_CHANNEL_1);
							HAL_TIM_PWM_Start_IT(&htim8,TIM_CHANNEL_2);
						}
						else if(ERROR_Y > 0x0010)
						{
							HAL_GPIO_WritePin(GPIOC,GPIO_PIN_10,GPIO_PIN_SET);  //X电机反转
							HAL_GPIO_WritePin(GPIOC,GPIO_PIN_11,GPIO_PIN_RESET);//Y电机正转
							HAL_TIM_PWM_Start_IT(&htim8,TIM_CHANNEL_1);
							HAL_TIM_PWM_Start_IT(&htim8,TIM_CHANNEL_2);
						}
					}
					else if(0x0000 < ERROR_Y < 0x0010)
					{
						Y_flag = 0;
						Z_flag = 1;
						HAL_TIM_PWM_Stop_IT(&htim8,TIM_CHANNEL_1);
						HAL_TIM_PWM_Stop_IT(&htim8,TIM_CHANNEL_2);
					}
				}
				else if(AXIS_Y == 5144)
				{
					Y_flag = 0;
					Z_flag = 1;			
				}
			}
			
			//Z轴控制
			if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_4) == RESET) 
			{
				HAL_Delay(30);
				if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_4) == RESET)  z_flag = 1;
			}
			if(Z_flag == 1)
			{
				if(AXIS_Z != 0)
				{
					z_flag = 0;
					if(ERROR_Z < 0x0000 || ERROR_Z > 0x0100)
					{
						if(ERROR_Z < 0)
						{
							HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET);//Z电机反转
							HAL_TIM_PWM_Start_IT(&htim1,TIM_CHANNEL_1);
						}
						else if(ERROR_Z > 0x0100)
						{
							HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_RESET);//Z电机正转
							HAL_TIM_PWM_Start_IT(&htim1,TIM_CHANNEL_1);
						}
					}
					else if( 0x0000 < ERROR_Z < 0x0100)
					{
						Z_flag = 0;
						HAL_TIM_PWM_Stop_IT(&htim1,TIM_CHANNEL_1);
					}
				}
				else if(AXIS_Z == 0)
				{
					if(z_flag == 1)
					HAL_TIM_PWM_Stop_IT(&htim1,TIM_CHANNEL_1);
					else
					{					
						HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET);//Z电机反转
						HAL_TIM_PWM_Start_IT(&htim1,TIM_CHANNEL_1);
					}
				}
			}
			
			
			//使用串口1打印出实时接受的数据，波特率38400
		  HAL_UART_Transmit(&huart1, (uint8_t *)USART3_buff, sizeof(USART3_buff), 100);
			HAL_Delay(100);
			
			rx3_flag = 0;//标志位清零，防止陷入后台
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV6;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
