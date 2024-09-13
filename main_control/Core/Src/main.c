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
#include "hrtim.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "PWM.h"
#include "PID.h"
#include "OLED.h"
#include "OLED_Data.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
PID_DUNC m_pid;
union Vol_r
{
  int v;
  char h_val[4];
};
union Cur_r
{
  int i;
  char h_val[4];
};

union Vol_r vol_r;
union Cur_r cur_r;
extern uint8_t RxData;
extern int Serial_RxFlag;
extern uint8_t Serial_RxPacket[8];
HRTIM_CompareCfgTypeDef pCompareCfg;
uint16_t phaseshift2 = 100;
uint16_t Phase = 100;
float current,voltage;
int PWM_START = 1;
int UVP = 0;
int OVP = 0;
int OCP = 0;
int16_t receive = 0;
uint16_t voltage_rev=0;
uint16_t current_rev=0;
uint16_t START_FLAG = 0;
uint32_t SET_FLAG = 1;
extern int rx_flag;
extern uint8_t RX[2];
extern uint8_t rx_cnt;
extern uint16_t phaseshift1;
int flag = 0;
int key1 = 0;
int key2 = 0;
int key3 = 0;
int key4 = 0;
int System_Start = 0;
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
  MX_HRTIM1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim2);
	PID_Init(&m_pid);
	PWM_Init();
	OLED_Init();
//	OLED_Clear();
//	OLED_Update();
  //__HAL_UART_ENABLE_IT();
	OLED_Clear();
	OLED_ShowString(0,0, "V_OUT",OLED_8X16);
	OLED_ShowString(0,16, "I_OUT",OLED_8X16);
	OLED_ShowString(0,32, "Charge",OLED_8X16);
	OLED_ShowString(0,48, "SET_I",OLED_8X16);
	OLED_ShowChar(40, 0, ':', OLED_8X16);                                                  
  OLED_ShowChar(40, 16, ':', OLED_8X16);                                                 
  OLED_ShowChar(48, 32, ':', OLED_8X16);   
  OLED_ShowChar(40, 48, ':', OLED_8X16); 	
           
						
  OLED_ShowChar(72 + 8 * 6, 0, 'V', OLED_8X16);                                          
  OLED_ShowChar(72 + 8 * 6, 16, 'A', OLED_8X16);
  OLED_ShowChar(72 + 8 * 6, 48, 'A', OLED_8X16);
	OLED_Update();
	
	//phaseshift2 = 900;
	


	HAL_UART_Receive_IT(&huart1, &RxData, 1);

//	while(Serial_GetRxFlag() == 0);
//	HAL_Delay(5000);
	
	HAL_UART_Transmit(&huart1,"AT+RST\r\n",sizeof("AT+RST\r\n"),0xFFFF);
	//USART2_Printf("AT+RST\r\n");
	HAL_Delay(500);
	HAL_UART_Transmit(&huart1,"AT+CWMODE=2\r\n",sizeof("AT+CWMODE=2\r\n"),0xFFFF);
	//USART2_Printf("AT+CWMODE=2\r\n");
	HAL_Delay(1000);
	HAL_UART_Transmit(&huart1,"AT+RST\r\n",sizeof("AT+RST\r\n"),0xFFFF);
	//USART2_Printf("AT+RST\r\n");
	HAL_Delay(500);
	HAL_UART_Transmit(&huart1,"AT+CWSAP=\"ESP-01S\",\"12345678\",6,4\r\n",sizeof("AT+CWSAP=\"ESP-01S\",\"12345678\",6,4\r\n"),0xFFFF);
	//USART2_Printf("AT+CWSAP=\"ESP-01S\",\"12345678\",6,4\r\n");
	HAL_Delay(500);
	HAL_UART_Transmit(&huart1,"AT+CIPMUX=1\r\n",sizeof("AT+CIPMUX=1\r\n"),0xFFFF);
	//USART2_Printf("AT+CIPMUX=1\r\n");
	HAL_Delay(500);
	HAL_UART_Transmit(&huart1,"AT+CIFSR\r\n",sizeof("AT+CIFSR\r\n"),0xFFFF);
	//USART2_Printf("AT+CIFSR\r\n");
	HAL_Delay(500);
	HAL_UART_Transmit(&huart1,"AT+CIPSERVER=1,8080\r\n",sizeof("AT+CIPSERVER=1,8080\r\n"),0xFFFF);
	//USART2_Printf("AT+CIPSERVER=1,8080\r\n");
	HAL_Delay(500);
	
	//HAL_UARTEx_ReceiveToIdle_IT(&huart1, RxData, sizeof(RxData));
	//HAL_UARTEx_ReceiveToIdle_IT(&huart1, &RX, 1);
	
				 
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//    receive++;
//		if(receive==10000)receive=0;
		

		if(Serial_GetRxFlag() == 1)
		{
			vol_r.h_val[0] = Serial_RxPacket[0];
			vol_r.h_val[1] = Serial_RxPacket[1];
			vol_r.h_val[2] = Serial_RxPacket[2];
			vol_r.h_val[3] = Serial_RxPacket[3];
			
			cur_r.h_val[0] = Serial_RxPacket[4];
			cur_r.h_val[1] = Serial_RxPacket[5];
			cur_r.h_val[2] = Serial_RxPacket[6];
			cur_r.h_val[3] = Serial_RxPacket[7];

//			voltage = ((float)vol_r.v/8192.0*3.3)*24.376+0.1171;
//			current = ((float)cur_r.i/8192.0*3.3)*5.375-0.0075;
			 
		}
		voltage = ((float)vol_r.v/8190.0*3.3)*24.666+0.4199;
	  current = ((float)cur_r.i/8190.0*3.3)*5.575+0.1186;
		
		if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_3) == RESET)  //升降平台主控输出低表示当前没有车，关闭原边pwm波
		//if(current < 1) 
		{
			HAL_Delay(20);
			if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_3) == RESET)
			{
				
				m_pid.SetSpeed = 0.5;
				//PWM_START = 0;
				START_FLAG = 0;
				SET_FLAG = 1;
				System_Start = 0;
				Phase = 0;
				phaseshift1 = 0;
			}
		}
		else if(OVP ==0 && OVP == 0 && System_Start == 1)
		{
			phaseshift1 = 100;
			if(PWM_START ==1)
			{	
				Phase = m_pid.OutPut + 100;
			}
			if(PWM_START ==0)
			{
				Phase = 100;
			}
		}
		
		if(voltage > 65) OVP = 1;
		else OVP = 0;
		if(current > 10.5) OCP = 1;
		else OCP = 0;
		
		
                                      
       
						
     OLED_ShowNum(56, 0, voltage , 3, OLED_8X16);                                        
     OLED_ShowChar(56 + 8 * 3, 0, '.', OLED_8X16);                                           
     OLED_ShowNum(56 + 8 * 4, 0, (uint16_t)((voltage ) * 1000) % 1000, 3, OLED_8X16);   
						
     OLED_ShowNum(56, 16, current , 3, OLED_8X16);                                     
     OLED_ShowChar(56 + 8 * 3, 16, '.', OLED_8X16);                                         
     OLED_ShowNum(56 + 8 * 4, 16, (uint16_t)((current ) * 1000) % 1000, 3, OLED_8X16);  

		 OLED_ShowNum(64, 32,(uint16_t)System_Start, 1, OLED_8X16);   
		 
		 
		 OLED_ShowNum(56, 48,(uint16_t)m_pid.SetSpeed, 3, OLED_8X16); 
		 OLED_ShowChar(56 + 8 * 3, 48, '.', OLED_8X16); 
		 OLED_ShowNum(56 + 8 * 4, 48, (uint16_t)((m_pid.SetSpeed ) * 10) % 10, 1, OLED_8X16);
		 OLED_Update();	
//		
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
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)   //1KHz
{
	if (htim->Instance == TIM2)
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);
	
//	if(OVP == 1 || OCP == 1)
//	{
//		Phase = 0;
//		phaseshift1 = 0;
//	}
	if(System_Start == 1) 
	{
		
		phaseshift1 = 100;
		Phase = m_pid.OutPut + 100;
		
		if(START_FLAG <= 3500) START_FLAG++;
		if(SET_FLAG <= 38001) SET_FLAG++;
		
	}
	else if(System_Start == 0)		
	{
		phaseshift1 = 0;
		Phase = 0;
	}
	
	if((START_FLAG == 3500)) PWM_START = 1;
	if(((SET_FLAG)%2000 == 0)) m_pid.SetSpeed += 0.5;
	
	if(OVP == 1 || OCP == 1)
	{
		flag = 1;
		
	}
	if(flag == 1)
	{	
		Phase = 0;
		phaseshift1 = 0;
	}
	else if((HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_3) == SET) && (System_Start == 1))
	{
		phaseshift1 = 100;
		if(PWM_START ==1)
		{	
			PID_realize(&m_pid,current);
			Phase = m_pid.OutPut + 100;
		}
		if(PWM_START ==0)
		{
			Phase = 100;
		}
		
	}
	
	m_pid.ActualSpeed = current;
	
//	if(PWM_START == 0) 
//	{
//		Phase = 100;
//		phaseshift1 = 100;
//	}
//	else if(PWM_START == 1)
//  {	      
//		//__HAL_RCC_HRTIM1_CLK_ENABLE();	
//		PID_realize(&m_pid,current);
//		Phase = m_pid.OutPut +100 ;
//		//Phase = m_pid.OutPut1+100;
//	}
	pCompareCfg.CompareValue = Phase;
	HAL_HRTIM_WaveformCompareConfig(&hhrtim1 , HRTIM_TIMERINDEX_MASTER , HRTIM_COMPAREUNIT_2 , &pCompareCfg);
	pCompareCfg.CompareValue = Phase+5312;
	HAL_HRTIM_WaveformCompareConfig(&hhrtim1 , HRTIM_TIMERINDEX_TIMER_E , HRTIM_COMPAREUNIT_1 , &pCompareCfg);
	
	pCompareCfg.CompareValue = phaseshift1;
	HAL_HRTIM_WaveformCompareConfig(&hhrtim1 , HRTIM_TIMERINDEX_MASTER , HRTIM_COMPAREUNIT_1 , &pCompareCfg);
	pCompareCfg.CompareValue = phaseshift1+5312;
	HAL_HRTIM_WaveformCompareConfig(&hhrtim1 , HRTIM_TIMERINDEX_TIMER_D , HRTIM_COMPAREUNIT_1 , &pCompareCfg);

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_10)
	{
		PWM_START = 1;
//		PWM_START++;
//		if(PWM_START == 2) PWM_START = 0;
//		key1++; 
	}
	else if(GPIO_Pin == GPIO_PIN_11)
	{
		m_pid.SetSpeed -= 0.2;
		key2++;
	}
	else if(GPIO_Pin == GPIO_PIN_12)
	{
			//m_pid.SetSpeed += 0.5;
		m_pid.SetSpeed += 0.2;
		key3++;
	}
	else if(GPIO_Pin == GPIO_PIN_13)
	{
			//m_pid.SetSpeed += 0.5;
		System_Start = 1;
		key4++;
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
