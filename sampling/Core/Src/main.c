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
#include "adc.h"
#include "dma.h"
#include "hrtim.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "OLED.h"
#include "OLED_Data.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
union Vol_s
{
  int v;
  char h_val[4];
};
union Cur_s
{
  int i;
  char h_val[4];
};

union Vol_s vol_s;
union Cur_s cur_s;
uint16_t AD0, AD1, AD2, AD3,SendData;
uint16_t AD_v[31],AD_c[31];
uint16_t temp_v,temp_c;
uint16_t ADv_sum=0,ADc_sum=0,ADv_avg=0,ADc_avg=0;
uint8_t cur_l,cur_h;
uint8_t vol_l,vol_h;
int samp_flag=0;
int t = 0;
int i = 0;
int j=0;
int k=0;
float voltage,current;
float voltage_avg,current_avg;
extern uint8_t Serial_TxPacket[];
volatile uint16_t ADC_DMAresulet[2];
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define ADC_MAX_VALUE 8192.0				
#define REF_3V3 3.3
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
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_HRTIM1_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
	HAL_HRTIM_WaveformCounterStart(&hhrtim1,HRTIM_TIMERID_MASTER);
	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
	HAL_ADC_Start_DMA(&hadc1,(uint32_t*)ADC_DMAresulet,2);
	
	HAL_TIM_Base_Start_IT(&htim2);
	
	HAL_UART_Transmit(&huart1,"AT+RST\r\n",sizeof("AT+RST\r\n"),0xFFFF);
	HAL_Delay(500);
	
	HAL_UART_Transmit(&huart1,"AT+CWMODE=1\r\n",sizeof("AT+CWMODE=1\r\n"),0xFFFF);
	HAL_Delay(1000);
	
	HAL_UART_Transmit(&huart1,"AT+RST\r\n",sizeof("AT+RST\r\n"),0xFFFF);
	HAL_Delay(500);
	
	HAL_UART_Transmit(&huart1,"AT+CWJAP=\"ESP-01S\",\"12345678\"\r\n",sizeof("AT+CWJAP=\"ESP-01S\",\"12345678\"\r\n"),0xFFFF);
	HAL_Delay(5000);
	
	HAL_UART_Transmit(&huart1,"AT+CIFSR\r\n",sizeof("AT+CIFSR\r\n"),0xFFFF);
	HAL_Delay(500);
	
	HAL_UART_Transmit(&huart1,"AT+CIPMUX=0\r\n",sizeof("AT+CIPMUX=0\r\n"),0xFFFF);
	HAL_Delay(500);
	
	HAL_UART_Transmit(&huart1,"AT+CIPSTART=\"TCP\",\"192.168.4.1\",8080\r\n",sizeof("AT+CIPSTART=\"TCP\",\"192.168.4.1\",8080\r\n"),0xFFFF);
	HAL_Delay(4000); 
	
	HAL_UART_Transmit(&huart1,"AT+CIPMODE=1\r\n",sizeof("AT+CIPMODE=1\r\n"),0xFFFF);
	HAL_Delay(500);
	
	HAL_UART_Transmit(&huart1,"AT+CIPSEND\r\n",sizeof("AT+CIPSEND\r\n"),0xFFFF);

	OLED_Init();
	OLED_Clear();
	OLED_ShowString(0,0, "V_OUT",OLED_8X16);
	OLED_ShowString(0,16, "I_OUT",OLED_8X16);
  OLED_ShowChar(40, 0, ':', OLED_8X16);                                                  
  OLED_ShowChar(40, 16, ':', OLED_8X16);  


  OLED_ShowString(0,32, "OVP:",OLED_8X16);
  OLED_ShowString(0,48, "OCP:",OLED_8X16);    
						
  OLED_ShowChar(72 + 8 * 6, 0, 'V', OLED_8X16);                                          
  OLED_ShowChar(72 + 8 * 6, 16, 'A', OLED_8X16);
	OLED_ShowChar(72 + 8 * 6, 32, 'V', OLED_8X16);                                          
  OLED_ShowChar(72 + 8 * 6, 48, 'A', OLED_8X16);
	OLED_Update();
	
//  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if(samp_flag == 1)
		{
			samp_flag = 0;
			
			AD0 = ADC_DMAresulet[0];   //电压采样值*REF_3V3/ADC_MAX_VALUE
			AD1 =  ADC_DMAresulet[1];   //电流采样值*REF_3V3/ADC_MAX_VALUE
			AD_v[i] = AD0;
			AD_c[i] = AD1;
			i++;
		
			if(i == 30)                         // 1k/30
			{
				i = 0;
				for(j=0;j<30;j++)
				{
					for(k=0;k<j;k++)				
					{
						if(AD_v[k] > AD_v[j])
						{
							temp_v = AD_v[k];
							AD_v[k] = AD_v[j];
							AD_v[j] = temp_v;
						}
						if(AD_c[k] > AD_c[j])
						{
							temp_c = AD_c[k];
							AD_c[k] = AD_c[j];
							AD_c[j] = temp_c;
						}
					}
				}
				for(j=13;j<18;j++) 
				{
					ADv_sum += AD_v[j];
					ADc_sum += AD_c[j];
				}
				
				ADv_avg = ADv_sum / 5;
				ADc_avg = ADc_sum / 5;
			
				ADv_sum = 0;
				ADc_sum = 0;
			
				vol_s.v = ADv_avg;
				cur_s.i = ADc_avg;
		
				//电压
				Serial_TxPacket[0] = vol_s.h_val[0];
				Serial_TxPacket[1] = vol_s.h_val[1];
				Serial_TxPacket[2] = vol_s.h_val[2];
				Serial_TxPacket[3] = vol_s.h_val[3];
		
				//电流
				Serial_TxPacket[4] = cur_s.h_val[0];
				Serial_TxPacket[5] = cur_s.h_val[1];
				Serial_TxPacket[6] = cur_s.h_val[2];
				Serial_TxPacket[7] = cur_s.h_val[3];
		
				Serial_SendPacket();


				voltage = ((float)ADv_avg/8190.0*3.3)*24.666+0.4199;
				current = ((float)ADc_avg/8190.0*3.3)*5.575+0.1186;
				
//				voltage = ((float)vol_s.v/8192.0*3.3);
//	      current = ((float)cur_s.i/8192.0*3.3);
				
				
//				OLED_ShowChar(40, 0, ':', OLED_8X16);                                                  
//        OLED_ShowChar(40, 16, ':', OLED_8X16);                                                 
//           
//						
//        OLED_ShowChar(72 + 8 * 6, 0, 'V', OLED_8X16);                                          
//        OLED_ShowChar(72 + 8 * 6, 16, 'A', OLED_8X16);                                         
       
						
     OLED_ShowNum(56, 0, voltage , 3, OLED_8X16);                                        
     OLED_ShowChar(56 + 8 * 3, 0, '.', OLED_8X16);                                           
     OLED_ShowNum(56 + 8 * 4, 0, (uint16_t)((voltage ) * 1000) % 1000, 3, OLED_8X16);   
						
     OLED_ShowNum(56, 16, current , 3, OLED_8X16);                                     
     OLED_ShowChar(56 + 8 * 3, 16, '.', OLED_8X16);                                         
     OLED_ShowNum(56 + 8 * 4, 16, (uint16_t)((current ) * 1000) % 1000, 3, OLED_8X16); 

     OLED_ShowNum(56, 32, 65 , 2, OLED_8X16);
		 OLED_ShowNum(56, 48, 10 , 2, OLED_8X16); OLED_ShowChar(56 + 8 * 2, 48, '.', OLED_8X16);  OLED_ShowNum(56 + 8 * 3, 48, 5 , 1, OLED_8X16); 
				
	   OLED_Update();	

  
			}
	  }
	}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM2)
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);
	samp_flag = 1;
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
