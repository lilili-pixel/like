#include "stm32g4xx_hal.h"
#include "hrtim.h"
#include "PWM.h"

uint16_t phaseshift1 = 100;

void PWM_Init()
{
	HAL_HRTIM_WaveformCounterStart(&hhrtim1,HRTIM_TIMERID_MASTER | HRTIM_TIMERID_TIMER_D | HRTIM_TIMERID_TIMER_E);
	HAL_HRTIM_WaveformOutputStart(&hhrtim1,HRTIM_OUTPUT_TD1);
	HAL_HRTIM_WaveformOutputStart(&hhrtim1,HRTIM_OUTPUT_TD2);
	HAL_HRTIM_WaveformOutputStart(&hhrtim1,HRTIM_OUTPUT_TE1);
	HAL_HRTIM_WaveformOutputStart(&hhrtim1,HRTIM_OUTPUT_TE2);
  HRTIM_CompareCfgTypeDef pCompareCfg = {0};
	
	
	pCompareCfg.CompareValue = phaseshift1;
	HAL_HRTIM_WaveformCompareConfig(&hhrtim1 , HRTIM_TIMERINDEX_MASTER , HRTIM_COMPAREUNIT_1 , &pCompareCfg);
	pCompareCfg.CompareValue = phaseshift1+5312;
	HAL_HRTIM_WaveformCompareConfig(&hhrtim1 , HRTIM_TIMERINDEX_TIMER_D , HRTIM_COMPAREUNIT_1 , &pCompareCfg);
}

