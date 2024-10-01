/*
 * eTechRacingADC.c
 *
 *  Created on: May 24, 2024
 *      Author: Andreu
 */

#include "eTechRacingADC.h"
#include "stdlib.h"

uint16_t ADC_Measure(ADC_HandleTypeDef hadc, uint8_t CH)
{
	uint16_t Measure;
	ADC_ChannelConfTypeDef sConfig = {0};
	  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	  */
	switch (CH){
	case 0:
	  sConfig.Channel = ADC_CHANNEL_0;
	  sConfig.Rank = 1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
	  break;
	case 1:
	  sConfig.Channel = ADC_CHANNEL_1;
	  sConfig.Rank = 1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;

	  break;
	case 2:
	  sConfig.Channel = ADC_CHANNEL_2;
	  sConfig.Rank = 1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;

	  break;
		case 3:
	  sConfig.Channel = ADC_CHANNEL_3;
	  sConfig.Rank = 1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;

	  break;
		case 4:
	  sConfig.Channel = ADC_CHANNEL_4;
	  sConfig.Rank = 1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
	  break;
		case 5:
	  sConfig.Channel = ADC_CHANNEL_5;
	  sConfig.Rank = 1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
	  break;
		case 6:
	  sConfig.Channel = ADC_CHANNEL_6;
	  sConfig.Rank = 1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
	  break;
		case 7:
	  sConfig.Channel = ADC_CHANNEL_7;
	  sConfig.Rank = 1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
	  break;
		case 8:
	  sConfig.Channel = ADC_CHANNEL_8;
	  sConfig.Rank = 1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
	  break;
		case 9:
	  sConfig.Channel = ADC_CHANNEL_9;
	  sConfig.Rank = 1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
	  break;
		case 10:
	  sConfig.Channel = ADC_CHANNEL_10;
	  sConfig.Rank = 1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
	  break;
		case 11:
	  sConfig.Channel = ADC_CHANNEL_11;
	  sConfig.Rank = 1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
	  break;
		case 12:
	  sConfig.Channel = ADC_CHANNEL_12;
	  sConfig.Rank = 1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
	  break;
		case 13:
		  sConfig.Channel = ADC_CHANNEL_13;
		  sConfig.Rank = 1;
		  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
		  break;
	 case 14:
		  sConfig.Channel = ADC_CHANNEL_14;
		  sConfig.Rank = 1;
		  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
		  break;
	 case 15:
		  sConfig.Channel = ADC_CHANNEL_15;
		  sConfig.Rank = 1;
		  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
		  break;
		}

	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
		  {
		    Error_Handler();

		  }
	  	  HAL_ADC_Start(&hadc);
		  HAL_ADC_PollForConversion(&hadc, 1000);
		  Measure = HAL_ADC_GetValue(&hadc);
		  return Measure;

}


