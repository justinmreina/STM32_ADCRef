/************************************************************************************************************************************/
/** @file		adc.c
 * 	@brief		HAL ADC Interface
 * 	@details	x
 *
 * 	@section	Opens
 * 			none current
 *
 * 	@section	Legal Disclaimer
 * 			2018© Ergsense LLC, All rights reserved. All contents of this source file and/or any other related source
 *			files are the explicit property of Ergsense LLC. Do not distribute. Do not copy.
 */
/************************************************************************************************************************************/
#include "adc.h"

//ADC Config
AdcChannelConfig adc_channels[NUM_ADC_CHANNELS];


/************************************************************************************************************************************/
/**	@fcn		int main(void)
 *  @brief		x
 *  @details	x
 */
/************************************************************************************************************************************/
AdcChannelConfig adc_getChannelConfig(uint32_t channel, GPIO_TypeDef *port, uint16_t pin) {

	//Locals
	AdcChannelConfig config;

	//Populate
	config.channel = channel;
	config.port    = port;
	config.pin     = pin;

	return config;
}


/************************************************************************************************************************************/
/**	@fcn		int main(void)
 *  @brief		x
 *  @details	x
 */
/************************************************************************************************************************************/
void adc_data_init(void) {



	return;
}

/************************************************************************************************************************************/
/**	@fcn		int main(void)
 *  @brief		x
 *  @details	x
 */
/************************************************************************************************************************************/
void adc_init(void) {


	return;
}


/************************************************************************************************************************************/
/**	@fcn		int main(void)
 *  @brief		x
 *  @details	x
 */
/************************************************************************************************************************************/
void adc_config_init(void) {
	return;
}


/************************************************************************************************************************************/
/**	@fcn		int main(void)
 *  @brief		x
 *  @details	x
 */
/************************************************************************************************************************************/
void adc_msp_init(void) {
	return;
}



/************************************************************************************************************************************/
/**	@fcn		void ADC_Config(void)
 *  @brief		ADC configuration
 *  @details	x
 */
/************************************************************************************************************************************/
void ADC_Config(void) {
  HAL_StatusTypeDef result;
  ADC_ChannelConfTypeDef   sConfig;

  /* Configuration of AdcHandle init structure: ADC parameters and regular group */
  AdcHandle.Instance = ADC1;

  if (HAL_ADC_DeInit(&AdcHandle) != HAL_OK)
  {
    /* ADC initialization error */
    Error_Handler();
  }

  AdcHandle.Init.ClockPrescaler        = ADC_CLOCK_ASYNC_DIV1;
  AdcHandle.Init.Resolution            = ADC_RESOLUTION_12B;
  AdcHandle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
  AdcHandle.Init.ScanConvMode          = ADC_SCAN_DIRECTION_FORWARD;    /* Sequencer will convert the number of channels configured below, successively from the lowest to the highest channel number */
  AdcHandle.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
  AdcHandle.Init.LowPowerAutoWait      = DISABLE;
  AdcHandle.Init.LowPowerAutoPowerOff  = DISABLE;
  AdcHandle.Init.ContinuousConvMode    = DISABLE;                       /* Continuous mode disabled to have only 1 rank converted at each conversion trig, and because discontinuous mode is enabled */
  AdcHandle.Init.DiscontinuousConvMode = ENABLE;                        /* Sequencer of regular group will convert the sequence in several sub-divided sequences */
  AdcHandle.Init.ExternalTrigConv      = ADC_SOFTWARE_START;            /* Software start to trig the 1st conversion manually, without external event */
  AdcHandle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE; /* Parameter discarded because trig of conversion by software start (no external event) */
  AdcHandle.Init.DMAContinuousRequests = ENABLE;                        /* ADC-DMA continuous requests to match with DMA configured in circular mode */
  AdcHandle.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;
  /* Note: Set long sampling time due to internal channels (VrefInt,          */
  /*       temperature sensor) constraints. Refer to device datasheet for     */
  /*       min/typ/max values.                                                */
  AdcHandle.Init.SamplingTimeCommon    = ADC_SAMPLETIME_239CYCLES_5;

  if (HAL_ADC_Init(&AdcHandle) != HAL_OK)
  {
    /* ADC initialization error */
    Error_Handler();
  }

  /* Configuration of channel on ADCx regular group on sequencer rank 1 */
  /* Note: Considering IT occurring after each ADC conversion (IT by DMA end  */
  /*       of transfer), select sampling time and ADC clock with sufficient   */
  /*       duration to not create an overhead situation in IRQHandler.        */
  sConfig.Channel      = ADC_CHANNEL_0;
  sConfig.Rank         = ADC_RANK_CHANNEL_NUMBER;

  if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK)
  {
    /* Channel Configuration Error */
    Error_Handler();
  }

  /* Configuration of channel on ADCx regular group on sequencer rank 2 */
  /* Replicate previous rank settings, change only channel */
  /* Note: On STM32F0xx, rank is defined by channel number. ADC Channel       */
  /*       ADC_CHANNEL_TEMPSENSOR is on ADC channel 16, there is 1 other      */
  /*       channel enabled with lower channel number. Therefore,              */
  /*       ADC_CHANNEL_TEMPSENSOR will be converted by the sequencer as the   */
  /*       2nd rank.                                                          */
  sConfig.Channel      = ADC_CHANNEL_1;

  if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK)
  {
    /* Channel Configuration Error */
    Error_Handler();
  }

  /* Configuration of channel on ADCx regular group on sequencer rank 3 */
  /* Replicate previous rank settings, change only channel */
  /* Note: On STM32F0xx, rank is defined by channel number. ADC Channel       */
  /*       ADC_CHANNEL_VREFINT is on ADC channel 17, there is are 2 other     */
  /*       channels enabled with lower channel number. Therefore,             */
  /*       ADC_CHANNEL_VREFINT will be converted by the sequencer as the      */
  /*       3rd rank.                                                          */
  sConfig.Channel      = ADC_CHANNEL_13;
  result = HAL_ADC_ConfigChannel(&AdcHandle, &sConfig);
  if (result != HAL_OK) {
    Error_Handler();					/* Channel Configuration Error 		  */
  }

  //ADC_IN5
  sConfig.Channel      = ADC_CHANNEL_5;
  if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK) {
    Error_Handler();					/* Channel Configuration Error 		  */
  }

  //ADC_IN6
  sConfig.Channel      = ADC_CHANNEL_6;
  if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK) {
    Error_Handler();					/* Channel Configuration Error 		  */
  }

  //ADC_IN7
  sConfig.Channel      = ADC_CHANNEL_7;
  if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK) {
    Error_Handler();					/* Channel Configuration Error 		  */
  }

  //ADC_IN8
  sConfig.Channel      = ADC_CHANNEL_8;
  if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK) {
    Error_Handler();					/* Channel Configuration Error 		  */
  }

  //ADC_IN9
  sConfig.Channel      = ADC_CHANNEL_9;
  if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK) {
    Error_Handler();					/* Channel Configuration Error 		  */
  }

  //ADC_IN14
  sConfig.Channel      = ADC_CHANNEL_14;
  if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK) {
    Error_Handler();					/* Channel Configuration Error 		  */
  }

  //ADC_IN15
  sConfig.Channel      = ADC_CHANNEL_15;
  if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK) {
    Error_Handler();					/* Channel Configuration Error 		  */
  }

  return;
}

