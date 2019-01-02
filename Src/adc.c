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
/**	@fcn		AdcChannelConfig adc_getChannelConfig(uint32_t channel, GPIO_TypeDef *port, uint16_t pin)
 *  @brief		Get channel configuration for use
 *  @details	x
 *
 *  @param 		[in] (uint32_t) channel - adc channel
 *  @param 		[in] (GPIO_TypeDef *) port - port for adc channel
 *  @param 		[in] (uint16_t) pin - pin for adc channel
 *
 *  @return 	(AdcChannelConfig) selected channel configuration value
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
void adc_init(void) {

	//Generate ADC channel listings for use
	adc_channels[0] = adc_getChannelConfig(ADC_CHANNEL_0, GPIOA,  0);
	adc_channels[1] = adc_getChannelConfig(ADC_CHANNEL_1, GPIOA,  1);
	adc_channels[2] = adc_getChannelConfig(ADC_CHANNEL_13, GPIOC, 3);
	adc_channels[3] = adc_getChannelConfig(ADC_CHANNEL_5, GPIOA,  5);
	adc_channels[4] = adc_getChannelConfig(ADC_CHANNEL_8, GPIOB,  0);
	adc_channels[5] = adc_getChannelConfig(ADC_CHANNEL_9, GPIOB,  1);
	adc_channels[6] = adc_getChannelConfig(ADC_CHANNEL_14, GPIOC, 4);
	adc_channels[7] = adc_getChannelConfig(ADC_CHANNEL_15, GPIOC, 5);

	return;
}


/************************************************************************************************************************************/
/**	@fcn		int main(void)
 *  @brief		x
 *  @details	x
 */
/************************************************************************************************************************************/
void adc_config_init(void) {

	//Locals
	HAL_StatusTypeDef result;

	/* Configuration of AdcHandle init structure: ADC parameters and regular group 													*/
	AdcHandle.Instance = ADC1;

	//Shutdown if existing
	result = HAL_ADC_DeInit(&AdcHandle);

	//Safety
	if(result != HAL_OK) {
		Error_Handler();													/* ADC initialization error 							*/
	}

	//Setup ADC Configuration
	AdcHandle.Init.ClockPrescaler        = ADC_CLOCK_ASYNC_DIV1;
	AdcHandle.Init.Resolution            = ADC_RESOLUTION_12B;
	AdcHandle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
	AdcHandle.Init.ScanConvMode          = ADC_SCAN_DIRECTION_FORWARD;    	/* Sequencer will convert the number of channels configured below, successively from the lowest to the highest channel number */
	AdcHandle.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
	AdcHandle.Init.LowPowerAutoWait      = DISABLE;
	AdcHandle.Init.LowPowerAutoPowerOff  = DISABLE;
	AdcHandle.Init.ContinuousConvMode    = DISABLE;                       	/* Continuous mode disabled to have only 1 rank converted at each conversion trig, and because discontinuous mode is enabled */
	AdcHandle.Init.DiscontinuousConvMode = ENABLE;                        	/* Sequencer of regular group will convert the sequence in several sub-divided sequences */
	AdcHandle.Init.ExternalTrigConv      = ADC_SOFTWARE_START;            	/* Software start to trig the 1st conversion manually, without external event */
	AdcHandle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE; 	/* Parameter discarded because trig of conversion by software start (no external event) */
	AdcHandle.Init.DMAContinuousRequests = ENABLE;                        	/* ADC-DMA continuous requests to match with DMA configured in circular mode */
	AdcHandle.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;
	AdcHandle.Init.SamplingTimeCommon    = ADC_SAMPLETIME_239CYCLES_5;		/* Note: Set long sampling time due to internal channels (VrefInt, temperature sensor) constraints. Refer to device datasheet for min/typ/max values. */

	//Init ADC
	result = HAL_ADC_Init(&AdcHandle);

	//Safety
	if (result != HAL_OK) {
		Error_Handler();													/* ADC initialization error 							*/
	}

	//Init Channels
	adc_config_channel(ADC_CHANNEL_0);
	adc_config_channel(ADC_CHANNEL_1);
	adc_config_channel(ADC_CHANNEL_13);										/* @note 	ADC_IN2 for Xtag							*/
	adc_config_channel(ADC_CHANNEL_5);										/* @note	ADC_IN3 for Xtag							*/
	adc_config_channel(ADC_CHANNEL_8);
	adc_config_channel(ADC_CHANNEL_9);
	adc_config_channel(ADC_CHANNEL_14);
	adc_config_channel(ADC_CHANNEL_15);

	return;
}


/************************************************************************************************************************************/
/**	@fcn		void adc_config_channel(uint32_t channel)
 *  @brief		Initialize the channel config array for use
 *  @details	x
 *
 *  @param 		[in] (uint32_t) channel - ADC channel for use (e.g. 'ADC_CHANNEL_0')
 *
 *  @pre 		adc_init()
 *  @warn		hangs on channel initialization failure
 */
/************************************************************************************************************************************/
void adc_config_channel(uint32_t channel) {

	//Locals
	HAL_StatusTypeDef result;
	ADC_ChannelConfTypeDef sConfig;

	//Config
	sConfig.Channel      = channel;
	sConfig.Rank         = ADC_RANK_CHANNEL_NUMBER;
	sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;						/* what value to use? Is this too long?					*/

	//Apply
	result = HAL_ADC_ConfigChannel(&AdcHandle, &sConfig);

	//Confirm
	if (result != HAL_OK) {
		Error_Handler();													/* Channel Configuration Error 		  					*/
	}

	return;
}


/************************************************************************************************************************************/
/**	@fcn		void adc_gpio_init(uint32_t channel)
 *  @brief		Initialize a GPIO pin selected for ADC channel use
 *  @details	e.g. "PA0" for 'ADC_CHANNEL_0'
 *
 *  @param 		[in] (uint32_t) channel - ADC channel for use (e.g. 'ADC_CHANNEL_0')
 *
 *  @pre 		adc_init()
 *  @warn		hangs on channel initialization failure
 */
/************************************************************************************************************************************/
void adc_gpio_init(uint32_t channel) {

	//Locals
	GPIO_InitTypeDef	config;
	GPIO_TypeDef 	   *port;
	uint16_t 			pin;

	//Lookup Port&Pin
	switch(channel) {
	 case ADC_CHANNEL_0:													/* PA0													*/
		port = GPIOA;
		pin  = 0;
		break;
	 case ADC_CHANNEL_1:													/* PA1													*/
		port = GPIOA;
		pin  = 1;
		break;
	 case ADC_CHANNEL_2:													/* PA2													*/
		port = GPIOA;
		pin  = 2;
		break;
	 case ADC_CHANNEL_3:													/* PA3													*/
		port = GPIOA;
		pin  = 3;
		break;
	 case ADC_CHANNEL_4:													/* PA4													*/
		port = GPIOA;
		pin  = 4;
		break;
	 case ADC_CHANNEL_5:													/* PA5													*/
		port = GPIOA;
		pin  = 5;
		break;
	 case ADC_CHANNEL_6:													/* PA6													*/
		port = GPIOA;
		pin  = 6;
		break;
	 case ADC_CHANNEL_7:													/* PA7													*/
		port = GPIOA;
		pin  = 7;
		break;
	 case ADC_CHANNEL_8:													/* PB0													*/
		port = GPIOB;
		pin  = 0;
		break;
	 case ADC_CHANNEL_9:													/* PB1													*/
		port = GPIOB;
		pin  = 1;
		break;
	 case ADC_CHANNEL_10:													/* PC0													*/
		port = GPIOC;
		pin  = 0;
		break;
	 case ADC_CHANNEL_11:													/* PC1													*/
		port = GPIOC;
		pin  = 1;
		break;
	 case ADC_CHANNEL_12:													/* PC2													*/
		port = GPIOC;
		pin  = 2;
		break;
	 case ADC_CHANNEL_13:													/* PC3													*/
		port = GPIOC;
		pin  = 3;
		break;
	 case ADC_CHANNEL_14:													/* PC4													*/
		port = GPIOC;
		pin  = 4;
		break;
	 case ADC_CHANNEL_15:													/* PC5													*/
		port = GPIOC;
		pin  = 5;
		break;
	 default:																/* safety catch on error input							*/
		for(;;);
	}

	//Config
	config.Pin  = pin;
	config.Mode = GPIO_MODE_ANALOG;
	config.Pull = GPIO_NOPULL;

	//Apply
	HAL_GPIO_Init(port, &config);

	return;
}


/************************************************************************************************************************************/
/**	@fcn		void adc_msp_init(void)
 *  @brief		Initialize the channel config array for use
 *  @details	x
 */
/************************************************************************************************************************************/
void adc_msp_init(void) {

	/* Configure GPIO pin of the selected ADC channel */
	adc_gpio_init(ADC_CHANNEL_0);
	adc_gpio_init(ADC_CHANNEL_1);
	adc_gpio_init(ADC_CHANNEL_13);											/* @note 	ADC_IN2 for Xtag							*/
	adc_gpio_init(ADC_CHANNEL_5);											/* @note 	ADC_IN3 for Xtag							*/
	adc_gpio_init(ADC_CHANNEL_8);
	adc_gpio_init(ADC_CHANNEL_9);
	adc_gpio_init(ADC_CHANNEL_14);
	adc_gpio_init(ADC_CHANNEL_15);

	return;
}

