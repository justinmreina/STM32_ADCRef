/************************************************************************************************************************************/
/** @file		adc.h
 * 	@brief		HAL ADC Interface
 * 	@details	x
 *
 * 	@section	Opens
 * 			none current
 *
 * 	@section	Legal Disclaimer
 * 			2018� Ergsense LLC, All rights reserved. All contents of this source file and/or any other related source
 *			files are the explicit property of Ergsense LLC. Do not distribute. Do not copy.
 */
/************************************************************************************************************************************/
#ifndef __ADC_H
#define __ADC_H

//Library
#include <stdint.h>

//Toolchain
#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_adc.h"
#include "stm32f0xx_hal_adc_ex.h"

//Project
#include "main.h"


//Definitions
#define NUM_ADC_CHANNELS	(8)


//Typedefs
typedef struct adcChannelConfig {
	uint32_t       channel;
	GPIO_TypeDef *port;
	uint16_t      pin;
} AdcChannelConfig;


//Globals
extern void adc_config_init(void);
extern void adc_msp_init(void);

//Locals
void adc_config_channel(uint32_t channel);

#endif /* __ADC_H */

