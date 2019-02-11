/**
  ******************************************************************************
  * @file    GPIO/GPIO_IOToggle/Src/main.c
  * @author  MCD Application Team
  * @version V1.6.0
  * @date    27-May-2016
  * @brief   This example describes how to configure and use GPIOs through
  *          the STM32F0xx HAL API.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal_adc.h"
#include "stm32f0xx_hal_adc_ex.h"

/* Private typedef -----------------------------------------------------------------------------------------------------------------*/
#define ADC_NUM_CHANNELS 	(5)    											/* number of adc channels under adc sequencer sequence	*/
#define _nop()				asm(" nop")										/* nop instruction using standard convention			*/


/* Private variables ---------------------------------------------------------------------------------------------------------------*/

//ADC
ADC_HandleTypeDef AdcHandle;												/* ADC handler declaration 								*/
uint16_t adc_vals[ADC_NUM_CHANNELS];										/* Variable containing ADC conversions results 			*/

//General
HAL_StatusTypeDef result;													/* used for HAL operation results review				*/
uint8_t sample_num;															/* active ADC sample in progress						*/


/* Private function prototypes -----------------------------------------------------------------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);

void gpio_led2_init(void);
void ADC_Config(void);
void _check(HAL_StatusTypeDef result);

/* Private functions ---------------------------------------------------------------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  *
  * @note 	Uses simple delay for sample completion, consider use of polling for improved design
  */
int main(void) {

	//Init Vars
	sample_num = 0;

	//Boot
	HAL_Init();
	SystemClock_Config();													/* Configure the system clock to 48 MHz 				*/
	gpio_led2_init();														/* enable led2 gpio 									*/
	ADC_Config();															/* initialize the ADC 									*/

	result = HAL_ADCEx_Calibration_Start(&AdcHandle);						/* Run the ADC calibration 								*/
	_check(result);

	//Boot Delay
	HAL_Delay(100);															/* periodic instability observed w/o boot delay			*/


	//******************************************************************************************************************************//
	//														START DMA SEQUENCE														//
	// @brief 	Start ADC conversion on regular group with transfer by DMA 															//
	//******************************************************************************************************************************//
	result = HAL_ADC_Start_DMA(&AdcHandle, (uint32_t *)adc_vals, ADC_NUM_CHANNELS);
	_check(result);

	//Sample ADC
	for(;;) {

		//**************************************************************************************************************************//
		//													SAMPLE NEXT CHANNEL														//
		//**************************************************************************************************************************//

		//Read ADC
		result = HAL_ADC_Start(&AdcHandle);
		_check(result);

		//Update Count
		sample_num = (sample_num+1)%ADC_NUM_CHANNELS;

		//Wait for Completion of Sample
		HAL_Delay(1);														/* Wait for conversion completion 						*/

		if(!sample_num) {
			_nop();															/* User breakpoint loc									*/
		}
	}
}


/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSI48)
  *            SYSCLK(Hz)                     = 48000000
  *            HCLK(Hz)                       = 48000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            HSI Frequency(Hz)              = 48000000
  *            PREDIV                         = 2
  *            PLLMUL                         = 2
  *            Flash Latency(WS)              = 1
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void) {

  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  
  /* Select HSI48 Oscillator as PLL source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI48;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;

  result = HAL_RCC_OscConfig(&RCC_OscInitStruct);
  _check(result);

  /* Select PLL as system clock source and configure the HCLK and PCLK1 clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  result = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);
  _check(result);
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void) {
  for(;;);																	/* User may add here some code to deal with this error	*/
}


/**
  * @fcn 	void gpio_led2_init(void)
  * @brief  Initialize the LED2 GPIO pin
  * @param  None
  * @retval None
  */
void gpio_led2_init(void) {

	//Locals
	GPIO_InitTypeDef  GPIO_InitStruct;

	//Init Clocks
	LED2_GPIO_CLK_ENABLE();													/* Enable each GPIO Clock 								*/

	//Configure IO
	GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull  = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

	//Init
	GPIO_InitStruct.Pin = LED2_PIN;
	HAL_GPIO_Init(LED2_GPIO_PORT, &GPIO_InitStruct);

	return;
}


/**
  * @brief  ADC configuration
  * @param  None
  * @retval None
  */
void ADC_Config(void) {

	//Locals
	ADC_ChannelConfTypeDef   sConfig;

	/* Configuration of AdcHandle init structure: ADC parameters and regular group */
	AdcHandle.Instance = ADCx;

	result = HAL_ADC_DeInit(&AdcHandle);
	_check(result);

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
	/* Note: Set long sampling time due to internal channels (VrefInt, temperature sensor) constraints. Refer to device datasheet	*/
	/* 		 for min/typ/max values.                                                												*/
	AdcHandle.Init.SamplingTimeCommon    = ADC_SAMPLETIME_239CYCLES_5;

	result = HAL_ADC_Init(&AdcHandle);
	_check(result);

	/* Configuration of channel on ADCx regular group on sequencer rank 1 */
	/* Note: Considering IT occurring after each ADC conversion (IT by DMA end of transfer), select sampling time and ADC clock		*/
	/* 		 with sufficient duration to not create an overhead situation in IRQHandler.        									*/
	sConfig.Channel      = ADCx_CHANNELa;
	sConfig.Rank         = ADC_RANK_CHANNEL_NUMBER;

	result = HAL_ADC_ConfigChannel(&AdcHandle, &sConfig);
	_check(result);

	/* Configuration of channel on ADCx regular group on sequencer rank 2 Replicate previous rank settings, change only channel 	*/
	/* Note: On STM32F0xx, rank is defined by channel number. ADC Channel ADC_CHANNEL_TEMPSENSOR is on ADC channel 16, there is 1 	*/
	/*		 other channel enabled with lower channel number. Therefore, ADC_CHANNEL_TEMPSENSOR will be converted by the sequencer	*/
	/*		 as the 2nd rank.                                                          												*/
	sConfig.Channel      = ADCx_CHANNELb;

	result = HAL_ADC_ConfigChannel(&AdcHandle, &sConfig);
	_check(result);

	//Channel 3
	sConfig.Channel      = ADCx_CHANNELc;
	result = HAL_ADC_ConfigChannel(&AdcHandle, &sConfig);
	_check(result);

	//Channel 4
	sConfig.Channel      = ADCx_CHANNELd;
	result = HAL_ADC_ConfigChannel(&AdcHandle, &sConfig);
	_check(result);

	//Channel 4
	sConfig.Channel      = ADCx_CHANNELe;
	result = HAL_ADC_ConfigChannel(&AdcHandle, &sConfig);
	_check(result);

	return;
}


/**
  * @brief  Check for HAL errors
  * @param  None
  * @retval None
  * @warn 	Spins on !HAL_OK
  */
void _check(HAL_StatusTypeDef result) {

	//Check result
	if (result != HAL_OK) {
		Error_Handler();    												/* Calibration Error 									*/
	}

	return;
}
