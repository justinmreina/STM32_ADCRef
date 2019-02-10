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

/** @addtogroup STM32F0xx_HAL_Examples
  * @{
  */

/** @addtogroup GPIO_IOToggle
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* ADC parameters */
#define ADCCONVERTEDVALUES_BUFFER_SIZE ((uint32_t)    5)    /* Size of array containing ADC converted values: set to ADC sequencer number of ranks converted, to have a rank in each address */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static GPIO_InitTypeDef  GPIO_InitStruct;

//ADC
ADC_HandleTypeDef AdcHandle;												/* ADC handler declaration 								*/
volatile uint16_t aADCxConvertedValues[ADCCONVERTEDVALUES_BUFFER_SIZE];		/* Variable containing ADC conversions results 			*/


/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);

void gpio_led2_init(void);
void ADC_Config(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void) {

  /* This sample code shows how to use GPIO HAL API to toggle LED2 IOs
    in an infinite loop. */

  /* STM32F0xx HAL library initialization:
       - Configure the Flash prefetch
       - Systick timer is configured by default as source of time base, but user 
         can eventually implement his proper time base source (a general purpose 
         timer for example or other time source), keeping in mind that Time base 
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
         handled in milliseconds basis.
       - Low Level Initialization
     */
  HAL_Init();

  /* Configure the system clock to 48 MHz */
  SystemClock_Config();

  /* enable led2 gpio */
  gpio_led2_init();

  /* initialize the ADC */
  ADC_Config();

	/* Run the ADC calibration */
	if (HAL_ADCEx_Calibration_Start(&AdcHandle) != HAL_OK) {
		Error_Handler();    												/* Calibration Error 									*/
	}

	/*## Enable peripherals ####################################################*/
	/*## Start ADC conversions #################################################*/

	/* Start ADC conversion on regular group with transfer by DMA */
	if (HAL_ADC_Start_DMA(&AdcHandle, (uint32_t *)aADCxConvertedValues, ADCCONVERTEDVALUES_BUFFER_SIZE) != HAL_OK) {
		Error_Handler();													/* Start Error 											*/
	}

	/* -3- Toggle IOs in an infinite loop */
	for(;;) {

		//READ ADC
		if (HAL_ADC_Start(&AdcHandle) != HAL_OK) {
			Error_Handler();
		}

		/* Wait for conversion completion */
		/* Note: A fixed wait time of 1ms is used for the purpose of this example: ADC conversions are decomposed between each rank
		 *  	 of the ADC sequencer. Function "HAL_ADC_PollForConversion(&AdcHandle, 1)" could be used, instead of wait time, but
		 *  	 with a different configuration (this function cannot be used if ADC configured in DMA mode and polling for end of
		 *  	 each conversion): a possible configuration is ADC polling for the entire sequence (ADC init parameter "EOCSelection"
		 *  	 set to ADC_EOC_SEQ_CONV) (this also induces that ADC discontinuous mode must be disabled).           				*/
		HAL_Delay(1);

    //Loop Delay
	HAL_GPIO_TogglePin(LED2_GPIO_PORT, LED2_PIN);
    HAL_Delay(100);															/* Insert delay 100 ms 									*/
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
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  
  /* Select HSI48 Oscillator as PLL source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI48;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct)!= HAL_OK)
  {
    Error_Handler();
  }

  /* Select PLL as system clock source and configure the HCLK and PCLK1 clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1)!= HAL_OK)
  {
    Error_Handler();
  }
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* User may add here some code to deal with this error */
  while(1)
  {
  }
}


/**
  * @fcn 	void gpio_led2_init(void)
  * @brief  Initialize the LED2 GPIO pin
  * @param  None
  * @retval None
  */
void gpio_led2_init(void) {

	/* -1- Enable each GPIO Clock (to be able to program the configuration registers) */
	LED2_GPIO_CLK_ENABLE();

	/* -2- Configure IOs in output push-pull mode to drive external LEDs */
	GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull  = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

	GPIO_InitStruct.Pin = LED2_PIN;
	HAL_GPIO_Init(LED2_GPIO_PORT, &GPIO_InitStruct);

	return;
}


/**
  * @brief  ADC configuration
  * @param  None
  * @retval None
  */
void ADC_Config(void)
{
  ADC_ChannelConfTypeDef   sConfig;

  /* Configuration of AdcHandle init structure: ADC parameters and regular group */
  AdcHandle.Instance = ADCx;

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
  sConfig.Channel      = ADCx_CHANNELa;
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
  sConfig.Channel      = ADCx_CHANNELb;

  if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK)
  {
    /* Channel Configuration Error */
    Error_Handler();
  }

  //Channel 3
  sConfig.Channel      = ADCx_CHANNELc;
  if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK) {
    Error_Handler();									/* Channel Configuration Error 				*/
  }

  //Channel 4
  sConfig.Channel      = ADCx_CHANNELd;
  if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK) {
    Error_Handler();									/* Channel Configuration Error 				*/
  }

  //Channel 4
  sConfig.Channel      = ADCx_CHANNELe;
  if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK) {
    Error_Handler();									/* Channel Configuration Error 				*/
  }

	return;
}

