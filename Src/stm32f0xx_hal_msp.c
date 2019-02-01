/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : stm32f0xx_hal_msp.c
  * Description        : This file provides code for the MSP Initialization 
  *                      and de-Initialization codes.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN Define */
 
/* USER CODE END Define */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN Macro */

/* USER CODE END Macro */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* External functions --------------------------------------------------------*/
/* USER CODE BEGIN ExternalFunctions */

/* USER CODE END ExternalFunctions */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
/**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{
  /* USER CODE BEGIN MspInit 0 */

  /* USER CODE END MspInit 0 */

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();

  /* System interrupt init*/

  /* USER CODE BEGIN MspInit 1 */

  /* USER CODE END MspInit 1 */
}

/**
* @brief ADC MSP Initialization
* This function configures the hardware resources used in this example
* @param hadc: ADC handle pointer
* @retval None
*/
void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc) {

	//Locals
	AdcChannelConfig config1, config2;										/* ADC Channel to use for demo							*/
	GPIO_InitTypeDef GPIO_InitStruct;
	static DMA_HandleTypeDef  DmaHandle;
	RCC_OscInitTypeDef        RCC_OscInitStructure;


	//Init
	memset(&GPIO_InitStruct, 0, sizeof(GPIO_InitStruct));
	config1 = adc_getChannelConfig(adc_channels[0]);
	config2 = adc_getChannelConfig(adc_channels[1]);

	if(hadc->Instance==ADC1) {

		/* Peripheral clock enable */
		__HAL_RCC_ADC1_CLK_ENABLE();
		__HAL_RCC_GPIOA_CLK_ENABLE();											/* @todo 	all gpio clocks for adc						*/

		/* Note: In case of usage of asynchronous clock derived from ADC dedicated  */
		/*       HSI RC oscillator 14MHz, with ADC setting                          */
		/*       "AdcHandle.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1",            */
		/*       the clock source has to be enabled at RCC top level using function */
		/*       "HAL_RCC_OscConfig()" (see comments in stm32f0_hal_adc.c header)   */

		/* Enable asynchronous clock source of ADCx */
		/* (place oscillator HSI14 under control of the ADC) */
		HAL_RCC_GetOscConfig(&RCC_OscInitStructure);
		RCC_OscInitStructure.OscillatorType = RCC_OSCILLATORTYPE_HSI14;
		RCC_OscInitStructure.HSI14CalibrationValue = RCC_HSI14CALIBRATION_DEFAULT;
		RCC_OscInitStructure.HSI14State = RCC_HSI14_ADC_CONTROL;
		HAL_RCC_OscConfig(&RCC_OscInitStructure);

		/* Enable clock of DMA associated to the peripheral */
		__HAL_RCC_DMA1_CLK_ENABLE();


		/**ADC GPIO Configuration
		PA0     ------> ADC_IN0
		VREF    ------> No GPIO
		*/
		GPIO_InitStruct.Pin = config1.gpio_pin;								/* GPIO_PIN_0											*/
		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(config1.port, &GPIO_InitStruct);

		GPIO_InitStruct.Pin = config2.gpio_pin;								/* GPIO_PIN_1											*/
		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(config2.port, &GPIO_InitStruct);


		/*##-3- Configure the DMA ##################################################*/
		/* Configure DMA parameters */
		DmaHandle.Instance = DMA1_Channel1;

		DmaHandle.Init.Direction           = DMA_PERIPH_TO_MEMORY;
		DmaHandle.Init.PeriphInc           = DMA_PINC_DISABLE;
		DmaHandle.Init.MemInc              = DMA_MINC_ENABLE;
		DmaHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;   /* Transfer from ADC by half-word to match with ADC configuration: ADC resolution 10 or 12 bits */
		DmaHandle.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;   /* Transfer to memory by half-word to match with buffer variable type: half-word */
		DmaHandle.Init.Mode                = DMA_CIRCULAR;              /* DMA in circular mode to match with ADC configuration: DMA continuous requests */
		DmaHandle.Init.Priority            = DMA_PRIORITY_HIGH;

		/* Deinitialize  & Initialize the DMA for new transfer */
		HAL_DMA_DeInit(&DmaHandle);
		HAL_DMA_Init(&DmaHandle);

		/* Associate the initialized DMA handle to the ADC handle */
		__HAL_LINKDMA(hadc, DMA_Handle, DmaHandle);

		/*##-4- Configure the NVIC #################################################*/

		/* NVIC configuration for DMA interrupt (transfer completion or error) */
		/* Priority: high-priority */
		HAL_NVIC_SetPriority(DMA1_Ch1_IRQn, 1, 0);
		HAL_NVIC_EnableIRQ(DMA1_Ch1_IRQn);


		/* NVIC configuration for ADC interrupt */
		/* Priority: high-priority */
		HAL_NVIC_SetPriority(ADC1_COMP_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(ADC1_COMP_IRQn);

		/* USER CODE BEGIN ADC1_MspInit 1 */

		/* USER CODE END ADC1_MspInit 1 */
	}

}

/**
* @brief ADC MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hadc: ADC handle pointer
* @retval None
*/

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc) {

	if(hadc->Instance==ADC1) {
		/* USER CODE BEGIN ADC1_MspDeInit 0 */

		/* USER CODE END ADC1_MspDeInit 0 */
		/* Peripheral clock disable */
		__HAL_RCC_ADC1_CLK_DISABLE();

		/**ADC GPIO Configuration
		PA0     ------> ADC_IN0
		*/
		HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0);

		/*##-3- Disable the DMA ####################################################*/
		/* De-Initialize the DMA associated to the peripheral */
		if(hadc->DMA_Handle != NULL) {
			HAL_DMA_DeInit(hadc->DMA_Handle);
		}

		/*##-4- Disable the NVIC ###################################################*/
		/* Disable the NVIC configuration for DMA interrupt */
		HAL_NVIC_DisableIRQ(DMA1_Ch1_IRQn);

		/* Disable the NVIC configuration for ADC interrupt */
		HAL_NVIC_DisableIRQ(ADC1_COMP_IRQn);

		/* USER CODE BEGIN ADC1_MspDeInit 1 */

		/* USER CODE END ADC1_MspDeInit 1 */
	}
}

/**
* @brief UART MSP Initialization
* This function configures the hardware resources used in this example
* @param huart: UART handle pointer
* @retval None
*/
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(huart->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();
  
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART2 GPIO Configuration    
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX 
    */
    GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */
  }

}

/**
* @brief UART MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param huart: UART handle pointer
* @retval None
*/

void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{

  if(huart->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();
  
    /**USART2 GPIO Configuration    
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX 
    */
    HAL_GPIO_DeInit(GPIOA, USART_TX_Pin|USART_RX_Pin);

  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }

}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
