/* USER CODE BEGIN Header */
/**
**************************************************************************************************************************************
* @file           : main.c
* @brief          : Main program body
*
* @section 		Reference
* 	1.	(Multi-Sequence ADC) C:\Users\justin\STM32Cube\Repository\STM32Cube_FW_F0_V1.9.0\Projects\STM32F091RC-Nucleo\Examples\
* 							 ADC\ADC_Sequencer
*
**************************************************************************************************************************************
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
**************************************************************************************************************************************
*/
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------------------------------------------------------------*/

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define ADCCONVERTEDVALUES_BUFFER_SIZE ((uint32_t)    3)    /* Size of array containing ADC converted values: set to ADC sequencer number of ranks converted, to have a rank in each address */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------------------------------------------------------------*/
ADC_HandleTypeDef hadc;

UART_HandleTypeDef huart2;

HAL_StatusTypeDef result;

uint32_t vals[ADC_NUM_CHANNELS];											/* adc measurement values								*/

/* Variable containing ADC conversions results */
uint32_t adc_channels[ADC_NUM_CHANNELS] = {ADC_CHANNEL_0,  ADC_CHANNEL_1,  ADC_CHANNEL_4,  ADC_CHANNEL_6,  ADC_CHANNEL_7,
										   ADC_CHANNEL_8,  ADC_CHANNEL_9,  ADC_CHANNEL_10, ADC_CHANNEL_11, ADC_CHANNEL_12,
										   ADC_CHANNEL_13, ADC_CHANNEL_14,	ADC_CHANNEL_15};

uint32_t adc_vals[ADC_NUM_CHANNELS];
double   volt_vals[ADC_NUM_CHANNELS];

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------------------------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC_Init(void);

/* USER CODE BEGIN PFP */

void _delay(uint32_t count);
double adc_getVoltage(uint32_t adc_val);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
* @brief  The application entry point.
* @retval int
*/
int main(void) {

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration -----------------------------------------------------------------------------------------------------------*/

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
	MX_USART2_UART_Init();
	MX_ADC_Init();

	/* USER CODE BEGIN 2 */

	  /*## Start ADC conversions #################################################*/

	/* Start ADC conversion on regular group with transfer by DMA */
	result = HAL_ADC_Start_DMA(&hadc, (uint32_t *)adc_vals, ADCCONVERTEDVALUES_BUFFER_SIZE);

	if(result != HAL_OK) {
		/* Start Error */
		Error_Handler();
	}

	/* USER CODE END 2 */

	/* Infinite loop */

	/* USER CODE BEGIN WHILE */

	for(;;) {

		//For each channel
		for(int i=0; i<ADC_NUM_CHANNELS; i++) {

			//Sample Channel
			result = HAL_ADC_Start(&hadc);

			//Safety
			if(result != HAL_OK) {
				Error_Handler();
			}

			_delay(100);

			//Wait for completion
			result = HAL_ADC_PollForConversion(&hadc, ADC_POLL_TIMEOUT_MS);

			//Safety
			if(result != HAL_OK) {
				Error_Handler();
			}

			_delay(100);

			//Get Value
			vals[i] = HAL_ADC_GetValue(&hadc);

			//Parse
			volt_vals[i] = adc_getVoltage(vals[i]);

			_delay(100);
		}
		_nop();																/* dev breakpoint loc									*/
	}

	/* USER CODE END 3 */

	return 0;
}


/**
* @brief System Clock Configuration
* @retval None
*/
void SystemClock_Config(void) {

	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

	/**Initializes the CPU, AHB and APB busses clocks */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.HSI14CalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
	RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV2;

	if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/**Initializes the CPU, AHB and APB busses clocks */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		Error_Handler();
	}

	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
	PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	if(HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}

	return;
}


/**
* @brief ADC Initialization Function
* @param None
* @retval None
*/
static void MX_ADC_Init(void) {

	//Locals
	ADC_ChannelConfTypeDef sConfig;

	//Init
	memset(&sConfig, 0, sizeof(sConfig));

	/**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) */
	hadc.Instance = ADC1;
	hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
	hadc.Init.Resolution = ADC_RESOLUTION_12B;
	hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
	hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc.Init.LowPowerAutoWait = DISABLE;
	hadc.Init.LowPowerAutoPowerOff = DISABLE;
	hadc.Init.ContinuousConvMode = DISABLE;
	hadc.Init.DiscontinuousConvMode = ENABLE;
	hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc.Init.DMAContinuousRequests = ENABLE;
	hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	hadc.Init.SamplingTimeCommon = ADC_SAMPLETIME_239CYCLES_5;				/* Note: Set long sampling time due to internal channels (VrefInt, temperature sensor) constraints. Refer to device datasheet for min/typ/max values. */

	result = HAL_ADC_Init(&hadc);

	if(result != HAL_OK) {
		Error_Handler();
	}

	//Setup Sequencer
	for(int i=0; i<ADC_NUM_CHANNELS; i++) {

		/**Configure channel #1 for the selected ADC regular channel to be converted. */
		sConfig.Channel = adc_channels[i];
		sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
		sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;

		result = HAL_ADC_ConfigChannel(&hadc, &sConfig);

		if(result != HAL_OK) {
			Error_Handler();
		}
	}


	//Calibrate
	result = HAL_ADCEx_Calibration_Start(&hadc);						/* Run the ADC calibration 								*/

	//Safety
	if(result != HAL_OK) {
	    Error_Handler();	    											/* Calibration Error 									*/
	}

	return;
}

/**
* @brief USART2 Initialization Function
* @param None
* @retval None
*/
static void MX_USART2_UART_Init(void)
{

/* USER CODE BEGIN USART2_Init 0 */

/* USER CODE END USART2_Init 0 */

/* USER CODE BEGIN USART2_Init 1 */

/* USER CODE END USART2_Init 1 */
huart2.Instance = USART2;
huart2.Init.BaudRate = 115200;
huart2.Init.WordLength = UART_WORDLENGTH_8B;
huart2.Init.StopBits = UART_STOPBITS_1;
huart2.Init.Parity = UART_PARITY_NONE;
huart2.Init.Mode = UART_MODE_TX_RX;
huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
huart2.Init.OverSampling = UART_OVERSAMPLING_16;
huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
if (HAL_UART_Init(&huart2) != HAL_OK)
{
Error_Handler();
}
/* USER CODE BEGIN USART2_Init 2 */

/* USER CODE END USART2_Init 2 */

}

/**
* @brief GPIO Initialization Function
* @param None
* @retval None
*/
static void MX_GPIO_Init(void)
{
GPIO_InitTypeDef GPIO_InitStruct = {0};

/* GPIO Ports Clock Enable */
__HAL_RCC_GPIOC_CLK_ENABLE();
__HAL_RCC_GPIOF_CLK_ENABLE();
__HAL_RCC_GPIOA_CLK_ENABLE();

/*Configure GPIO pin Output Level */
HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

/*Configure GPIO pin : B1_Pin */
GPIO_InitStruct.Pin = B1_Pin;
GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
GPIO_InitStruct.Pull = GPIO_NOPULL;
HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

/*Configure GPIO pin : LD2_Pin */
GPIO_InitStruct.Pin = LD2_Pin;
GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
GPIO_InitStruct.Pull = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
* @fcn		void Error_Handler(void)
* @brief  	This function is executed in case of error occurrence.
* @detail 	User can add his own implementation to report the HAL error return state
*
* @return 	None
*/
void Error_Handler(void) {

	/* USER CODE BEGIN Error_Handler_Debug */

	/* USER CODE END Error_Handler_Debug */

	return;
}

/************************************************************************************************************************************/
/**
 * @fcn			void _delay(uint32_t count)
 * @brief  		small time delay
 * @detail 		x
 *
 * @param 		[in] (uint32_t) count - counts of delay
 */
/************************************************************************************************************************************/
void _delay(uint32_t count) {

	//Delay
	for(int i=0; i<count; i++) {
		asm(" nop");
	}

	return;
}


/************************************************************************************************************************************/
/**	@fcn		double adc_getVoltage(uint32_t adc_val)
 *  @brief		convert adc measurement to voltage value
 *  @details	x
 *
 *  @param 		[in] (uint32_t) adc_val - adc measured value
 *  @return		(double) voltage value of measurement
 */
/************************************************************************************************************************************/
double adc_getVoltage(uint32_t adc_val) {

	//Locals
	double pin_val;

	//Calc
	pin_val = (((double)adc_val)*VCC_VOLTS)/(ADC_RANGE);

	return pin_val;
}


/************************************************************************************************************************************/
/**	@fcn		void adc_getChannelConfig(uint32_t channel)
 *  @brief		channel for configuration information
 *  @details	x
 *
 *  @param 		[in] (uint32_t) channel - channel for selection
 *
 *  @return		(AdcChannelConfig) selected channel configuration
 */
/************************************************************************************************************************************/
AdcChannelConfig adc_getChannelConfig(uint32_t channel) {

	//Locals
	AdcChannelConfig config;

	//Init State
	config.channel = channel;												/* store the specified channel value					*/

	//Lookup Port&Pin
	switch(channel) {
		case ADC_CHANNEL_0:													/* PA0													*/
			config.port = GPIOA;
			config.pin  = 0;
			break;
		case ADC_CHANNEL_1:													/* PA1													*/
			config.port = GPIOA;
			config.pin  = 1;
			break;
		case ADC_CHANNEL_2:													/* PA2													*/
			config.port = GPIOA;
			config.pin  = 2;
			break;
		case ADC_CHANNEL_3:													/* PA3													*/
			config.port = GPIOA;
			config.pin  = 3;
			break;
		case ADC_CHANNEL_4:													/* PA4													*/
			config.port = GPIOA;
			config.pin  = 4;
			break;
		case ADC_CHANNEL_5:													/* PA5													*/
			config.port = GPIOA;
			config.pin  = 5;
			break;
		case ADC_CHANNEL_6:													/* PA6													*/
			config.port = GPIOA;
			config.pin  = 6;
			break;
		case ADC_CHANNEL_7:													/* PA7													*/
			config.port = GPIOA;
			config.pin  = 7;
			break;
		case ADC_CHANNEL_8:													/* PB0													*/
			config.port = GPIOB;
			config.pin  = 0;
			break;
		case ADC_CHANNEL_9:													/* PB1													*/
			config.port = GPIOB;
			config.pin  = 1;
			break;
		case ADC_CHANNEL_10:												/* PC0													*/
			config.port = GPIOC;
			config.pin  = 0;
			break;
		case ADC_CHANNEL_11:												/* PC1													*/
			config.port = GPIOC;
			config.pin  = 1;
			break;
		case ADC_CHANNEL_12:												/* PC2													*/
			config.port = GPIOC;
			config.pin  = 2;
			break;
		case ADC_CHANNEL_13:												/* PC3													*/
			config.port = GPIOC;
			config.pin  = 3;
			break;
		case ADC_CHANNEL_14:												/* PC4													*/
			config.port = GPIOC;
			config.pin  = 4;
			break;
		case ADC_CHANNEL_15:												/* PC5													*/
			config.port = GPIOC;
			config.pin  = 5;
			break;
		default:															/* safety catch on error input							*/
			for(;;);
	}

	//Store GPIO pin value
	config.gpio_pin = (1 << config.pin);									/* e.g. 0x01 for "GPIO_PIN_0"							*/

	return config;
}


#ifdef  USE_FULL_ASSERT
/**
* @brief  Reports the name of the source file and the source line number
*         where the assert_param error has occurred.
* @param  file: pointer to the source file name
* @param  line: assert_param error line source number
* @retval None
*/
void assert_failed(char *file, uint32_t line)
{ 
/* USER CODE BEGIN 6 */
/* User can add his own implementation to report the file name and line number,
tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
