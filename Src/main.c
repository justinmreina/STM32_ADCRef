/**
  ******************************************************************************
  * @file    ADC/ADC_Sequencer/Src/main.c
  * @author  MCD Application Team
  * @brief   This example provides a short description of how to use the ADC
  *          peripheral with sequencer, to convert several channels.
  *          Channels converted are 1 channel on external pin and 2 internal
  *          channels (VrefInt and temperature sensor).
  *          Moreover, voltage and temperature are then computed.
  *
  * @section 	Opens
  * 	ADC_CHANNEL_13 not working, crashes Nucleo-64 and unsure why
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal_adc.h"
#include "stm32f0xx_hal_adc_ex.h"
#include "stm32f0xx_hal_dac.h"
#include "adc.h"
#include <string.h>

/** @addtogroup STM32F0xx_HAL_Examples
  * @{
  */

/** @addtogroup ADC_Sequencer
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define VDDA_APPLI                     ((uint32_t) 3300)    /* Value of analog voltage supply Vdda (unit: mV) */
#define RANGE_12BITS                   ((uint32_t) 4095)    /* Max digital value with a full range of 12 bits */

/* ADC parameters */
#define ADCCONVERTEDVALUES_BUFFER_SIZE ((uint32_t)    NUM_ADC_CHANNELS)    /* Size of array containing ADC converted values: set to ADC sequencer number of ranks converted, to have a rank in each address */

/* Internal temperature sensor: constants data used for indicative values in  */
/* this example. Refer to device datasheet for min/typ/max values.            */
/* For more accurate values, device should be calibrated on offset and slope  */
/* for application temperature range.                                         */
#define INTERNAL_TEMPSENSOR_V30        ((int32_t) 1430)         /* Internal temperature sensor, parameter V30 (unit: mV). Refer to device datasheet for min/typ/max values. */
#define INTERNAL_TEMPSENSOR_AVGSLOPE   ((int32_t) 4300)         /* Internal temperature sensor, parameter Avg_Slope (unit: uV/DegCelsius). Refer to device datasheet for min/typ/max values. */
#define TEMP30_CAL_ADDR   ((uint16_t*) ((uint32_t) 0x1FFFF7B8)) /* Internal temperature sensor, parameter TS_CAL1: TS ADC raw data acquired at a temperature of 110 DegC (+-5 DegC), VDDA = 3.3 V (+-10 mV). */
#define TEMP110_CAL_ADDR  ((uint16_t*) ((uint32_t) 0x1FFFF7C2)) /* Internal temperature sensor, parameter TS_CAL2: TS ADC raw data acquired at a temperature of  30 DegC (+-5 DegC), VDDA = 3.3 V (+-10 mV). */
#define VDDA_TEMP_CAL                  ((uint32_t) 3300)        /* Vdda value with which temperature sensor has been calibrated in production (+-10 mV). */

/* Internal voltage reference */
#define VREFINT_CAL       ((uint16_t*) ((uint32_t) 0x1FFFF7BA)) /* Internal temperature sensor, parameter VREFINT_CAL: Raw data acquired at a temperature of 30 DegC (+-5 DegC), VDDA = 3.3 V (+-10 mV). */
                                                                /* This calibration parameter is intended to calculate the actual VDDA from Vrefint ADC measurement. */

/* Private macro -------------------------------------------------------------*/
/**
  * @brief  Computation of temperature (unit: degree Celsius) from the internal
  *         temperature sensor measurement by ADC.
  *         Computation is using temperature sensor calibration values done
  *         in production.
  *         Computation formula:
  *         Temperature = (TS_ADC_DATA - TS_CAL1) * (110degC - 30degC)
  *                       / (TS_CAL2 - TS_CAL1) + 30degC
  *         with TS_ADC_DATA = temperature sensor raw data measured by ADC
  *              Avg_Slope = (TS_CAL2 - TS_CAL1) / (110 - 30)
  *              TS_CAL1 = TS_ADC_DATA @30degC (calibrated in factory)
  *              TS_CAL2 = TS_ADC_DATA @110degC (calibrated in factory)
  *         Calculation validity conditioned to settings:
  *          - ADC resolution 12 bits (need to scale conversion value if using a different resolution).
  *          - Power supply of analog voltage set to literal VDDA_APPLI (need to scale value if using a different value of analog
  *            voltage supply).
  * @param TS_ADC_DATA: Temperature sensor digital value measured by ADC
  * @retval None
  */
#define COMPUTATION_TEMPERATURE_TEMP30_TEMP110(TS_ADC_DATA)                    \
  (((( ((int32_t)((TS_ADC_DATA * VDDA_APPLI) / VDDA_TEMP_CAL)                  \
        - (int32_t) *TEMP30_CAL_ADDR)                                          \
     ) * (int32_t)(110 - 30)                                                   \
    ) / (int32_t)(*TEMP110_CAL_ADDR - *TEMP30_CAL_ADDR)                        \
   ) + 30                                                                      \
  )

/**
  * @brief  Computation of temperature (unit: degree Celsius) from the internal temperature sensor measurement by ADC. Computation
  * 		is using temperature sensor standard parameters (refer to device datasheet).
  *         Computation formula:
  *         Temperature = (VTS - V30)/Avg_Slope + 30
  *         with VTS = temperature sensor voltage
  *              Avg_Slope = temperature sensor slope (unit: uV/DegCelsius)
  *              V30 = temperature sensor @25degC and Vdda defined at VDDA_TEMP_CAL (unit: mV)
  *         Calculation validity conditioned to settings:
  *          - ADC resolution 12 bits (need to scale value if using a different resolution).
  *          - Power supply of analog voltage set to literal VDDA_APPLI (need to scale value if using a different value of analog
  *            voltage supply).
  * @param TS_ADC_DATA: Temperature sensor digital value measured by ADC
  * @retval None
  */
#define COMPUTATION_TEMPERATURE_STD_PARAMS_AVGSLOPE_V30(TS_ADC_DATA)           \
  ((( ((int32_t)((INTERNAL_TEMPSENSOR_V30 * VDDA_TEMP_CAL) / VDDA_APPLI)       \
       - (int32_t)(((TS_ADC_DATA) * VDDA_APPLI) / RANGE_12BITS)                \
      ) * 1000                                                                 \
    ) / INTERNAL_TEMPSENSOR_AVGSLOPE                                           \
   ) + 25                                                                      \
  )

/**
  * @brief  Computation of voltage (unit: mV) from ADC measurement digital value on range 12 bits.
  *         Calculation validity conditioned to settings:
  *          - ADC resolution 12 bits (need to scale value if using a different resolution).
  *          - Power supply of analog voltage Vdda 3.3V (need to scale value if using a different analog voltage supply value).
  * @param ADC_DATA: Digital value measured by ADC
  * @retval None
  */
#define COMPUTATION_DIGITAL_12BITS_TO_VOLTAGE(ADC_DATA)                        \
  ( ((ADC_DATA) * VDDA_APPLI) / RANGE_12BITS)



/* Private variables ---------------------------------------------------------*/
/* Peripherals handlers declaration */
/* ADC handler declaration */
ADC_HandleTypeDef    AdcHandle;

#if defined(WAVEFORM_VOLTAGE_GENERATION_FOR_TEST)
/* DAC handler declaration */
DAC_HandleTypeDef    DacHandle;  /* DAC used for waveform voltage generation for test */
#endif /* WAVEFORM_VOLTAGE_GENERATION_FOR_TEST */

/* Variable containing ADC conversions results */
__IO uint16_t   aADCxConvertedValues[ADCCONVERTEDVALUES_BUFFER_SIZE];

/* Variables for ADC conversions results computation to physical values */
uint16_t   uhADCChannelToDAC_mVolt = 0;
uint16_t   uhVrefInt_mVolt = 0;
 int32_t   wTemperature_DegreeCelsius = 0;

/* Variables to manage push button on board: interface between ExtLine interruption and main program */
uint8_t         ubUserButtonClickCount = 0;      /* Count number of clicks: Incremented after User Button interrupt */
__IO uint8_t    ubUserButtonClickEvent = RESET;  /* Event detection: Set after User Button interrupt */

/* Variable to report ADC sequencer status */
uint8_t         ubSequenceCompleted = RESET;     /* Set when all ranks of the sequence have been converted */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
#if defined(WAVEFORM_VOLTAGE_GENERATION_FOR_TEST)
static void WaveformVoltageGenerationForTest_Config(void);
static void WaveformVoltageGenerationForTest_Update(void);
#endif /* WAVEFORM_VOLTAGE_GENERATION_FOR_TEST */

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void) {

	//Locals
	HAL_StatusTypeDef result;

	/* STM32F0xx HAL library initialization:
	- Configure the Flash prefetch
	- Systick timer is configured by default as source of time base, but user  can eventually implement his proper time base
	source (a general purpose timer for example or other time source), keeping in mind that Time base duration should be kept
	1ms since PPP_TIMEOUT_VALUEs are defined and handled in milliseconds basis.
	- Low Level Initialization
	*/
	HAL_Init();

	/* Configure the system clock to 48 MHz */
	SystemClock_Config();


	/*## Configure peripherals #################################################*/

	/* Initialize LED on board */
	BSP_LED_Init(LED2);

	/* Configure User push-button in Interrupt mode */
	BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

	/* Configure the ADCx peripheral */
	adc_config_init();

	/* Run the ADC calibration */
	if (HAL_ADCEx_Calibration_Start(&AdcHandle) != HAL_OK) {
		Error_Handler();													/* Calibration Error 									*/
	}


#if defined(WAVEFORM_VOLTAGE_GENERATION_FOR_TEST)
	/* Configure the DAC peripheral and generate a constant voltage of Vdda/2.  */
	WaveformVoltageGenerationForTest_Config();
#endif /* WAVEFORM_VOLTAGE_GENERATION_FOR_TEST */

	/*## Enable peripherals ####################################################*/

	/*## Start ADC conversions #################################################*/

	/* Start ADC conversion on regular group with transfer by DMA */
	if (HAL_ADC_Start_DMA(&AdcHandle, (uint32_t *)aADCxConvertedValues, ADCCONVERTEDVALUES_BUFFER_SIZE) != HAL_OK) {
		Error_Handler();													/* Start Error 											*/
	}

	/* Infinite loop */
	for(;;) {

		/* Wait for event on push button to perform following actions */
		while ((ubUserButtonClickEvent) == RESET);

		ubUserButtonClickEvent = RESET;										/* Reset variable for next loop iteration 				*/

		#if defined(WAVEFORM_VOLTAGE_GENERATION_FOR_TEST)
		/* Modifies the voltage level incrementally from 0V to Vdda at each call. Circular waveform of ramp: When the maximum level is 	*/
		/*reaches, restart from 0V. 																									*/
		WaveformVoltageGenerationForTest_Update();
		#endif /* WAVEFORM_VOLTAGE_GENERATION_FOR_TEST */


		/* Start ADC conversion */
		/* Since sequencer is enabled in discontinuous mode, this will perform the conversion of the next rank in sequencer. 		*/
		/* Note: For this example, conversion is triggered by software start, therefore "HAL_ADC_Start()" must be called for each 	*/
		/*       conversion. Since DMA transfer has been initiated previously by function "HAL_ADC_Start_DMA()", this function will */
		/*		 keep DMA transfer active. 																							*/
		result = HAL_ADC_Start(&AdcHandle);

		if(result != HAL_OK) {
			Error_Handler();
		}

		/* Wait for conversion completion */
		/* Note: A fixed wait time of 1ms is used for the purpose of this         													*/
		/*       example: ADC conversions are decomposed between each rank of the ADC sequencer. Function 							*/
		/*		 "HAL_ADC_PollForConversion(&AdcHandle, 1)" could be used, instead of wait time, but with a different configuration */
		/*		 (this function cannot be used if ADC configured in DMA mode and polling for end of each conversion): a possible 	*/
		/*		 configuration is ADC polling for the entire sequence (ADC init parameter "EOCSelection" set to ADC_EOC_SEQ_CONV) 	*/
		/*		 (this also induces that ADC discontinuous mode must be disabled).           										*/
		HAL_Delay(1);

		/* Turn-on/off LED2 in function of ADC sequencer status 																	*/
		/* - Turn-off if sequencer has not yet converted all ranks 																	*/
		/* - Turn-on if sequencer has converted all ranks 																			*/
		if (ubSequenceCompleted == RESET) {
			BSP_LED_Off(LED2);
		} else {
			BSP_LED_On(LED2);

			/* Computation of ADC conversions raw data to physical values */
			/* Note: ADC results are transferred into array "aADCxConvertedValues" in the order of their rank in ADC sequencer.		*/
			uhADCChannelToDAC_mVolt    = COMPUTATION_DIGITAL_12BITS_TO_VOLTAGE(aADCxConvertedValues[0]);
			uhVrefInt_mVolt            = COMPUTATION_DIGITAL_12BITS_TO_VOLTAGE(aADCxConvertedValues[2]);
			wTemperature_DegreeCelsius = COMPUTATION_TEMPERATURE_TEMP30_TEMP110(aADCxConvertedValues[1]);

			/* Reset variable for next loop iteration */
			ubSequenceCompleted = RESET;
			memset((uint8_t *)aADCxConvertedValues, 0xFF, sizeof(aADCxConvertedValues));
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
void SystemClock_Config(void) {

	//Locals
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_OscInitTypeDef RCC_OscInitStruct;

	/* Select HSI48 Oscillator as PLL source */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
	RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI48;
	RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV2;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct)!= HAL_OK) {
		while(1);															/* Initialization Error 								*/
	}

	/* Select PLL as system clock source and configure the HCLK and PCLK1 clocks dividers */
	RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1);
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1)!= HAL_OK) {
		while(1);																/* Initialization Error 								*/
	}

	return;
}


#if defined(WAVEFORM_VOLTAGE_GENERATION_FOR_TEST)
/**
  * @brief  For this example, generate a waveform voltage on a spare DAC
  *         channel, so user has just to connect a wire between DAC channel
  *         (pin PA.04) and ADC channel (pin PA.04) to run this example.
  *         (this prevents the user from resorting to an external signal generator)
  *         This function configures the DAC and generates a constant voltage of Vdda/2.
  *         To modify the voltage level, use function "WaveformVoltageGenerationForTest_Update"
  * @param  None
  * @retval None
  */
static void WaveformVoltageGenerationForTest_Config(void)
{
  static DAC_ChannelConfTypeDef sConfig;

  /*## Configure peripherals #################################################*/
  /* Configuration of DACx peripheral */
  DacHandle.Instance = DACx;

  if (HAL_DAC_Init(&DacHandle) != HAL_OK)
  {
    /* DAC initialization error */
    Error_Handler();
  }

  /* Configuration of DAC channel */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;

  if (HAL_DAC_ConfigChannel(&DacHandle, &sConfig, DACx_CHANNEL_TO_ADCx_CHANNELa) != HAL_OK)
  {
    /* Channel configuration error */
    Error_Handler();
  }

  /*## Enable peripherals ####################################################*/

  /* Set DAC Channel data register: channel corresponding to ADC channel CHANNELa */
  /* Set DAC output to 1/2 of full range (4095 <=> Vdda=3.3V): 2048 <=> 1.65V */
  if (HAL_DAC_SetValue(&DacHandle, DACx_CHANNEL_TO_ADCx_CHANNELa, DAC_ALIGN_12B_R, RANGE_12BITS/2) != HAL_OK)
  {
    /* Setting value Error */
    Error_Handler();
  }

  /* Enable DAC Channel: channel corresponding to ADC channel CHANNELa */
  if (HAL_DAC_Start(&DacHandle, DACx_CHANNEL_TO_ADCx_CHANNELa) != HAL_OK)
  {
    /* Start Error */
    Error_Handler();
  }

}

/**
  * @brief  For this example, generate a waveform voltage on a spare DAC
  *         channel, so user has just to connect a wire between DAC channel
  *         (pin PA.04) and ADC channel (pin PA.04) to run this example.
  *         (this prevents the user from resorting to an external signal generator)
  *         This function modifies the voltage level  from 0V to Vdda in 4 steps,
  *         incrementally at each function call.
  *         Circular waveform of ramp: When the maximum level is reaches,
  *         restart from 0V.
  * @param  None
  * @retval None
  */
static void WaveformVoltageGenerationForTest_Update(void)
{
  static uint8_t ub_dac_steps_count = 0;      /* Count number of clicks: Incremented after User Button interrupt */

  /* Set DAC voltage on channel corresponding to ADCx_CHANNELa              */
  /* in function of user button clicks count.                               */
  /* Set DAC output on 5 voltage levels, successively to:                   */
  /*  - minimum of full range (0 <=> ground 0V)                             */
  /*  - 1/4 of full range (4095 <=> Vdda=3.3V): 1023 <=> 0.825V             */
  /*  - 1/2 of full range (4095 <=> Vdda=3.3V): 2048 <=> 1.65V              */
  /*  - 3/4 of full range (4095 <=> Vdda=3.3V): 3071 <=> 2.475V             */
  /*  - maximum of full range (4095 <=> Vdda=3.3V)                          */
  if (HAL_DAC_SetValue(&DacHandle,
                       DACx_CHANNEL_TO_ADCx_CHANNELa,
                       DAC_ALIGN_12B_R,
                       ((RANGE_12BITS * ub_dac_steps_count) / 4)
                      ) != HAL_OK)
  {
    /* Start Error */
    Error_Handler();
  }

  /* Wait for voltage settling time */
  HAL_Delay(1);

  /* Manage ub_dac_steps_count to increment it in 4 steps and circularly.   */
  if (ub_dac_steps_count < 4)
  {
    ub_dac_steps_count++;
  }
  else
  {
    ub_dac_steps_count = 0;
  }

}
#endif /* WAVEFORM_VOLTAGE_GENERATION_FOR_TEST */


/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == USER_BUTTON_PIN)
  {
    /* Set variable to report push button event to main program */
    ubUserButtonClickEvent = SET;
  }
}

/**
  * @brief  Conversion complete callback in non blocking mode
  * @param  AdcHandle : ADC handle
  * @note   This example shows a simple way to report end of conversion
  *         and get conversion result. You can add your own implementation.
  * @retval None
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *AdcHandle)
{
  /* Report to main program that ADC sequencer has reached its end */
  ubSequenceCompleted = SET;
}

/**
  * @brief  Conversion DMA half-transfer callback in non blocking mode
  * @param  hadc: ADC handle
  * @retval None
  */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{

}

/**
  * @brief  ADC error callback in non blocking mode
  *        (ADC conversion with interruption or transfer by DMA)
  * @param  hadc: ADC handle
  * @retval None
  */
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
  /* In case of ADC error, call main error handler */
  Error_Handler();
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* User may add here some code to deal with a potential error */

  /* In case of error, LED2 is toggling at a frequency of 1Hz */
  while(1)
  {
    /* Toggle LED2 */
    BSP_LED_Toggle(LED2);
    HAL_Delay(500);
  }
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}

#endif

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
