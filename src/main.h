/**
  ******************************************************************************
  * @file    GPIO/GPIO_IOToggle/Inc/main.h
  * @author  MCD Application Team
  * @version V1.6.0
  * @date    27-May-2016
  * @brief   Header for main.c module
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal_conf.h"
#include "stm32f0xx_hal_dma.h"
#include "stm32f0xx_hal.h"
#include "stm32f0xx_nucleo.h"
#include "stm32f0xx_hal_adc.h"
#include "stm32f0xx_hal_adc_ex.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
/* Exported variables ------------------------------------------------------- */
extern ADC_HandleTypeDef AdcHandle;

/* ## Definition of ADC related resources ################################### */
/* Definition of ADCx clock resources */
#define ADCx                            ADC1
#define ADCx_CLK_ENABLE()               __HAL_RCC_ADC1_CLK_ENABLE()

#define ADCx_FORCE_RESET()              __HAL_RCC_ADC1_FORCE_RESET()
#define ADCx_RELEASE_RESET()            __HAL_RCC_ADC1_RELEASE_RESET()

/* Definition of ADCx channels & pins*/
#define ADCx_CHANNELa                   ADC_CHANNEL_0
#define ADCx_CHANNELa_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()
#define ADCx_CHANNELa_GPIO_PORT         GPIOA
#define ADCx_CHANNELa_PIN               GPIO_PIN_0

#define ADCx_CHANNELb                   ADC_CHANNEL_1
#define ADCx_CHANNELb_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()
#define ADCx_CHANNELb_GPIO_PORT         GPIOA
#define ADCx_CHANNELb_PIN               GPIO_PIN_1

#define ADCx_CHANNELc                   ADC_CHANNEL_8
#define ADCx_CHANNELc_GPIO_CLK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()
#define ADCx_CHANNELc_GPIO_PORT         GPIOB
#define ADCx_CHANNELc_PIN               GPIO_PIN_0

#define ADCx_CHANNELd                   ADC_CHANNEL_9
#define ADCx_CHANNELd_GPIO_CLK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()
#define ADCx_CHANNELd_GPIO_PORT         GPIOB
#define ADCx_CHANNELd_PIN               GPIO_PIN_1

#define ADCx_CHANNELe                   ADC_CHANNEL_14
#define ADCx_CHANNELe_GPIO_CLK_ENABLE() __HAL_RCC_GPIOC_CLK_ENABLE()
#define ADCx_CHANNELe_GPIO_PORT         GPIOC
#define ADCx_CHANNELe_PIN               GPIO_PIN_4

#define ADCx_CHANNELf                   ADC_CHANNEL_15
#define ADCx_CHANNELf_GPIO_CLK_ENABLE() __HAL_RCC_GPIOC_CLK_ENABLE()
#define ADCx_CHANNELf_GPIO_PORT         GPIOC
#define ADCx_CHANNELf_PIN               GPIO_PIN_5

/* Definition of ADCx DMA resources */
#define ADCx_DMA_CLK_ENABLE()           __HAL_RCC_DMA1_CLK_ENABLE()
#define ADCx_DMA                        DMA1_Channel1
#define ADCx_DMA_IRQn                   DMA1_Ch1_IRQn
#define ADCx_DMA_IRQHandler             DMA1_Ch1_IRQHandler

/* Definition of ADCx NVIC resources */
#define ADCx_IRQn                       ADC1_COMP_IRQn
#define ADCx_IRQHandler                 ADC1_COMP_IRQHandler

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
