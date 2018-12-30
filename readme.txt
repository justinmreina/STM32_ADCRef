@brief 		Project Generation and Reference Notes
@details 	For successful configuration and use of adc
@auth 		Justin Reina
@date 		12/29/18

@note 	Defaults are used unless otherwise specified


@section 	Generation Procedure
	Open TrueSTUDIO STM32							(v9.0.1)
	  File->New C Project->Embedded C Project
	  Device->STM32F0->Boards->NUCLEO-F091RC
	  Runtime library->Library: 'Newlib standard'	[#FF90]
	  Deploy & Debug to Nucleo-F091RC 				[#25DD]	
	Open STM32CubeMX V1.0 							(Version 5.0.0)
	  Access to Board Selector->Part Number Search: 'NUCLEO-F091RC'
	  Start Project
	  Init to Defaults? 'Yes'
	  Pinout & Config->Analog->ADC: IN0 + IN1
	  Project Name: "STM32_ADCRef-CubeMx"
	  Toolchain: TrueSTUDIO
	  Generate Code									[#C5F2]
	  
@section 	Branch Description
	Single_Discrete: One channel discrete sample acquisition
	Single_DMA: One channel DMA sample acquisition
	Multi_Discrete: Multi-channel discrete sample acquisition
	MultiSample_DMA: Multi-channel DMA sample acquisition
	CubeMX_ADC_AnalogWatchdog: STM32Cube_FW_F0_V1.9.0\Projects\
		STM32F091RC-Nucleo\Examples\ADC\ADC_AnalogWatchdog 
	CubeMX_ADC_Sequencer: STM32Cube_FW_F0_V1.9.0\Projects\
		STM32F091RC-Nucleo\Examples\ADC\ADC_Sequencer

@section 	Notes
	ADC_IN0: PA0
	ADC_IN1: PA1

