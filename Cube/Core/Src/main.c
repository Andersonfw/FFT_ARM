/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "math.h"
#include "string.h"
#include "stdio.h"
#include "arm_math.h"
#include "arm_const_structs.h"
#include "stdbool.h"
#include "stdlib.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

DAC_HandleTypeDef hdac1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*------------------------------------------------------------------------------

	N (Bins) = FFT Size/2

    FR = Fmax/N(Bins)

	For a 44100 sampling rate, we have a 22050 Hz band. With a 1024 FFT size,
	we divide this band into 512 bins.

	FR = 22050/1024 ≃ 21,53 Hz.
 -------------------------------------------------------------------------------*/

// Private define --------------------------------------------------------------
#define SAMPLE_SIZE 2048 // Tamanho da amostra do sinal (potência de 2)
#define FFT_SIZE (SAMPLE_SIZE / 2) // Tamanho da FFT. N_BINS
#define HANNING_COEFF		0.5f
#define FFTRMSSCALE 	1.4142f    //2.0(FFT) * 0.707(RMS)
#define HANN_KF_ACC			(2 * PI / (FFT_SIZE - 1))
#define FS 55000.0f						// Frequẽncia de amostragem
#define FC 2000.0f						// Frequeência de corte

//#define DSB 1				//Banda simples SSB ou dupla DSP no espectro de frequências

#ifdef DSB
	#define FFT_BINS (FFT_SIZE)
#else
	#define FFT_BINS (FFT_SIZE/2)
#endif
// Private variables -----------------------------------------------------------
float32_t input_signal_AC[SAMPLE_SIZE];	//sinal de entrada lido pelo AD normalizado de -1,5 até +1,5
float32_t fft_mag_BINS[FFT_BINS];		//amplitude dos BINS da FFT
float32_t fft_raw_BINS[SAMPLE_SIZE];	//Valores complexos dos BINS da FFT
uint32_t ifftFlag = 0; 					// 0 Real FFT RFFT; 1 Real Inversa RIFFT
uint32_t doBitReverse = 1;

arm_rfft_fast_instance_f32 fft;
arm_rfft_instance_f32 si;
arm_cfft_radix4_instance_f32 S;	/* ARM CFFT module */
float freqMax = 0;
float ampMax = 0;

/* Reference index at which max energy of bin ocuurs */
uint32_t testIndex = 0;					// Index de posição do BIN de máxima energia
float32_t maxValue = 0;						// Valor do BIN de máxima energia


volatile uint32_t g_ADCValue, g_DACValue;	// Váriaveis de armazenamento dos valores do ADC DAC
char buffer[10];							// Buffer de recepção da UART

/* Variáveis para aplicação do filtro SallenKey Segundo Ordem*/
float tanres = 0;
float32_t C = 0;
float32_t zeta = 0;
float32_t b0 = 0;
float32_t b1 = 0;
float32_t b2 = 0;
float32_t a0 = 0;
float32_t a1 = 0;
float32_t a2 = 0;
float32_t xh1 = 0;
float32_t xh2 = 0;
float32_t yh1 = 0;
float32_t yh2 = 0;
float32_t x,y = {0};

// Private function prototypes -------------------------------------------------
float HanningWindow(uint32_t u32Sample);
void FFTCalc(void);
void ComputeFft(float *pfInputSignal, float *pfFftOutput, bool bApplyWindow);

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

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

	MX_ADC1_Init();
	MX_DAC1_Init();
	MX_TIM3_Init();
	MX_USART2_UART_Init();
	/* USER CODE BEGIN 2 */


	tanres = tanf(M_PI  * FC / FS);

	C = 1/tanres;
	zeta = 0.707;

	b0 = 1/(1+2*zeta*C+C*C);
	b1 = 2*b0;
	b2 = b0;

	a0 = 1;
	a1 = 2*b0*(1-C*C);
	a2 = b0 * (1-2*zeta*C+C*C);

	xh1 = 0;
	xh2 = 0;
	yh1 = 0;
	yh2 = 0;

	HAL_UART_Receive_IT(&huart2, (uint8_t*)buffer, 1);  //Inicia o recebimento da UART
	HAL_TIM_Base_Start_IT(&htim3);						//Inicia o timer
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);				//Inicia o DAC
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */
		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 1;
	RCC_OscInitStruct.PLL.PLLN = 10;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void)
{

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_MultiModeTypeDef multimode = {0};
	ADC_ChannelConfTypeDef sConfig = {0};

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc1.Init.LowPowerAutoWait = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	hadc1.Init.OversamplingMode = DISABLE;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure the ADC multi-mode
	 */
	multimode.Mode = ADC_MODE_INDEPENDENT;
	if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief DAC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_DAC1_Init(void)
{

	/* USER CODE BEGIN DAC1_Init 0 */

	/* USER CODE END DAC1_Init 0 */

	DAC_ChannelConfTypeDef sConfig = {0};

	/* USER CODE BEGIN DAC1_Init 1 */

	/* USER CODE END DAC1_Init 1 */

	/** DAC Initialization
	 */
	hdac1.Instance = DAC1;
	if (HAL_DAC_Init(&hdac1) != HAL_OK)
	{
		Error_Handler();
	}

	/** DAC channel OUT1 config
	 */
	sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
	sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
	sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
	sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
	sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
	if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN DAC1_Init 2 */

	/* USER CODE END DAC1_Init 2 */

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void)
{

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	/* USER CODE BEGIN TIM3_Init 1 */
	uint32_t uwTimclock = 0;
	uint32_t uwPrescalerValue = 0;
	uint32_t periodValue = 0;

	/* Compute TIM1 clock */
	uwTimclock = HAL_RCC_GetPCLK2Freq();

	/* Compute the prescaler value to have TIM1 counter clock equal to 1MHz */
	uwPrescalerValue = (uint32_t) ((uwTimclock / 1000000) - 1);
	periodValue = (uint32_t) ((1000000 / FS) - 1);

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = uwPrescalerValue;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = periodValue;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_TIMING;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */

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
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

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

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	HAL_ADC_Start_IT(&hadc1);
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	g_ADCValue = HAL_ADC_GetValue(&hadc1);
	x = g_ADCValue;

	y = b0 * x + b1 * xh1 + b2 * xh2 - a1 * yh1 - a2 * yh2;
	yh2 = yh1;
	yh1 = y;
	xh2 = xh1;
	xh1 = x;

	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1,DAC_ALIGN_12B_R,(uint16_t)y);

	//Converter o valor para ponto flutuante e normalizar para [-1, 1]
	float32_t sample = ((float32_t)g_ADCValue * (3.3f/4096.0f)) - 1.5f; //variação de -1.5V até + 1,5V

	// Adicionar a amostra ao vetor de entrada
	static uint16_t index = 0;
	input_signal_AC[index] = sample;
	index++;
	//Se o vetor de entrada estiver completo, zera o index
	if (index >= SAMPLE_SIZE)
		index = 0;
}
// *****************************************************************************
/// @brief		Janela de Hanning
/// @fn			float HanningWindow(u32 u32Sample)
/// @param[in]	u32Sample			@brief Amostra
/// @retval		Amostra corrigida
// *****************************************************************************
float HanningWindow(uint32_t u32Sample)
{
	return (HANNING_COEFF * (1.0f - arm_cos_f32(HANN_KF_ACC * u32Sample)));
}
// *****************************************************************************
/// @brief		Calcula a FFT do sinal de entrada
/// @fn			void ComputeFft(float *pfInputSignal, float *pfFftOutput, bool bApplyWindow)
/// @param[in]	pfInputSignal		@brief Sinal de entrada
/// @param[out]	pfFftOutput			@brief FFT
// *****************************************************************************
void ComputeFft(float *pfInputSignal, float *pfFftOutput, bool bApplyWindow)
{
	int n;
	float fScale;
	float *pout;

	pout = pfFftOutput;
	n = 0;

	if(bApplyWindow)
	{
		fScale = FFTRMSSCALE / (HANNING_COEFF) / FFT_SIZE;	//fator de ajuste para normalizar ao valor RMS do sinal de entrada.
		while (n < FFT_SIZE)
		{
			*pout++ = pfInputSignal[n] * HanningWindow(n) ;	// aplica a janela
			n++;
			*pout++ = 0.0f;
			*pout++ = pfInputSignal[n] * HanningWindow(n) ;	// aplica a janela
			n++;
			*pout++ = 0.0f;
		}
	}else
	{
		fScale = FFTRMSSCALE / FFT_SIZE;	//fator de ajuste para normalizar ao valor RMS do sinal de entrada.
		while (n < FFT_SIZE)
		{
			*pout++ = pfInputSignal[n++] ;
			*pout++ = 0.0f;
			*pout++ = pfInputSignal[n++] ;
			*pout++ = 0.0f;
		}
	}
	fScale = 1.00f / FFT_SIZE;	//fator de ajuste para normalizar ao valor de pico do sinal de entrada.

	arm_cfft_f32(&arm_cfft_sR_f32_len1024, pfFftOutput, 0, 1);
//	arm_cmplx_mag_f32(pfFftOutput, pfFftOutput, FFT_BINS);	// calcula o módulo da amplitude
//	arm_scale_f32(pfFftOutput, fScale, pfFftOutput, FFT_BINS); //normaliza os valores conforme o fator
}

// *****************************************************************************
//	for (int n = 0; n < SAMPLE_SIZE; n += 2)
//		input_signal[n] *= HanningWindow(n);

//	/* Initialize the CFFT/CIFFT module, intFlag = 0, doBitReverse = 1 */
//	arm_cfft_radix4_init_f32(&S, FFT_SIZE, 0, 1);
//
//	/* Process the data through the CFFT/CIFFT module */
//	arm_cfft_radix4_f32(&S, input_signal);
//
//	/* Process the data through the Complex Magniture Module for calculating the magnitude at each bin */
//	arm_cmplx_mag_f32(input_signal, fft_raw_BINS, FFT_SIZE);
//
//	/* Calculates maxValue and returns corresponding value */
//	arm_max_f32(fft_raw_BINS, FFT_SIZE, &maxValue, &testIndex);

//	arm_rfft_fast_init_f32(&fft, FFT_SIZE);
//	arm_rfft_fast_f32(&fft, input_signal, output_signal, ifftFlag);
//	arm_cmplx_mag_f32(output_signal, fft_power, FFT_SIZE);
//	/* Calculates maxValue and returns corresponding BIN value */
//		arm_max_f32(fft_mag_BINS, FFT_SIZE, &maxValue, &testIndex);
// *****************************************************************************


// *****************************************************************************
/// @brief		Calculo da FFT
/// @fn			void FFTCalc(void)
/// @param[in]
/// @retval
// *****************************************************************************
void FFTCalc(void)
{
	ComputeFft(input_signal_AC, fft_raw_BINS,false);
	arm_cmplx_mag_f32(fft_raw_BINS, fft_mag_BINS, FFT_BINS);	// calcula o módulo da amplitude de número complexo
//	arm_abs_f32(fft_raw_BINS, fft_mag_BINS, FFT_BINS);			//calcula o módulo da amplitude de número float
	arm_scale_f32(fft_mag_BINS, (float)1.00f/FFT_SIZE, fft_mag_BINS, FFT_BINS);

	arm_max_f32(fft_mag_BINS, FFT_SIZE, &maxValue, &testIndex);
	freqMax = (float)testIndex * FS / FFT_SIZE;
	#ifdef DSB
		ampMax = fabs(maxValue);
	#else
		ampMax = 2 * fabs(maxValue);
	#endif
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	HAL_TIM_Base_Stop_IT(&htim3);
	if(huart->Instance == USART2)
	{
		int i;
		uint8_t u8data[10];
		uint8_t u8size = 0;
		if(buffer[0]== 'I')
		{
			for(i = 0; i < SAMPLE_SIZE - 1; i++ )
			{
				u8size = sprintf((char*)u8data,"%.5f, ",input_signal_AC[i]);
				HAL_UART_Transmit(&huart2, u8data, u8size,10);
			}
			u8size = sprintf((char*)u8data,"%.6f",input_signal_AC[i]);
			HAL_UART_Transmit(&huart2, u8data, u8size,10);
			HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", 2,10);
		}
		else if (buffer[0]== 'O')
		{
			FFTCalc();
			for(i = 0; i < FFT_BINS - 1; i++ )
			{
				u8size = sprintf((char*)u8data,"%.5f, ",fft_mag_BINS[i]);
				HAL_UART_Transmit(&huart2, u8data, u8size,10);
			}
			u8size = sprintf((char*)u8data,"%.5f",fft_mag_BINS[i]);
			HAL_UART_Transmit(&huart2, u8data, u8size,10);
			HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", 2,10);
		}
		else if (buffer[0]== 'C')
		{
			u8size = sprintf((char*)u8data,"%ld\r\n",(uint32_t)FC);
			HAL_UART_Transmit(&huart2, u8data, u8size,10);
		}
		else if (buffer[0]== 'S')
		{
			u8size = sprintf((char*)u8data,"%ld\r\n",(uint32_t)FS);
			HAL_UART_Transmit(&huart2, u8data, u8size,10);
		}
		else if (buffer[0]== 'M')
		{
			if(FFT_SIZE == FFT_BINS)
				u8size = sprintf((char*)u8data,"1\r\n"); //DSB
			else
				u8size = sprintf((char*)u8data,"0\r\n"); //SSB
			HAL_UART_Transmit(&huart2, u8data, u8size,10);
		}
		memset(buffer,0x00,sizeof(buffer));
		HAL_UART_Receive_IT(&huart2, (uint8_t*)buffer, 1);
		HAL_TIM_Base_Start_IT(&htim3);
	}
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	HAL_UART_Receive_IT(&huart2, (uint8_t*)buffer, 1);
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
