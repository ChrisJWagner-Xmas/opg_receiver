/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "constants.h"
#include "current_source.h"
#include "ads8671.h"
#include "frame_encoding.h"
#include "sensor.h"
#include "math.h"
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

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
HAL_StatusTypeDef status = HAL_OK;

// ++++++++ Measurement global variables ++++++++++++
uint32_t frame_number = 0; // keep track of frames.
uint8_t red_led_status = 0;
uint8_t do_crosstalk_recalculation = 0;
uint8_t reset_crosstalk_values = 0;
uint8_t crosstalk_fsm_counter = 0;
int16_t crosstalk_values[NUM_SENSORS] = {0};
int16_t crosstalk_offset = 0;

enum State{
	ST_IDLE,
	ST_MEASURE,
	ST_TRANSMIT,
	ST_ERROR
};

static enum State state = ST_IDLE;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  // Measurement variables.
  int16_t sensor_values[NUM_SENSORS] = {0};
  int16_t sensor_values_tx[NUM_SENSORS] = {0};
  int16_t offset_values[NUM_SENSORS] = {0};

  uint8_t frame_buffer[FRAME_BUFFER_LENGTH_BYTES] = {0};
  frame_buffer[0] = START_BYTE_0; // Initialize here b.c. these never change.
  frame_buffer[1] = START_BYTE_1; // -"-

  // Timer variables.
  uint32_t PRESC_VALUE = TIM3_PERIPHERAL_CLOCK_FREQ_HZ/(MEASURING_FREQUENCY_HZ*ARR_VALUE) - 1;
  uint32_t PRESC_VALUE_BUTTON_CAPTURE = TIM3_PERIPHERAL_CLOCK_FREQ_HZ/(BUTTON_CAPTURE_MIN_FREQ_HZ*ARR_VALUE) - 1;

  if(PRESC_VALUE >= 65535)
  {
	  GPIOB->BSRR = STATUS_LED_RED_B_HIGH;
  }

  if(PRESC_VALUE_BUTTON_CAPTURE >= 65535)
  {
	  GPIOB->BSRR = STATUS_LED_RED_B_HIGH;
  }

  // ADC1 & current source variables.
  uint16_t current_lsb = 0;

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
  MX_USB_DEVICE_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

	// ***********************************************
	// ************** Initializations *****************
	// ***********************************************

  // ++++++++++++++++ Configure the timer. ++++++++++++++++++
  // Set the measuring frequency. f_meas = f_apb1/((PSC+1)*ARR)
    TIM3->PSC = (uint16_t)PRESC_VALUE;
    TIM3->ARR = (uint16_t)ARR_VALUE;
    // TIM3->CCR1 = (uint16_t)ARR_VALUE;
	TIM3->DIER |= TIM_DIER_UIE; // Enable interrupt generation when CNT = ARR.

	  // SPI SETUP
	  // Enable interrupts when transmission or reception is completed and rx buffer is not empty
	  // or the transmit buffer is empty.
	  SPI1->CR2 |= SPI_CR2_RXNEIE;
	  SPI1->CR2 |= SPI_CR2_TXEIE;
	  SPI1->CR1 |= SPI_CR1_SPE;

	// ++++++++++++++++ ADC SETUP ++++++++++++++++
    // TODO: PUT INTO OWN FUNCTION.

    // Enable end of conversion (EOC) interrupts.
  	ADC1->CR1 |= ADC_CR1_EOCIE;
  	NVIC_EnableIRQ(ADC1_2_IRQn); // TODO: NECESSARY?

  	// Set the trigger as software trigger (page 241).
  	ADC1->CR2 |= (ADC_CR2_EXTSEL_2 & ADC_CR2_EXTSEL_1 & ADC_CR2_EXTSEL_0);

	// Wake up the ADC from any power-down mode.
	ADC1->CR2 |= ADC_CR2_ADON;
	HAL_Delay(1);

	// Turn the ADC on a second time (TODO: NECESSARY?).
	ADC1->CR2 |= ADC_CR2_ADON;

	// Calibrate the ADC.
	ADC1->CR2 |= ADC_CR2_CAL; // set calibration start bit
	while(ADC1->CR2 & ADC_CR2_CAL)
	{
		asm("NOP"); // Wait for the calibration to finish. ADC_CR2_CAL flips back to 0.
	}

	// ***********************************************
	// *********** Initialization tests ***************
	// ***********************************************

  // +++++ Test the current source and emitter +++++++

  // ++++++ Read out the current through each emitter. ++++++
  int failing_sensor_index = check_sensor_currents();
  if(failing_sensor_index > 0)
  {
	  state = ST_ERROR;
  }

  // TODO: Blink STATUS_LED if any of the emitter is out of range [2, XXX] mA.

  // ++++++ Test the STATUS_LED once ++++++++++++++++++++++++
  GPIOB->BSRR = STATUS_LED_B_HIGH;
  HAL_Delay(1000);
  GPIOB->BSRR = STATUS_LED_B_LOW;

  // +++++++++++++++++ Test the ADC +++++++++++++++++++++++++
  // Toggle the CS Line high initially.
  GPIOA->BSRR = SPI_CS_HIGH;

  uint32_t data_word = 0;

  // Change the device id for testing.
  spi_tx_word(WRITE_ID_TO_DEVICE);
  spi_tx_word(READ_ID_FROM_DEVICE);
  data_word = spi_rx_word();

    uint8_t return_reg_value = data_word >> 24; // spi_rx_buffer[0];
  // return_reg_value = spi_rx_buffer[0];
  if(return_reg_value != 0x05)
  {
	  GPIOA->BSRR = STATUS_LED_B_HIGH;
  }

  // Change the range select register to a range of 0 to 3xVref.
  spi_tx_word(WRITE_RANGE_SELECTION);
  // Check, by reading back the register value.
  spi_tx_word(READ_RANGE_SEL_REG);
  data_word = spi_rx_word();

  return_reg_value = data_word >> 24; // spi_rx_buffer[0];
  if(return_reg_value != 0x08)
  {
	  GPIOA->BSRR = STATUS_LED_B_HIGH;
  }

  // Calibrate the sensors before starting the timer by calculating the cross-talk.
  // Cross-talk is fixed by the detector/emitter orientation and position on the pcb.
  // Note: changing the emitter current required recalculation.
  /*
  GPIOB->BSRR = STATUS_LED_RED_B_HIGH;
  calculate_crosstalk(crosstalk_values); // should take around 30 ms.
  GPIOB->BSRR = STATUS_LED_RED_B_LOW;
  */

  // Start the state machine.
  TIM3->CR1 |= TIM_CR1_CEN; // Start the timer.
  // Start the button capture timer 2.
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
  TIM2->CR1 |= TIM_CR1_CEN; // Start the timer.

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  switch(state)
	  {
	  case ST_IDLE:
		  asm("NOP");
		  break;

	  case ST_MEASURE: ;

		  // Check first, if a crosstalk recalculation is scheduled.
		  if(do_crosstalk_recalculation)
		  {
			  GPIOB->BSRR = STATUS_LED_RED_B_HIGH;
			  calculate_crosstalk(crosstalk_values); // should take around 30 ms.
			  GPIOB->BSRR = STATUS_LED_RED_B_LOW;
			  do_crosstalk_recalculation = 0;
		  }

		  // On every second button press, reset the crosstalk values.
		  if(reset_crosstalk_values)
		  {
			  GPIOB->BSRR = STATUS_LED_RED_B_HIGH;
			  for(int i = 0; i < NUM_SENSORS; i++)
			  {
				  crosstalk_values[i] = 0;
			  }
			  GPIOB->BSRR = STATUS_LED_RED_B_LOW;
			  reset_crosstalk_values = 0;
		  }

		  // Do OFFSET calculation prior to sensing until the current source can
		  // be enabled without zeroing out the PD response for a short time.

		  current_lsb = measure_sensors(offset_values, NUM_MEASUREMENTS_PER_SENSOR_DC_OFFSET);

		  // Read out the sensor values with the current source turned on.
		  GPIOA->BSRR = CURRENT_SOURCE_A_ON;
		  current_lsb = measure_sensors(sensor_values, NUM_MEASUREMENTS_PER_SENSOR);
		  // Measure the emitter current once for transmit before turning it off.

		  // current_lsb = get_emitter_current_lsb();
		  GPIOA->BSRR = CURRENT_SOURCE_A_OFF;

		  // Calculate the PD value-dependent small deviation in the crosstalk value.
		  uint32_t mean_offset_value = 0;
		  for(int sensor_index = 0; sensor_index < NUM_SENSORS; sensor_index++)
		  {
			  mean_offset_value += offset_values[sensor_index];
		  }
		  mean_offset_value = mean_offset_value/NUM_SENSORS;

		  // int16_t crosstalk_offset = 50 - (int16_t)(log((double)mean_offset_value/0.5189)/0.2027);
		  crosstalk_offset = 0; // TODO: crosstalk_offset is not yet compatible with single optode board

		  // Calculate the final sensor values (minus DC offset and cross-talk).
		  for(int sensor_index = 0; sensor_index < NUM_SENSORS; sensor_index++)
		  {
			  sensor_values_tx[sensor_index] = sensor_values[sensor_index]
											 - offset_values[sensor_index]
											 - crosstalk_values[sensor_index]
											 + crosstalk_offset;
		  }

		  state = ST_TRANSMIT;

	  case ST_TRANSMIT: ; // https://www.educative.io/edpresso/resolving-the-a-label-can-only-be-part-of-a-statement-error
		  // Construct the frame.
		  // (ADC module range register value) TODO IF NEEDED.

	  	  uint32_t byte_index = NUM_START_BYTES;

	  	  // **** Fill the frame buffer, all MSB first. ************
		  // 2 Alignment bits (already done).

	  	  // 32 bit frame buffer first.
	  	  frame_buffer[byte_index++] = frame_number >> 24;
	  	  frame_buffer[byte_index++] = frame_number >> 16;
	  	  frame_buffer[byte_index++] = frame_number >> 8;
	  	  frame_buffer[byte_index++] = frame_number;

	  	  // 16 bit emitter current as lsb value.
	  	  frame_buffer[byte_index++] = current_lsb >> 8;
	  	  frame_buffer[byte_index++] = current_lsb;

	  	  // 16 bits for every sensor value.
		  for(int sensor_index = 0; sensor_index < NUM_SENSORS; sensor_index++)
		  {
			  // Transmit MSB first.
			  frame_buffer[byte_index++] = sensor_values_tx[sensor_index] >> 8;
			  frame_buffer[byte_index++] = sensor_values_tx[sensor_index];
		  }

		  // ****************************************

		  uint32_t start_index = NUM_START_BYTES; // FOR DEBUGGING, MUST BE 2.
		  uint32_t stop_index = byte_index;       // FOR DEBUGGING, MUST BE FRAME_BUFFER_LENGTH_BYTES - 1
		  // Checksum.
		  uint8_t checksum = calculate_checksum(frame_buffer, start_index, stop_index);
		  frame_buffer[byte_index] = checksum;
		  status = CDC_Transmit_FS((uint8_t*)frame_buffer, FRAME_BUFFER_LENGTH_BYTES);
		  state = ST_IDLE;
		  break;

	  case ST_ERROR:
		  // Current out of range.
		  // Etc.
		  // Either check again here if error is lifted or never leave until reset.
		  // Currently, TIM3 will automatically toggle ST_MEASURE again.
		  break;
	  }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 48-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 48-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CS_ON_Pin|SPI1_MANUAL_CS_Pin|LED_OUT_A0_Pin|LED_IN_A1_Pin
                          |LED_IN_A0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, STATUS_LED_Pin|STATUS_LED_RED_Pin|PD_A1_Pin|PD_A0_Pin
                          |PD_Gates_A1_Pin|PD_Gates_A0_Pin|LED_OUT_A1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CS_ON_Pin SPI1_MANUAL_CS_Pin LED_OUT_A0_Pin LED_IN_A1_Pin
                           LED_IN_A0_Pin */
  GPIO_InitStruct.Pin = CS_ON_Pin|SPI1_MANUAL_CS_Pin|LED_OUT_A0_Pin|LED_IN_A1_Pin
                          |LED_IN_A0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : STATUS_LED_Pin STATUS_LED_RED_Pin PD_A1_Pin PD_A0_Pin
                           PD_Gates_A1_Pin PD_Gates_A0_Pin LED_OUT_A1_Pin */
  GPIO_InitStruct.Pin = STATUS_LED_Pin|STATUS_LED_RED_Pin|PD_A1_Pin|PD_A0_Pin
                          |PD_Gates_A1_Pin|PD_Gates_A0_Pin|LED_OUT_A1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	// Update the frame number and go to measure state.
	if(htim->Instance == TIM3)
	{
		frame_number++;
		// Blink to status led each time for checking purposes.
		if(frame_number%2 == 0)
		{
			GPIOB->BSRR = STATUS_LED_B_HIGH;
		}else
		{
			GPIOB->BSRR = STATUS_LED_B_LOW;
		}
		state = ST_MEASURE;
	}
}


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	{
	  crosstalk_fsm_counter++;
	  // Select action based on counter value.
	  if(crosstalk_fsm_counter == 1)
	  {
		  do_crosstalk_recalculation = 1;
	  }
	  if(crosstalk_fsm_counter == 2)
	  {
		  reset_crosstalk_values = 1;
	  }
	  // Only increment for [0,1,2].
	  if(crosstalk_fsm_counter >= 2)
	  {
		  crosstalk_fsm_counter = 0;
	  }
	}
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
