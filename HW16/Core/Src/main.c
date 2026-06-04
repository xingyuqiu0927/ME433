/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define INA219_ADDR 0x40
#define INA219_REG_CONFIG 0x00
#define INA219_REG_SHUNT_VOLTAGE 0x01
#define INA219_REG_BUS_VOLTAGE 0x02
#define INA219_REG_POWER 0x03
#define INA219_REG_CURRENT 0x04
#define INA219_REG_CALIBRATION 0x05

#define ADC_SAFE_MIN 250U
#define ADC_SAFE_MAX (4095U - ADC_SAFE_MIN)
#define PWM_PERIOD_COUNTS 2399U
#define PWM_OFF_COUNTS (PWM_PERIOD_COUNTS + 1U)
#define CONTROL_SAMPLES 800U
#define CONTROL_SEGMENT_SAMPLES 200U
#define DESIRED_CURRENT_RAW 450
#define CURRENT_KP 2.0f
#define CURRENT_KI 0.06f
#define CURRENT_FILTER_ALPHA 0.08f
#define CURRENT_INTEGRAL_LIMIT 30000.0f
#define CURRENT_CONTROL_LIMIT 2200.0f
#define ADC_LOW_START_THRESHOLD 400U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

COM_InitTypeDef BspCOMInit;

/* USER CODE BEGIN PV */
volatile uint8_t state = 0;
static volatile uint8_t ina219_ok = 1;
static volatile uint16_t log_count = 0;
static volatile uint16_t adc_log[CONTROL_SAMPLES];
static volatile int16_t desired_current_log[CONTROL_SAMPLES];
static volatile int16_t actual_current_log[CONTROL_SAMPLES];
static volatile int16_t pwm_log[CONTROL_SAMPLES];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_ADC1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
uint32_t read_adc(void);
void init_ina219(void);
float read_ina219(void);
void writeINA219(int reg, int value);
signed short readINA219(unsigned char reg);
int __io_getchar(void);
static void motor_off(void);
static void set_motor_pwm(float control);
static void print_control_log(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern UART_HandleTypeDef hcom_uart[];

uint32_t read_adc(void)
{
  uint32_t raw;

  HAL_ADC_Start(&hadc1);
  if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
  {
    raw = HAL_ADC_GetValue(&hadc1);
  }
  else
  {
    raw = 0;
  }
  HAL_ADC_Stop(&hadc1);

  return raw;
}

void init_ina219(void)
{
  unsigned short ina219_calValue = 1024;
  unsigned short ina219_config = 0b0011000010001111;

  writeINA219(INA219_REG_CALIBRATION, ina219_calValue);
  writeINA219(INA219_REG_CONFIG, ina219_config);
}

float read_ina219(void)
{
  float ma = 0;
  signed short value = readINA219(INA219_REG_CURRENT);

  ma = value / 3.0f;
  return ma;
}

void writeINA219(int reg, int value)
{
  uint8_t buf[3];

  buf[0] = reg;
  buf[1] = value >> 8;
  buf[2] = value & 0xff;

  if (HAL_I2C_Master_Transmit(&hi2c2, INA219_ADDR << 1, buf, 3, 10) != HAL_OK)
  {
    ina219_ok = 0;
  }
}

signed short readINA219(unsigned char reg)
{
  uint8_t buffer[2] = {0};

  if ((HAL_I2C_Master_Transmit(&hi2c2, INA219_ADDR << 1, &reg, 1, 10) != HAL_OK) ||
      (HAL_I2C_Master_Receive(&hi2c2, INA219_ADDR << 1, buffer, 2, 10) != HAL_OK))
  {
    ina219_ok = 0;
    return 0;
  }

  ina219_ok = 1;
  return (signed short)((buffer[0] << 8) | buffer[1]);
}

static void motor_off(void)
{
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWM_OFF_COUNTS);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, PWM_OFF_COUNTS);
}

static void set_motor_pwm(float control)
{
  uint32_t magnitude;
  uint32_t drive_compare;

  if (control > PWM_OFF_COUNTS)
  {
    control = PWM_OFF_COUNTS;
  }
  else if (control < -(float)PWM_OFF_COUNTS)
  {
    control = -(float)PWM_OFF_COUNTS;
  }

  if (control > 0.0f)
  {
    magnitude = (uint32_t)control;
    drive_compare = PWM_OFF_COUNTS - magnitude;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWM_OFF_COUNTS);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, drive_compare);
  }
  else if (control < 0.0f)
  {
    magnitude = (uint32_t)(-control);
    drive_compare = PWM_OFF_COUNTS - magnitude;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, PWM_OFF_COUNTS);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, drive_compare);
  }
  else
  {
    motor_off();
  }
}

static void print_control_log(void)
{
  uint16_t count = log_count;

  printf("run_complete samples=%u limits=%u..%u\r\n", count, ADC_SAFE_MIN, ADC_SAFE_MAX);
  printf("idx,desired_ma3,current_ma3,pwm_cmd,adc\r\n");
  for (uint16_t i = 0; i < count; i++)
  {
    printf("%u,%d,%d,%d,%u\r\n",
           i,
           desired_current_log[i],
           actual_current_log[i],
           pwm_log[i],
           adc_log[i]);
  }
}

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
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE END 2 */

  /* Initialize leds */
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_BLUE);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  printf("\r\nHW16 ready. Send 'a' to run current control.\r\n");

  init_ina219();
  motor_off();
  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    int ch = __io_getchar();
    if ((ch == 'a') || (ch == 'A'))
    {
      log_count = 0;
      state = 1;
      while (state == 1)
      {
      }
      print_control_log();
      printf("ready\r\n");
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

  __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_0);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV4;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  ADC_ChannelConfTypeDef sConfig = {0};
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_ADC_CLK_ENABLE();

  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_SEQ_FIXED;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_I2C2_CLK_ENABLE();

  GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF6_I2C2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00402D41;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{
  TIM_OC_InitTypeDef sConfigOC = {0};
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_TIM1_CLK_ENABLE();

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = PWM_PERIOD_COUNTS;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Alternate = GPIO_AF5_TIM1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{
  __HAL_RCC_TIM2_CLK_ENABLE();

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 11;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_NVIC_SetPriority(TIM2_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  static volatile uint16_t counter = 0;
  static float current_error_integral = 0.0f;
  static float filtered_current = 0.0f;
  static int16_t run_direction = 1;
  uint32_t adc;

  if (htim == &htim2)
  {
    adc = read_adc();

    if ((adc < ADC_SAFE_MIN) || (adc > ADC_SAFE_MAX))
    {
      motor_off();
      state = 0;
      log_count = counter;
      counter = 0;
      current_error_integral = 0.0f;
      filtered_current = 0.0f;
      return;
    }

    if (state == 1)
    {
      int16_t desired_current;
      int16_t raw_current;
      int16_t actual_current;
      int16_t pwm_command;
      int32_t current_error;
      float control;

      if (counter == 0)
      {
        run_direction = (adc < ADC_LOW_START_THRESHOLD) ? -1 : 1;
        current_error_integral = 0.0f;
        filtered_current = 0.0f;
      }
      else if ((counter % CONTROL_SEGMENT_SAMPLES) == 0U)
      {
        current_error_integral = 0.0f;
      }

      if (((counter / CONTROL_SEGMENT_SAMPLES) % 2U) == 0U)
      {
        desired_current = run_direction * DESIRED_CURRENT_RAW;
      }
      else
      {
        desired_current = -run_direction * DESIRED_CURRENT_RAW;
      }

      raw_current = readINA219(INA219_REG_CURRENT);
      if (ina219_ok == 0)
      {
        motor_off();
        state = 0;
        log_count = counter;
        counter = 0;
        current_error_integral = 0.0f;
        filtered_current = 0.0f;
        return;
      }

      filtered_current += CURRENT_FILTER_ALPHA * ((float)raw_current - filtered_current);
      actual_current = (int16_t)filtered_current;
      current_error = desired_current - actual_current;
      current_error_integral += (float)current_error;
      if (current_error_integral > CURRENT_INTEGRAL_LIMIT)
      {
        current_error_integral = CURRENT_INTEGRAL_LIMIT;
      }
      else if (current_error_integral < -CURRENT_INTEGRAL_LIMIT)
      {
        current_error_integral = -CURRENT_INTEGRAL_LIMIT;
      }
      control = (CURRENT_KP * (float)current_error) + (CURRENT_KI * current_error_integral);
      if (control > CURRENT_CONTROL_LIMIT)
      {
        control = CURRENT_CONTROL_LIMIT;
      }
      else if (control < -CURRENT_CONTROL_LIMIT)
      {
        control = -CURRENT_CONTROL_LIMIT;
      }
      pwm_command = (int16_t)control;
      set_motor_pwm((float)pwm_command);

      if (counter < CONTROL_SAMPLES)
      {
        adc_log[counter] = adc;
        desired_current_log[counter] = desired_current;
        actual_current_log[counter] = actual_current;
        pwm_log[counter] = pwm_command;
      }

      counter++;
      log_count = counter;

      if (counter >= CONTROL_SAMPLES)
      {
        motor_off();
        state = 0;
        counter = 0;
        current_error_integral = 0.0f;
        filtered_current = 0.0f;
      }
    }
  }
}

int __io_getchar(void)
{
  uint8_t ch = 0;

  if (HAL_UART_Receive(&hcom_uart[COM1], &ch, 1, HAL_MAX_DELAY) != HAL_OK)
  {
    return -1;
  }

  return ch;
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
#ifdef USE_FULL_ASSERT
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
