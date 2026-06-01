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
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CAN_MESSAGE_ID          0x433U
#define CAN_SELF_TEST_ID        0x55AU
#define CAN_BAUD_RATE          250000U
#define BUTTON_DEBOUNCE_MS     200U
#define CAN_SELF_TEST_TIMEOUT_MS 200U
#define CAN_TX_TIMEOUT_MS      1000U
#define UART_TX_TIMEOUT_MS     100U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static CAN_TxHeaderTypeDef TxHeader;
static CAN_RxHeaderTypeDef RxHeader;
static uint8_t TxData[8] = { 'H', 'W', '1', '2', 'L', '4', '7', '6' };
static uint8_t SelfTestData[8] = { 'L', 'O', 'O', 'P', 'B', 'A', 'C', 'K' };
static uint8_t RxData[8];
static uint32_t TxMailbox;
static uint8_t TxPending;
static uint32_t TxStartTick;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_CAN1_Init(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
static void CAN_InitForMode(uint32_t mode);
static void CAN_ConfigFilter(void);
static void CAN_ConfigNormalMode(void);
static uint8_t CAN_InternalLoopbackSelfTest(void);
static void CAN_PollReceive(void);
static void CAN_SendButtonMessage(void);
static void CAN_ServiceTxStatus(void);
static uint8_t ButtonPressed(void);
static void UART_Print(const char *message);
static void UART_PrintFrame(const char *prefix, const CAN_RxHeaderTypeDef *header, const uint8_t *data);
static void UART_PrintTxFrame(const char *prefix, const CAN_TxHeaderTypeDef *header, const uint8_t *data);

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
  MX_USART2_UART_Init();
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */
  if (CAN_InternalLoopbackSelfTest() != 0U)
  {
    UART_Print("CAN INTERNAL LOOPBACK PASS\r\n");
  }
  else
  {
    UART_Print("CAN INTERNAL LOOPBACK FAIL\r\n");
  }

  CAN_ConfigNormalMode();
  UART_Print("\r\nHW12 CAN ready: CAN1 PB8/PB9, 250 kbit/s, ID 0x433\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    CAN_PollReceive();
    CAN_ServiceTxStatus();

    if (ButtonPressed())
    {
      CAN_SendButtonMessage();
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  CAN_InitForMode(CAN_MODE_NORMAL);
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

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
static void CAN_InitForMode(uint32_t mode)
{
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 20;
  hcan1.Init.Mode = mode;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;

  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void CAN_ConfigFilter(void)
{
  CAN_FilterTypeDef filter = {0};

  filter.FilterBank = 0;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterIdHigh = 0x0000;
  filter.FilterIdLow = 0x0000;
  filter.FilterMaskIdHigh = 0x0000;
  filter.FilterMaskIdLow = 0x0000;
  filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  filter.FilterActivation = ENABLE;
  filter.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK)
  {
    Error_Handler();
  }
}

static void CAN_ConfigNormalMode(void)
{
  HAL_CAN_DeInit(&hcan1);
  CAN_InitForMode(CAN_MODE_NORMAL);
  CAN_ConfigFilter();

  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }

  TxHeader.StdId = CAN_MESSAGE_ID;
  TxHeader.ExtId = 0;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.DLC = sizeof(TxData);
  TxHeader.TransmitGlobalTime = DISABLE;
  TxPending = 0U;
}

static uint8_t CAN_InternalLoopbackSelfTest(void)
{
  CAN_TxHeaderTypeDef selfTxHeader = {0};
  CAN_RxHeaderTypeDef selfRxHeader = {0};
  uint8_t selfRxData[8] = {0};
  uint32_t selfTxMailbox = 0U;
  uint32_t startTick = 0U;
  uint8_t passed = 0U;

  HAL_CAN_DeInit(&hcan1);
  CAN_InitForMode(CAN_MODE_LOOPBACK);
  CAN_ConfigFilter();

  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
    return 0U;
  }

  selfTxHeader.StdId = CAN_SELF_TEST_ID;
  selfTxHeader.ExtId = 0U;
  selfTxHeader.IDE = CAN_ID_STD;
  selfTxHeader.RTR = CAN_RTR_DATA;
  selfTxHeader.DLC = sizeof(SelfTestData);
  selfTxHeader.TransmitGlobalTime = DISABLE;

  if (HAL_CAN_AddTxMessage(&hcan1, &selfTxHeader, SelfTestData, &selfTxMailbox) == HAL_OK)
  {
    startTick = HAL_GetTick();
    while ((HAL_GetTick() - startTick) < CAN_SELF_TEST_TIMEOUT_MS)
    {
      if (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0U)
      {
        if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &selfRxHeader, selfRxData) == HAL_OK)
        {
          passed = ((selfRxHeader.StdId == CAN_SELF_TEST_ID) &&
                    (selfRxHeader.IDE == CAN_ID_STD) &&
                    (selfRxHeader.DLC == sizeof(SelfTestData)) &&
                    (memcmp(selfRxData, SelfTestData, sizeof(SelfTestData)) == 0)) ? 1U : 0U;
        }
        break;
      }
    }

    if (HAL_CAN_IsTxMessagePending(&hcan1, selfTxMailbox) != 0U)
    {
      HAL_CAN_AbortTxRequest(&hcan1, selfTxMailbox);
    }
  }

  HAL_CAN_Stop(&hcan1);
  HAL_CAN_DeInit(&hcan1);
  HAL_CAN_ResetError(&hcan1);

  return passed;
}

static void CAN_PollReceive(void)
{
  while (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0U)
  {
    if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
    {
      UART_Print("CAN RX error\r\n");
      return;
    }

    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    UART_PrintFrame("RX", &RxHeader, RxData);
  }
}

static void CAN_SendButtonMessage(void)
{
  if (TxPending != 0U)
  {
    UART_Print("TX still pending; waiting for CAN ACK\r\n");
    return;
  }

  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0U)
  {
    UART_Print("CAN TX mailbox full\r\n");
    return;
  }

  if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK)
  {
    UART_Print("CAN TX request failed\r\n");
    HAL_CAN_ResetError(&hcan1);
    return;
  }

  TxPending = 1U;
  TxStartTick = HAL_GetTick();
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
  UART_PrintTxFrame("TX", &TxHeader, TxData);
}

static void CAN_ServiceTxStatus(void)
{
  char line[64];

  if (TxPending == 0U)
  {
    return;
  }

  if (HAL_CAN_IsTxMessagePending(&hcan1, TxMailbox) == 0U)
  {
    uint32_t error = HAL_CAN_GetError(&hcan1);

    TxPending = 0U;
    if (error == HAL_CAN_ERROR_NONE)
    {
      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
      UART_Print("TX complete\r\n");
    }
    else
    {
      snprintf(line, sizeof(line), "CAN TX error: 0x%08lX\r\n", (unsigned long)error);
      UART_Print(line);
      HAL_CAN_ResetError(&hcan1);
    }
    return;
  }

  if ((HAL_GetTick() - TxStartTick) > CAN_TX_TIMEOUT_MS)
  {
    HAL_CAN_AbortTxRequest(&hcan1, TxMailbox);
    TxPending = 0U;
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
    UART_Print("CAN TX timeout; check bus wiring, termination, and another active node\r\n");
    HAL_CAN_ResetError(&hcan1);
  }
}

static uint8_t ButtonPressed(void)
{
  static GPIO_PinState lastState = GPIO_PIN_SET;
  static uint32_t lastPressTick = 0U;
  GPIO_PinState state = HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin);
  uint32_t now = HAL_GetTick();
  uint8_t pressed = 0U;

  if ((lastState == GPIO_PIN_SET) && (state == GPIO_PIN_RESET) &&
      ((now - lastPressTick) > BUTTON_DEBOUNCE_MS))
  {
    pressed = 1U;
    lastPressTick = now;
  }

  lastState = state;
  return pressed;
}

static void UART_Print(const char *message)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)message, (uint16_t)strlen(message), UART_TX_TIMEOUT_MS);
}

static void UART_PrintFrame(const char *prefix, const CAN_RxHeaderTypeDef *header, const uint8_t *data)
{
  char line[128];
  int len = snprintf(line, sizeof(line),
                     "%s ID=0x%03lX DLC=%lu DATA=%02X %02X %02X %02X %02X %02X %02X %02X\r\n",
                     prefix,
                     (unsigned long)header->StdId,
                     (unsigned long)header->DLC,
                     data[0], data[1], data[2], data[3],
                     data[4], data[5], data[6], data[7]);

  if (len > 0)
  {
    HAL_UART_Transmit(&huart2, (uint8_t *)line, (uint16_t)len, UART_TX_TIMEOUT_MS);
  }
}

static void UART_PrintTxFrame(const char *prefix, const CAN_TxHeaderTypeDef *header, const uint8_t *data)
{
  CAN_RxHeaderTypeDef printHeader = {0};

  printHeader.StdId = header->StdId;
  printHeader.DLC = header->DLC;
  UART_PrintFrame(prefix, &printHeader, data);
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
