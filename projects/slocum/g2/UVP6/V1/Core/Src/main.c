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
#include "cmsis_os.h"
#include "app_fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "dbg.h"
#include "stdlib.h"
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
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c4;

UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_tx;

RTC_HandleTypeDef hrtc;

SD_HandleTypeDef hsd1;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for sensors_t */
osThreadId_t sensors_tHandle;
const osThreadAttr_t sensors_t_attributes = {
  .name = "sensors_t",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 256 * 4
};
/* Definitions for sensors_tmr */
osTimerId_t sensors_tmrHandle;
const osTimerAttr_t sensors_tmr_attributes = {
  .name = "sensors_tmr"
};
/* USER CODE BEGIN PV */

uint8_t keepalive_msg[]={0x34,0x38,0x33,0x31,0x09,0x36,0x34,0x30,0x09,0x31,0x38,0x35,0x2E,0x33,0x34,0x36,0x09,0x39,0x30,0x2E,0x30,0x36,0x32,0x09,0x32,0x36,0x2E,0x35,0x37,0x32,0x09,0x32,0x37,0x2E,0x35,0x39,0x34,0x09,0x33,0x30,0x2E,0x37,0x33,0x34,0x09,0x33,0x39,0x2E,0x30,0x36,0x36,0x09,0x38,0x2E,0x33,0x33,0x32,0x09,0x37,0x35,0x34,0x2E,0x37,0x09,0x38,0x31,0x34,0x2E,0x37,0x09,0x34,0x38,0x2E,0x37,0x0D,0x0A};

uint8_t  cmd_msg[100];
uint32_t cmd_msg_indx=0;

char tt[200];
RTC_TimeTypeDef sTimeStamp;
RTC_DateTypeDef sTimeStampDate;
uint8_t depth_cmd=0xac;
uint8_t OUTX_L_XL=0x28;
uint8_t OUTX_H_XL=0x29;
uint8_t OUTY_L_XL=0x2A;
uint8_t OUTY_H_XL=0x2B;
uint8_t OUTZ_L_XL=0x2C;
uint8_t OUTZ_H_XL=0x2D;
uint8_t CTRL1_XL[]={0x10,0x20};
uint8_t who_am_i=0x0f;
uint8_t id;
uint8_t data[6];
uint8_t rx_byte;
uint8_t rx_byte2;

FATFS fs;
FIL fl;
FRESULT res;

uint16_t P;
uint16_t T;
UINT btwritten;
uint32_t tmstmp;
int i=0;


enum{
  	FSM_STOPPED=0,
	FSM_STARTING,
	FSM_STARTED,
	FSM_STOPPING,
	FSM_TIMER,
	FSM_RTC_UPDATE
};
uint32_t fsm_status=FSM_STOPPED;
uint8_t uart2_rx_byte;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ICACHE_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_SDMMC1_SD_Init(void);
static void MX_I2C1_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C4_Init(void);
static void MX_USART3_UART_Init(void);
void StartDefaultTask(void *argument);
void sensors_f(void *argument);
void sensors_tmr_cb(void *argument);

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
  MX_DMA_Init();
  MX_ICACHE_Init();
  MX_USART1_UART_Init();
  MX_RTC_Init();
  MX_SDMMC1_SD_Init();
  if (MX_FATFS_Init() != APP_OK) {
    Error_Handler();
  }
  MX_I2C1_Init();
  MX_LPUART1_UART_Init();
  MX_USART2_UART_Init();
  MX_I2C4_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */


  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of sensors_tmr */
  sensors_tmrHandle = osTimerNew(sensors_tmr_cb, osTimerPeriodic, NULL, &sensors_tmr_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of sensors_t */
  sensors_tHandle = osThreadNew(sensors_f, NULL, &sensors_t_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	/*
	HAL_UART_Transmit_DMA(&huart1, "Hello\n", 6);

	HAL_Delay(1000);
	//HAL_RTCEx_GetTimeStamp(&hrtc, &sTimeStamp, &sTimeStampDate, RTC_FORMAT_BIN);
	HAL_RTC_GetTime(&hrtc, &sTimeStamp, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &sTimeStampDate, RTC_FORMAT_BIN);
	sprintf(tt,"%u:%u:%u\n",sTimeStamp.Hours,sTimeStamp.Minutes,sTimeStamp.Seconds);
	HAL_UART_Transmit_DMA(&huart1, tt, strlen(tt));
	HAL_Delay(1000);
	*/

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE0) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 18;
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x40D04660;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C4_Init(void)
{

  /* USER CODE BEGIN I2C4_Init 0 */

  /* USER CODE END I2C4_Init 0 */

  /* USER CODE BEGIN I2C4_Init 1 */

  /* USER CODE END I2C4_Init 1 */
  hi2c4.Instance = I2C4;
  hi2c4.Init.Timing = 0x10F0B6FF;
  hi2c4.Init.OwnAddress1 = 0;
  hi2c4.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c4.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c4.Init.OwnAddress2 = 0;
  hi2c4.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c4.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c4.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c4, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c4, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C4_Init 2 */

  /* USER CODE END I2C4_Init 2 */

}

/**
  * @brief ICACHE Initialization Function
  * @param None
  * @retval None
  */
static void MX_ICACHE_Init(void)
{

  /* USER CODE BEGIN ICACHE_Init 0 */

  /* USER CODE END ICACHE_Init 0 */

  /* USER CODE BEGIN ICACHE_Init 1 */

  /* USER CODE END ICACHE_Init 1 */

  /** Enable instruction cache (default 2-ways set associative cache)
  */
  if (HAL_ICACHE_Enable() != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ICACHE_Init 2 */

  /* USER CODE END ICACHE_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  hlpuart1.FifoMode = UART_FIFOMODE_DISABLE;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_PrivilegeStateTypeDef privilegeState = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  privilegeState.rtcPrivilegeFull = RTC_PRIVILEGE_FULL_NO;
  privilegeState.backupRegisterPrivZone = RTC_PRIVILEGE_BKUP_ZONE_NONE;
  privilegeState.backupRegisterStartZone2 = RTC_BKP_DR0;
  privilegeState.backupRegisterStartZone3 = RTC_BKP_DR0;
  if (HAL_RTCEx_PrivilegeModeSet(&hrtc, &privilegeState) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SDMMC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDMMC1_SD_Init(void)
{

  /* USER CODE BEGIN SDMMC1_Init 0 */

  /* USER CODE END SDMMC1_Init 0 */

  /* USER CODE BEGIN SDMMC1_Init 1 */

  /* USER CODE END SDMMC1_Init 1 */
  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_4B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd1.Init.ClockDiv = 0;
  /* USER CODE BEGIN SDMMC1_Init 2 */


  /* USER CODE END SDMMC1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PWR_MCU_PROG_GPIO_Port, PWR_MCU_PROG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PWR_MCU_PROG_Pin */
  GPIO_InitStruct.Pin = PWR_MCU_PROG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PWR_MCU_PROG_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PE10 PE11 PE12 PE13
                           PE14 PE15 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OCTOSPI1;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
uint32_t sd_dma_rx_semaphore;
uint32_t sd_dma_tx_semaphore;

void HAL_SD_TxCpltCallback(SD_HandleTypeDef *hsd)
{
	sd_dma_tx_semaphore=1;
	//print_param("tcb",hsd->ErrorCode);
}


void HAL_SD_RxCpltCallback(SD_HandleTypeDef *hsd)
{
	sd_dma_rx_semaphore=1;
	//print_param("rCb",hsd->ErrorCode);

}
void HAL_SD_ErrorCallback(SD_HandleTypeDef *hsd)
{
  //print_param("ecb",hsd->ErrorCode);
  //print_param("STAecb",hsd1.Instance->STA);
  //print_param("CLKCR",hsd1.Instance->CLKCR);
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
 if(huart==&huart2)
 {
	 if(uart2_rx_byte=='\r'){
		 uart2_rx_byte=0x00;
	 }

     cmd_msg[cmd_msg_indx]=uart2_rx_byte;
     if(uart2_rx_byte==0x00){
    	 cmd_msg_indx=0;
    	 if(fsm_status==FSM_STARTED||fsm_status==FSM_TIMER)fsm_status=FSM_RTC_UPDATE;
     }
     else{
    	 cmd_msg_indx++;
    	 cmd_msg_indx%=100;
     }

	 HAL_UART_Receive_IT(&huart2,&uart2_rx_byte,1);
 }
}


/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  cmd_msg_indx=0;
  HAL_UART_Receive_IT(&huart2,&uart2_rx_byte,1);

  for(;;)
  {
	HAL_UART_Transmit(&huart2, keepalive_msg, 76, 100);
    osDelay(1000);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_sensors_f */
/**
* @brief Function implementing the sensors_t thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_sensors_f */
void sensors_f(void *argument)
{
  /* USER CODE BEGIN sensors_f */
  /* Infinite loop */

	  osDelay(200);
	  HAL_UART_Transmit(&huart1, "Start\n", 6, 100);

	  res=f_mount(&fs,"",1);
	  print_param("mount", res);
	  HAL_I2C_Master_Transmit(&hi2c4, 0xd4, CTRL1_XL, 2, 100);

	  fsm_status=FSM_STARTING;
	  char* pch;
	  while(1){
		 switch(fsm_status){
		   case FSM_RTC_UPDATE:
			   // d.m.y ss:mm:hh weekday
               print_str(cmd_msg);
			   pch = strtok (cmd_msg,"."); //Date
			   sTimeStampDate.Date=strtoul(pch,NULL,0);
			   pch = strtok (NULL,".");    //Month
			   sTimeStampDate.Month=strtoul(pch,NULL,0);
			   pch = strtok (NULL," ");    //Year
			   sTimeStampDate.Year=strtoul(pch,NULL,0);

			   pch = strtok (NULL,":"); //Seconds
			   sTimeStamp.Seconds=strtoul(pch,NULL,0);
			   pch = strtok (NULL,":");    //Minutes
			   sTimeStamp.Minutes=strtoul(pch,NULL,0);
			   pch = strtok (NULL," ");    //Hours
			   sTimeStamp.Hours=strtoul(pch,NULL,0);

			   pch = strtok (NULL," ");    //Weekday
			   sTimeStampDate.WeekDay =strtoul(pch,NULL,0);

			   HAL_RTC_SetTime(&hrtc, &sTimeStamp, RTC_FORMAT_BIN);
			   HAL_RTC_SetDate(&hrtc, &sTimeStampDate, RTC_FORMAT_BIN);

			   fsm_status=FSM_STARTED;

		   break;
		   case FSM_STARTING:
			HAL_RTC_GetTime(&hrtc, &sTimeStamp, RTC_FORMAT_BIN);
			HAL_RTC_GetDate(&hrtc, &sTimeStampDate, RTC_FORMAT_BIN);

			sprintf(tt,"data_%02u%02u%02u%02u%02u%02u.txt",sTimeStampDate.Date,sTimeStampDate.Month,sTimeStampDate.Year,sTimeStamp.Hours,sTimeStamp.Minutes,sTimeStamp.Seconds);


			f_open(&fl,tt,FA_OPEN_APPEND|FA_WRITE);
			f_write(&fl,"Start\n",6,&btwritten);
			osTimerStart(sensors_tmrHandle, 100);
			fsm_status=FSM_STARTED;
		   break;
		   case FSM_TIMER:
		    HAL_RTC_GetTime(&hrtc, &sTimeStamp, RTC_FORMAT_BIN);
		    HAL_RTC_GetDate(&hrtc, &sTimeStampDate, RTC_FORMAT_BIN);
		    uint32_t subsec=(sTimeStamp.SecondFraction-sTimeStamp.SubSeconds)*1000/(sTimeStamp.SecondFraction+1);

		    //sprintf(tt,"RTC %02u:%02u:%02u %02u:%02u:%02u\n",sTimeStampDate.Date,sTimeStampDate.Month,sTimeStampDate.Year,sTimeStamp.Hours,sTimeStamp.Minutes,sTimeStamp.Seconds);

			//IMU SENSOR!!
			//HAL_I2C_Master_Transmit(&hi2c4, 0xd4, &who_am_i, 1, 100);
			//HAL_I2C_Master_Receive(&hi2c4, 0xd4, &id, 1, 100);
			//sprintf(tt,"who_am_i:%u \n",id);
			//HAL_UART_Transmit(&huart1, tt, strlen(tt),100);

			HAL_I2C_Master_Transmit(&hi2c4, 0xd4, &OUTX_L_XL, 1, 100);
			HAL_I2C_Master_Receive(&hi2c4, 0xd4, data, 1, 100);
			HAL_I2C_Master_Transmit(&hi2c4, 0xd4, &OUTX_H_XL, 1, 100);
			HAL_I2C_Master_Receive(&hi2c4, 0xd4, data+1, 1, 100);

			HAL_I2C_Master_Transmit(&hi2c4, 0xd4, &OUTY_L_XL, 1, 100);
			HAL_I2C_Master_Receive(&hi2c4, 0xd4, data+2, 1, 100);
			HAL_I2C_Master_Transmit(&hi2c4, 0xd4, &OUTY_H_XL, 1, 100);
			HAL_I2C_Master_Receive(&hi2c4, 0xd4, data+3, 1, 100);

			HAL_I2C_Master_Transmit(&hi2c4, 0xd4, &OUTZ_L_XL, 1, 100);
			HAL_I2C_Master_Receive(&hi2c4, 0xd4, data+4, 1, 100);
			HAL_I2C_Master_Transmit(&hi2c4, 0xd4, &OUTZ_H_XL, 1, 100);
			HAL_I2C_Master_Receive(&hi2c4, 0xd4, data+5, 1, 100);

			int16_t OUTX_XL=(data[1]<<8)|data[0];
			int16_t OUTY_XL=(data[3]<<8)|data[2];
			int16_t OUTZ_XL=(data[5]<<8)|data[4];


			//DEPTH SENSOR !!
			HAL_I2C_Master_Transmit(&hi2c1, 0x80, &depth_cmd, 1, 100);
			osDelay(15);
			HAL_I2C_Master_Receive(&hi2c1, 0x80, data, 5, 100);
			P=(data[1]<<8)|data[2];
			T=(data[3]<<8)|data[4];

			sprintf(tt,"%02u:%02u:%02u,%02u:%02u:%02u.%03u,%u,%u,%d,%d,%d\n",sTimeStampDate.Date,sTimeStampDate.Month,sTimeStampDate.Year,sTimeStamp.Hours,sTimeStamp.Minutes,sTimeStamp.Seconds,subsec,P,T,OUTX_XL,OUTY_XL,OUTZ_XL);

			res=f_write(&fl,tt,strlen(tt),&btwritten);
			osDelay(20);
			f_sync(&fl);
			osDelay(20);
			print_str(tt);
			if(fsm_status!=FSM_RTC_UPDATE)fsm_status=FSM_STARTED;
		   break;
		   case FSM_STOPPING:
			osTimerStop(sensors_tmrHandle);
			f_close(&fl);
			osDelay(20);
			print_str("Finished\n");
			fsm_status=FSM_STOPPED;
		   break;
		   case FSM_STOPPED:

		   break;
		  }


		  osDelay(1);
	  }



  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END sensors_f */
}

/* sensors_tmr_cb function */
void sensors_tmr_cb(void *argument)
{
  /* USER CODE BEGIN sensors_tmr_cb */
   if(fsm_status==FSM_STARTED) fsm_status=FSM_TIMER;

  /* USER CODE END sensors_tmr_cb */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
	HAL_UART_Transmit(&huart1, "err", 3,100);
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
