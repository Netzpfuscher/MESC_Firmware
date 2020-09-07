/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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

#include "cmsis_os.h"
#include "usb_device.h"

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
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;
DMA_HandleTypeDef hdma_adc3;

COMP_HandleTypeDef hcomp1;
COMP_HandleTypeDef hcomp2;
COMP_HandleTypeDef hcomp4;
COMP_HandleTypeDef hcomp7;

I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;

OPAMP_HandleTypeDef hopamp1;
OPAMP_HandleTypeDef hopamp2;
OPAMP_HandleTypeDef hopamp3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .priority = (osPriority_t)osPriorityNormal,
    .stack_size = 128 * 4};
/* Definitions for SlowLoopTask */
osThreadId_t SlowLoopTaskHandle;
const osThreadAttr_t SlowLoopTask_attributes = {
    .name = "SlowLoopTask",
    .priority = (osPriority_t)osPriorityLow,
    .stack_size = 128 * 4};
/* Definitions for ComsTask */
osThreadId_t ComsTaskHandle;
const osThreadAttr_t ComsTask_attributes = {
    .name = "ComsTask",
    .priority = (osPriority_t)osPriorityLow,
    .stack_size = 128 * 4};
/* Definitions for BatCheckTask */
osThreadId_t BatCheckTaskHandle;
const osThreadAttr_t BatCheckTask_attributes = {
    .name = "BatCheckTask",
    .priority = (osPriority_t)osPriorityLow,
    .stack_size = 128 * 4};
/* USER CODE BEGIN PV */
uint16_t a = 0;
float adcBuff1[3] = {0, 0, 0};
uint32_t adcBuff2[3] = {0, 0, 0};
uint32_t adcBuff3[3] = {0, 0, 0};
uint32_t ICVals[2] = {0, 0};
uint32_t RegBuff = 0;
uint32_t quickHall = 0;
int initing = 1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_COMP1_Init(void);
static void MX_COMP2_Init(void);
static void MX_COMP4_Init(void);
static void MX_COMP7_Init(void);
static void MX_I2C1_Init(void);
static void MX_OPAMP1_Init(void);
static void MX_OPAMP2_Init(void);
static void MX_OPAMP3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART3_UART_Init(void);
void StartDefaultTask(void *argument);
void SlowLoopEntry(void *argument);
void ComsTaskEntry(void *argument);
void BatCheck(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// fixme: this function should not be here. It should be in a separate file or
// at least marked explicitly what it does and why.
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
        ICVals[0] = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_1);

        // Target is 20000 guard is +-10000
        if ((ICVals[0] < 10000) || (30000 < ICVals[0])) {
            a = 0;
            BLDCVars.ReqCurrent = 0;
        }

        else if (ICVals[0] != 0) {
            BLDCState = BLDC_FORWARDS;
            ICVals[1] = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_2);
            if (ICVals[1] > 2000) ICVals[1] = 2000;
            if (ICVals[1] < 1000) ICVals[1] = 1000;

            // Mid-point is 1500 guard is +-100
            if ((ICVals[1] > 1400) && (1600 > ICVals[1])) {
                ICVals[1] = 1500;
            }
            // Set the current setpoint here
            if (1) {  // Current control, ToDo convert to Enum
                if (ICVals[1] > 1600)
                    BLDCVars.ReqCurrent =
                        ((float)ICVals[1] - 1600) /
                        5.0;  // Crude hack, which gets current scaled to +/-80A
                              // based on 1000-2000us PWM in
                else if (ICVals[1] < 1400)
                    BLDCVars.ReqCurrent =
                        ((float)ICVals[1] - 1400) /
                        5.0;  // Crude hack, which gets current scaled to +/-80A
                              // based on 1000-2000us PWM in
                else
                    BLDCVars.ReqCurrent = 0;
            }

            if (0) {  // Duty cycle control, ToDo convert to Enum
                if (a < 10) {
                    BLDCVars.BLDCduty = 0;
                }
                if (a > 9) {
                    BLDCVars.BLDCduty = 10 * (a - 9);
                }
            }
        }
        // todo: remove dead code.
        /////////////////////////////////
        /*		if(ICVals[0]!=0){
                                ICVals[1]=HAL_TIM_ReadCapturedValue(&htim3,
           TIM_CHANNEL_2); if(ICVals[1]>1500){ BLDCState=BLDC_FORWARDS;
                                        a=100*(ICVals[1]-1500)/1500;
                                }
                                else if(ICVals[1]<1500){
                                        BLDCState=BLDC_FORWARDS;
                                        a=100*(1500-ICVals[1])/1500;
                                }
                        }*/
    }
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU
     * Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the
     * Systick. */
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
    MX_ADC1_Init();
    MX_ADC2_Init();
    MX_ADC3_Init();
    MX_COMP1_Init();
    MX_COMP2_Init();
    MX_COMP4_Init();
    MX_COMP7_Init();
    MX_I2C1_Init();
    MX_OPAMP1_Init();
    MX_OPAMP2_Init();
    MX_OPAMP3_Init();
    MX_TIM1_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_USART3_UART_Init();
    /* USER CODE BEGIN 2 */
    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
    // Place to mess about with PWM in

    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc3, ADC_SINGLE_ENDED);
    HAL_Delay(3000);
    // fixme: what does this do? needs explanation.
    quickHall = (GPIOB->IDR >> 6) & 0x7;

    /* Trying to fix this pernicious Opamp offset, implementing the calibration
    routine apparently only works if the opamp is not in PGA mode.
     * Sadly, it makes no damned difference, leaving this code here since it
    possibly should be included regardless. hopamp1.Init.Mode =
    OPAMP_STANDALONE_MODE ; HAL_OPAMP_Init(&hopamp1);
    HAL_OPAMP_SelfCalibrate(&hopamp1);
    HAL_Delay(50);
    hopamp1.Init.Mode = OPAMP_PGA_MODE;
    HAL_OPAMP_Init(&hopamp1);
  */

    HAL_Delay(50);
    HAL_OPAMP_Start(&hopamp1);
    HAL_OPAMP_Start(&hopamp2);
    HAL_OPAMP_Start(&hopamp3);

    // fixme: this portion of the code requires explanation of what is
    // happening.
    motor_init();
    hw_init();
    motor.Rphase = 0.1;
    BLDCInit();
    measurement_buffers.ADCOffset[0] = 1900;
    measurement_buffers.ADCOffset[1] = 1900;
    measurement_buffers.ADCOffset[2] = 1900;

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    __HAL_TIM_SET_COUNTER(&htim1, 10);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    __HAL_TIM_SET_COUNTER(&htim1, 10);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
    __HAL_TIM_SET_COUNTER(&htim1, 10);
    /*
    HAL_COMP_Start(&hcomp1);
    HAL_COMP_Start(&hcomp2);
    HAL_COMP_Start(&hcomp4);
    HAL_COMP_Start(&hcomp7);*/
    __HAL_TIM_SET_COUNTER(&htim1, 10);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&measurement_buffers.RawADC[0][0], 3);
    __HAL_TIM_SET_COUNTER(&htim1, 10);

    HAL_ADC_Start_DMA(&hadc2, (uint32_t *)&measurement_buffers.RawADC[1][0], 3);
    __HAL_TIM_SET_COUNTER(&htim1, 10);

    HAL_ADC_Start_DMA(&hadc3, (uint32_t *)&measurement_buffers.RawADC[2][0], 1);
    // Here we can init the measurement buffer offsets; ADC and timer and
    // interrupts are running...
    HAL_Delay(100);
    initing = 0;

    __HAL_TIM_MOE_ENABLE(
        &htim1);  // initialising the comparators triggers the break state

    BLDCVars.BLDCduty = 70;

    // Add a little area in which I can mess about without the RTOS
    while (1) {
        HAL_Delay(100);
        // explicit typecasting to stop warning generation due to direct string
        // argument.
        HAL_UART_Transmit(&huart3, (uint8_t *)"HelloWorld\r", 12, 10);
    }
    /* USER CODE END 2 */

    /* Init scheduler */
    osKernelInitialize();

    /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
    /* USER CODE END RTOS_MUTEX */

    /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
    /* USER CODE END RTOS_SEMAPHORES */

    /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
    /* USER CODE END RTOS_TIMERS */

    /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
    /* USER CODE END RTOS_QUEUES */

    /* Create the thread(s) */
    /* creation of defaultTask */
    defaultTaskHandle =
        osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

    /* creation of SlowLoopTask */
    SlowLoopTaskHandle =
        osThreadNew(SlowLoopEntry, NULL, &SlowLoopTask_attributes);

    /* creation of ComsTask */
    ComsTaskHandle = osThreadNew(ComsTaskEntry, NULL, &ComsTask_attributes);

    /* creation of BatCheckTask */
    BatCheckTaskHandle = osThreadNew(BatCheck, NULL, &BatCheckTask_attributes);

    /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
    /* USER CODE END RTOS_THREADS */

    /* Start scheduler */
    osKernelStart();

    /* We should never get here as control is now taken by the scheduler */
    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        /* USER CODE END WHILE */
        // TODO: should put a bit of panic code here. something that'll send a
        // message through uart and set all motor controls in safe state.
        // Perhaps outside of while loop.

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType =
        RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }
    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
    PeriphClkInit.PeriphClockSelection =
        RCC_PERIPHCLK_USB | RCC_PERIPHCLK_USART3 | RCC_PERIPHCLK_I2C1 |
        RCC_PERIPHCLK_TIM1 | RCC_PERIPHCLK_ADC12 | RCC_PERIPHCLK_ADC34;
    PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
    PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
    PeriphClkInit.Adc34ClockSelection = RCC_ADC34PLLCLK_DIV1;
    PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
    PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
    PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        Error_Handler();
    }
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {
    /* USER CODE BEGIN ADC1_Init 0 */

    /* USER CODE END ADC1_Init 0 */

    ADC_MultiModeTypeDef multimode = {0};
    ADC_AnalogWDGConfTypeDef AnalogWDGConfig = {0};
    ADC_ChannelConfTypeDef sConfig = {0};

    /* USER CODE BEGIN ADC1_Init 1 */

    /* USER CODE END ADC1_Init 1 */
    /** Common config
     */
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_FALLING;
    hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_TRGO;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 3;
    hadc1.Init.DMAContinuousRequests = ENABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hadc1.Init.LowPowerAutoWait = DISABLE;
    hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) {
        Error_Handler();
    }
    /** Configure the ADC multi-mode
     */
    multimode.Mode = ADC_MODE_INDEPENDENT;
    if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK) {
        Error_Handler();
    }
    /** Configure Analog WatchDog 1
     */
    AnalogWDGConfig.WatchdogNumber = ADC_ANALOGWATCHDOG_1;
    AnalogWDGConfig.WatchdogMode = ADC_ANALOGWATCHDOG_SINGLE_REG;
    AnalogWDGConfig.HighThreshold = 0;
    AnalogWDGConfig.LowThreshold = 0;
    AnalogWDGConfig.Channel = ADC_CHANNEL_1;
    AnalogWDGConfig.ITMode = DISABLE;
    if (HAL_ADC_AnalogWDGConfig(&hadc1, &AnalogWDGConfig) != HAL_OK) {
        Error_Handler();
    }
    /** Configure Regular Channel
     */
    sConfig.Channel = ADC_CHANNEL_3;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.SamplingTime = ADC_SAMPLETIME_61CYCLES_5;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }
    /** Configure Regular Channel
     */
    sConfig.Channel = ADC_CHANNEL_1;
    sConfig.Rank = ADC_REGULAR_RANK_2;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }
    /** Configure Regular Channel
     */
    sConfig.Channel = ADC_CHANNEL_4;
    sConfig.Rank = ADC_REGULAR_RANK_3;
    sConfig.SamplingTime = ADC_SAMPLETIME_4CYCLES_5;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN ADC1_Init 2 */

    /* USER CODE END ADC1_Init 2 */
}

/**
 * @brief ADC2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC2_Init(void) {
    /* USER CODE BEGIN ADC2_Init 0 */

    /* USER CODE END ADC2_Init 0 */

    ADC_ChannelConfTypeDef sConfig = {0};

    /* USER CODE BEGIN ADC2_Init 1 */

    /* USER CODE END ADC2_Init 1 */
    /** Common config
     */
    hadc2.Instance = ADC2;
    hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
    hadc2.Init.Resolution = ADC_RESOLUTION_12B;
    hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
    hadc2.Init.ContinuousConvMode = DISABLE;
    hadc2.Init.DiscontinuousConvMode = DISABLE;
    hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_FALLING;
    hadc2.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_TRGO;
    hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc2.Init.NbrOfConversion = 3;
    hadc2.Init.DMAContinuousRequests = ENABLE;
    hadc2.Init.EOCSelection = ADC_EOC_SEQ_CONV;
    hadc2.Init.LowPowerAutoWait = DISABLE;
    hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
    if (HAL_ADC_Init(&hadc2) != HAL_OK) {
        Error_Handler();
    }
    /** Configure Regular Channel
     */
    sConfig.Channel = ADC_CHANNEL_3;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.SamplingTime = ADC_SAMPLETIME_61CYCLES_5;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {
        Error_Handler();
    }
    /** Configure Regular Channel
     */
    sConfig.Channel = ADC_CHANNEL_1;
    sConfig.Rank = ADC_REGULAR_RANK_2;
    sConfig.SamplingTime = ADC_SAMPLETIME_4CYCLES_5;
    if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {
        Error_Handler();
    }
    /** Configure Regular Channel
     */
    sConfig.Channel = ADC_CHANNEL_2;
    sConfig.Rank = ADC_REGULAR_RANK_3;
    if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN ADC2_Init 2 */

    /* USER CODE END ADC2_Init 2 */
}

/**
 * @brief ADC3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC3_Init(void) {
    /* USER CODE BEGIN ADC3_Init 0 */

    /* USER CODE END ADC3_Init 0 */

    ADC_MultiModeTypeDef multimode = {0};
    ADC_ChannelConfTypeDef sConfig = {0};

    /* USER CODE BEGIN ADC3_Init 1 */

    /* USER CODE END ADC3_Init 1 */
    /** Common config
     */
    hadc3.Instance = ADC3;
    hadc3.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
    hadc3.Init.Resolution = ADC_RESOLUTION_12B;
    hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc3.Init.ContinuousConvMode = DISABLE;
    hadc3.Init.DiscontinuousConvMode = DISABLE;
    hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_FALLING;
    hadc3.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_TRGO;
    hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc3.Init.NbrOfConversion = 1;
    hadc3.Init.DMAContinuousRequests = ENABLE;
    hadc3.Init.EOCSelection = ADC_EOC_SEQ_CONV;
    hadc3.Init.LowPowerAutoWait = DISABLE;
    hadc3.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
    if (HAL_ADC_Init(&hadc3) != HAL_OK) {
        Error_Handler();
    }
    /** Configure the ADC multi-mode
     */
    multimode.Mode = ADC_MODE_INDEPENDENT;
    if (HAL_ADCEx_MultiModeConfigChannel(&hadc3, &multimode) != HAL_OK) {
        Error_Handler();
    }
    /** Configure Regular Channel
     */
    sConfig.Channel = ADC_CHANNEL_1;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.SamplingTime = ADC_SAMPLETIME_61CYCLES_5;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN ADC3_Init 2 */

    /* USER CODE END ADC3_Init 2 */
}

/**
 * @brief COMP1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_COMP1_Init(void) {
    /* USER CODE BEGIN COMP1_Init 0 */

    /* USER CODE END COMP1_Init 0 */

    /* USER CODE BEGIN COMP1_Init 1 */

    /* USER CODE END COMP1_Init 1 */
    hcomp1.Instance = COMP1;
    hcomp1.Init.InvertingInput = COMP_INVERTINGINPUT_1_4VREFINT;
    hcomp1.Init.NonInvertingInput = COMP_NONINVERTINGINPUT_IO1;
    hcomp1.Init.Output = COMP_OUTPUT_TIM1BKIN2;
    hcomp1.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
    hcomp1.Init.Hysteresis = COMP_HYSTERESIS_NONE;
    hcomp1.Init.BlankingSrce = COMP_BLANKINGSRCE_NONE;
    hcomp1.Init.Mode = COMP_MODE_HIGHSPEED;
    hcomp1.Init.WindowMode = COMP_WINDOWMODE_DISABLE;
    hcomp1.Init.TriggerMode = COMP_TRIGGERMODE_NONE;
    if (HAL_COMP_Init(&hcomp1) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN COMP1_Init 2 */

    /* USER CODE END COMP1_Init 2 */
}

/**
 * @brief COMP2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_COMP2_Init(void) {
    /* USER CODE BEGIN COMP2_Init 0 */

    /* USER CODE END COMP2_Init 0 */

    /* USER CODE BEGIN COMP2_Init 1 */

    /* USER CODE END COMP2_Init 1 */
    hcomp2.Instance = COMP2;
    hcomp2.Init.InvertingInput = COMP_INVERTINGINPUT_1_4VREFINT;
    hcomp2.Init.NonInvertingInput = COMP_NONINVERTINGINPUT_IO1;
    hcomp2.Init.Output = COMP_OUTPUT_TIM1BKIN2;
    hcomp2.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
    hcomp2.Init.Hysteresis = COMP_HYSTERESIS_NONE;
    hcomp2.Init.BlankingSrce = COMP_BLANKINGSRCE_NONE;
    hcomp2.Init.Mode = COMP_MODE_HIGHSPEED;
    hcomp2.Init.WindowMode = COMP_WINDOWMODE_DISABLE;
    hcomp2.Init.TriggerMode = COMP_TRIGGERMODE_NONE;
    if (HAL_COMP_Init(&hcomp2) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN COMP2_Init 2 */

    /* USER CODE END COMP2_Init 2 */
}

/**
 * @brief COMP4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_COMP4_Init(void) {
    /* USER CODE BEGIN COMP4_Init 0 */

    /* USER CODE END COMP4_Init 0 */

    /* USER CODE BEGIN COMP4_Init 1 */

    /* USER CODE END COMP4_Init 1 */
    hcomp4.Instance = COMP4;
    hcomp4.Init.InvertingInput = COMP_INVERTINGINPUT_1_4VREFINT;
    hcomp4.Init.NonInvertingInput = COMP_NONINVERTINGINPUT_IO1;
    hcomp4.Init.Output = COMP_OUTPUT_TIM1BKIN2;
    hcomp4.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
    hcomp4.Init.Hysteresis = COMP_HYSTERESIS_NONE;
    hcomp4.Init.BlankingSrce = COMP_BLANKINGSRCE_NONE;
    hcomp4.Init.Mode = COMP_MODE_HIGHSPEED;
    hcomp4.Init.WindowMode = COMP_WINDOWMODE_DISABLE;
    hcomp4.Init.TriggerMode = COMP_TRIGGERMODE_NONE;
    if (HAL_COMP_Init(&hcomp4) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN COMP4_Init 2 */

    /* USER CODE END COMP4_Init 2 */
}

/**
 * @brief COMP7 Initialization Function
 * @param None
 * @retval None
 */
static void MX_COMP7_Init(void) {
    /* USER CODE BEGIN COMP7_Init 0 */

    /* USER CODE END COMP7_Init 0 */

    /* USER CODE BEGIN COMP7_Init 1 */

    /* USER CODE END COMP7_Init 1 */
    hcomp7.Instance = COMP7;
    hcomp7.Init.InvertingInput = COMP_INVERTINGINPUT_3_4VREFINT;
    hcomp7.Init.NonInvertingInput = COMP_NONINVERTINGINPUT_IO1;
    hcomp7.Init.Output = COMP_OUTPUT_TIM1BKIN2;
    hcomp7.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
    hcomp7.Init.Hysteresis = COMP_HYSTERESIS_NONE;
    hcomp7.Init.BlankingSrce = COMP_BLANKINGSRCE_NONE;
    hcomp7.Init.Mode = COMP_MODE_HIGHSPEED;
    hcomp7.Init.WindowMode = COMP_WINDOWMODE_DISABLE;
    hcomp7.Init.TriggerMode = COMP_TRIGGERMODE_NONE;
    if (HAL_COMP_Init(&hcomp7) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN COMP7_Init 2 */

    /* USER CODE END COMP7_Init 2 */
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {
    /* USER CODE BEGIN I2C1_Init 0 */

    /* USER CODE END I2C1_Init 0 */

    /* USER CODE BEGIN I2C1_Init 1 */

    /* USER CODE END I2C1_Init 1 */
    hi2c1.Instance = I2C1;
    hi2c1.Init.Timing = 0x0000020B;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
        Error_Handler();
    }
    /** Configure Analogue filter
     */
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) !=
        HAL_OK) {
        Error_Handler();
    }
    /** Configure Digital filter
     */
    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN I2C1_Init 2 */

    /* USER CODE END I2C1_Init 2 */
}

/**
 * @brief OPAMP1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_OPAMP1_Init(void) {
    /* USER CODE BEGIN OPAMP1_Init 0 */

    /* USER CODE END OPAMP1_Init 0 */

    /* USER CODE BEGIN OPAMP1_Init 1 */

    /* USER CODE END OPAMP1_Init 1 */
    hopamp1.Instance = OPAMP1;
    hopamp1.Init.Mode = OPAMP_PGA_MODE;
    hopamp1.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
    hopamp1.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
    hopamp1.Init.PgaConnect = OPAMP_PGA_CONNECT_INVERTINGINPUT_NO;
    hopamp1.Init.PgaGain = OPAMP_PGA_GAIN_16;
    hopamp1.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
    if (HAL_OPAMP_Init(&hopamp1) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN OPAMP1_Init 2 */

    /* USER CODE END OPAMP1_Init 2 */
}

/**
 * @brief OPAMP2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_OPAMP2_Init(void) {
    /* USER CODE BEGIN OPAMP2_Init 0 */

    /* USER CODE END OPAMP2_Init 0 */

    /* USER CODE BEGIN OPAMP2_Init 1 */

    /* USER CODE END OPAMP2_Init 1 */
    hopamp2.Instance = OPAMP2;
    hopamp2.Init.Mode = OPAMP_PGA_MODE;
    hopamp2.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
    hopamp2.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
    hopamp2.Init.PgaConnect = OPAMP_PGA_CONNECT_INVERTINGINPUT_NO;
    hopamp2.Init.PgaGain = OPAMP_PGA_GAIN_16;
    hopamp2.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
    if (HAL_OPAMP_Init(&hopamp2) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN OPAMP2_Init 2 */

    /* USER CODE END OPAMP2_Init 2 */
}

/**
 * @brief OPAMP3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_OPAMP3_Init(void) {
    /* USER CODE BEGIN OPAMP3_Init 0 */

    /* USER CODE END OPAMP3_Init 0 */

    /* USER CODE BEGIN OPAMP3_Init 1 */

    /* USER CODE END OPAMP3_Init 1 */
    hopamp3.Instance = OPAMP3;
    hopamp3.Init.Mode = OPAMP_PGA_MODE;
    hopamp3.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
    hopamp3.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
    hopamp3.Init.PgaConnect = OPAMP_PGA_CONNECT_INVERTINGINPUT_NO;
    hopamp3.Init.PgaGain = OPAMP_PGA_GAIN_16;
    hopamp3.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
    if (HAL_OPAMP_Init(&hopamp3) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN OPAMP3_Init 2 */

    /* USER CODE END OPAMP3_Init 2 */
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {
    /* USER CODE BEGIN TIM1_Init 0 */

    /* USER CODE END TIM1_Init 0 */

    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    /* USER CODE BEGIN TIM1_Init 1 */

    /* USER CODE END TIM1_Init 1 */
    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 0;
    htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
    htim1.Init.Period = 1023;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC4REF;
    sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) !=
        HAL_OK) {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 512;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) !=
        HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) !=
        HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) !=
        HAL_OK) {
        Error_Handler();
    }
    sConfigOC.Pulse = 5;
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) !=
        HAL_OK) {
        Error_Handler();
    }
    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime = 30;
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.BreakFilter = 0;
    sBreakDeadTimeConfig.Break2State = TIM_BREAK2_ENABLE;
    sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
    sBreakDeadTimeConfig.Break2Filter = 6;
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) !=
        HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM1_Init 2 */

    /* USER CODE END TIM1_Init 2 */
    HAL_TIM_MspPostInit(&htim1);
}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {
    /* USER CODE BEGIN TIM3_Init 0 */

    /* USER CODE END TIM3_Init 0 */

    TIM_SlaveConfigTypeDef sSlaveConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_IC_InitTypeDef sConfigIC = {0};

    /* USER CODE BEGIN TIM3_Init 1 */

    /* USER CODE END TIM3_Init 1 */
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 71;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 65535;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_IC_Init(&htim3) != HAL_OK) {
        Error_Handler();
    }
    sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
    sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
    sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
    sSlaveConfig.TriggerFilter = 0;
    if (HAL_TIM_SlaveConfigSynchro(&htim3, &sSlaveConfig) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) !=
        HAL_OK) {
        Error_Handler();
    }
    sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
    sConfigIC.ICFilter = 0;
    if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
    sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
    sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
    if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM3_Init 2 */

    /* USER CODE END TIM3_Init 2 */
}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void) {
    /* USER CODE BEGIN TIM4_Init 0 */

    /* USER CODE END TIM4_Init 0 */

    TIM_SlaveConfigTypeDef sSlaveConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_IC_InitTypeDef sConfigIC = {0};

    /* USER CODE BEGIN TIM4_Init 1 */

    /* USER CODE END TIM4_Init 1 */
    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 109;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = 65535;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim4) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_IC_Init(&htim4) != HAL_OK) {
        Error_Handler();
    }
    sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
    sSlaveConfig.InputTrigger = TIM_TS_TI1F_ED;
    sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
    sSlaveConfig.TriggerFilter = 0;
    if (HAL_TIM_SlaveConfigSynchro(&htim4, &sSlaveConfig) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) !=
        HAL_OK) {
        Error_Handler();
    }
    sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
    sConfigIC.ICFilter = 0;
    if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_3) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_ConfigTI1Input(&htim4, TIM_TI1SELECTION_XORCOMBINATION) !=
        HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM4_Init 2 */

    /* USER CODE END TIM4_Init 2 */
}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void) {
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
    huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart3) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN USART3_Init 2 */

    /* USER CODE END USART3_Init 2 */
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {
    /* DMA controller clock enable */
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    /* DMA interrupt init */
    /* DMA1_Channel1_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
    /* DMA1_Channel2_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
    /* DMA1_Channel3_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
    /* DMA1_Channel6_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
    /* DMA1_Channel7_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
    /* DMA2_Channel1_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA2_Channel1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA2_Channel1_IRQn);
    /* DMA2_Channel5_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA2_Channel5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA2_Channel5_IRQn);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvcpltCallback(ADC_HandleTypeDef *hadc) {}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument) {
    /* init code for USB_DEVICE */
    MX_USB_DEVICE_Init();
    /* USER CODE BEGIN 5 */
    /* Infinite loop */
    for (;;) {
        osDelay(1);
    }
    /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_SlowLoopEntry */
/**
 * @brief Function implementing the SlowLoopTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_SlowLoopEntry */
void SlowLoopEntry(void *argument) {
    /* USER CODE BEGIN SlowLoopEntry */
    /* Infinite loop */
    for (;;) {
        osDelay(1);
    }
    /* USER CODE END SlowLoopEntry */
}

/* USER CODE BEGIN Header_ComsTaskEntry */
/**
 * @brief Function implementing the ComsTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_ComsTaskEntry */
void ComsTaskEntry(void *argument) {
    /* USER CODE BEGIN ComsTaskEntry */
    /* Infinite loop */
    for (;;) {
        osDelay(1);
    }
    /* USER CODE END ComsTaskEntry */
}

/* USER CODE BEGIN Header_BatCheck */
/**
 * @brief Function implementing the BatCheckTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_BatCheck */
void BatCheck(void *argument) {
    /* USER CODE BEGIN BatCheck */
    /* Infinite loop */
    for (;;) {
        osDelay(1);
    }
    /* USER CODE END BatCheck */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM7 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    /* USER CODE BEGIN Callback 0 */

    /* USER CODE END Callback 0 */
    if (htim->Instance == TIM7) {
        HAL_IncTick();
    }
    /* USER CODE BEGIN Callback 1 */

    /* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state
     */

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
void assert_failed(uint8_t *file, uint32_t line) {
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line
       number, tex: printf("Wrong parameters value: file %s on line %d\r\n",
       file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
