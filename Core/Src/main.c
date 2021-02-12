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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "SEGGER_RTT.h"
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
SAI_HandleTypeDef hsai_BlockA1;
SAI_HandleTypeDef hsai_BlockB1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SAI1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static enum {
    SILENCE_CVSD,
    SILENCE_mSBC,
    SINE_CVSD,
    SINE_mSBC,
} i2s_tx_mode;

// input signal: pre-computed sine wave, 266 Hz at 16000 Hz

static uint16_t phase = 0;
static const int16_t sine_int16_at_16000hz[] = {
        0,   3135,   6237,   9270,  12202,  14999,  17633,  20073,  22294,  24270,
        25980,  27406,  28531,  29344,  29835,  30000,  29835,  29344,  28531,  27406,
        25980,  24270,  22294,  20073,  17633,  14999,  12202,   9270,   6237,   3135,
        0,  -3135,  -6237,  -9270, -12202, -14999, -17633, -20073, -22294, -24270,
        -25980, -27406, -28531, -29344, -29835, -30000, -29835, -29344, -28531, -27406,
        -25980, -24270, -22294, -20073, -17633, -14999, -12202,  -9270,  -6237,  -3135,
};

// pre-encoded SCO mSBC packets with sine wave 266 Hz at 16000 Hz,
static const uint8_t sco_msbc_sine_data[] = {
        0x01, 0x08, 0xAD, 0x00, 0x00, 0x69, 0xE0, 0x21, 0x11, 0x00, 0x58, 0xEB, 0x91, 0x84, 0xEE,
        0xC8, 0xD1, 0x62, 0x8C, 0x6D, 0x7A, 0x95, 0x69, 0xBD, 0x23, 0xDC, 0x3C, 0xC9, 0x2E, 0xAD,
        0xCF, 0x7A, 0x6E, 0x24, 0xEA, 0x4D, 0x2F, 0x5B, 0x91, 0x42, 0xB6, 0x8D, 0x43, 0x3D, 0x6D,
        0x8A, 0x3C, 0xD1, 0xB1, 0xE0, 0x83, 0xCC, 0x7C, 0x79, 0x8A, 0x2B, 0x7E, 0x91, 0x30, 0x00,

        0x01, 0x38, 0xAD, 0x00, 0x00, 0x69, 0xE0, 0x21, 0x11, 0x00, 0x58, 0xEB, 0x91, 0x84, 0xEE,
        0xC8, 0xD1, 0x62, 0x8C, 0x6D, 0x7A, 0x95, 0x69, 0xBD, 0x23, 0xDC, 0x3C, 0xC9, 0x2E, 0xAD,
        0xCF, 0x7A, 0x6E, 0x24, 0xEA, 0x4D, 0x2F, 0x5B, 0x91, 0x42, 0xB6, 0x8D, 0x43, 0x3D, 0x6D,
        0x8A, 0x3C, 0xD1, 0xB1, 0xE0, 0x83, 0xCC, 0x7C, 0x79, 0x8A, 0x2B, 0x7E, 0x91, 0x30, 0x00,

        0x01, 0xC8, 0xAD, 0x00, 0x00, 0x69, 0xE0, 0x21, 0x11, 0x00, 0x58, 0xEB, 0x91, 0x84, 0xEE,
        0xC8, 0xD1, 0x62, 0x8C, 0x6D, 0x7A, 0x95, 0x69, 0xBD, 0x23, 0xDC, 0x3C, 0xC9, 0x2E, 0xAD,
        0xCF, 0x7A, 0x6E, 0x24, 0xEA, 0x4D, 0x2F, 0x5B, 0x91, 0x42, 0xB6, 0x8D, 0x43, 0x3D, 0x6D,
        0x8A, 0x3C, 0xD1, 0xB1, 0xE0, 0x83, 0xCC, 0x7C, 0x79, 0x8A, 0x2B, 0x7E, 0x91, 0x30, 0x00,

        0x01, 0xF8, 0xAD, 0x00, 0x00, 0x69, 0xE0, 0x21, 0x11, 0x00, 0x58, 0xEB, 0x91, 0x84, 0xEE,
        0xC8, 0xD1, 0x62, 0x8C, 0x6D, 0x7A, 0x95, 0x69, 0xBD, 0x23, 0xDC, 0x3C, 0xC9, 0x2E, 0xAD,
        0xCF, 0x7A, 0x6E, 0x24, 0xEA, 0x4D, 0x2F, 0x5B, 0x91, 0x42, 0xB6, 0x8D, 0x43, 0x3D, 0x6D,
        0x8A, 0x3C, 0xD1, 0xB1, 0xE0, 0x83, 0xCC, 0x7C, 0x79, 0x8A, 0x2B, 0x7E, 0x91, 0x30, 0x00,
};

// pre-encoded SCO mSBC packets with silence
static const uint8_t sco_msbc_silence_data[] = {
        0x01, 0x08, 0xAD, 0x00, 0x00, 0xC5, 0x00, 0x00, 0x00, 0x00, 0x77, 0x6D, 0xB6, 0xDD, 0xDB,
        0x6D, 0xB7, 0x76, 0xDB, 0x6D, 0xDD, 0xB6, 0xDB, 0x77, 0x6D, 0xB6, 0xDD, 0xDB, 0x6D, 0xB7,
        0x76, 0xDB, 0x6D, 0xDD, 0xB6, 0xDB, 0x77, 0x6D, 0xB6, 0xDD, 0xDB, 0x6D, 0xB7, 0x76, 0xDB,
        0x6D, 0xDD, 0xB6, 0xDB, 0x77, 0x6D, 0xB6, 0xDD, 0xDB, 0x6D, 0xB7, 0x76, 0xDB, 0x6C, 0x00,
        0x01, 0x38, 0xAD, 0x00, 0x00, 0xC5, 0x00, 0x00, 0x00, 0x00, 0x77, 0x6D, 0xB6, 0xDD, 0xDB,
        0x6D, 0xB7, 0x76, 0xDB, 0x6D, 0xDD, 0xB6, 0xDB, 0x77, 0x6D, 0xB6, 0xDD, 0xDB, 0x6D, 0xB7,
        0x76, 0xDB, 0x6D, 0xDD, 0xB6, 0xDB, 0x77, 0x6D, 0xB6, 0xDD, 0xDB, 0x6D, 0xB7, 0x76, 0xDB,
        0x6D, 0xDD, 0xB6, 0xDB, 0x77, 0x6D, 0xB6, 0xDD, 0xDB, 0x6D, 0xB7, 0x76, 0xDB, 0x6C, 0x00,
        0x01, 0xC8, 0xAD, 0x00, 0x00, 0xC5, 0x00, 0x00, 0x00, 0x00, 0x77, 0x6D, 0xB6, 0xDD, 0xDB,
        0x6D, 0xB7, 0x76, 0xDB, 0x6D, 0xDD, 0xB6, 0xDB, 0x77, 0x6D, 0xB6, 0xDD, 0xDB, 0x6D, 0xB7,
        0x76, 0xDB, 0x6D, 0xDD, 0xB6, 0xDB, 0x77, 0x6D, 0xB6, 0xDD, 0xDB, 0x6D, 0xB7, 0x76, 0xDB,
        0x6D, 0xDD, 0xB6, 0xDB, 0x77, 0x6D, 0xB6, 0xDD, 0xDB, 0x6D, 0xB7, 0x76, 0xDB, 0x6C, 0x00,
        0x01, 0xF8, 0xAD, 0x00, 0x00, 0xC5, 0x00, 0x00, 0x00, 0x00, 0x77, 0x6D, 0xB6, 0xDD, 0xDB,
        0x6D, 0xB7, 0x76, 0xDB, 0x6D, 0xDD, 0xB6, 0xDB, 0x77, 0x6D, 0xB6, 0xDD, 0xDB, 0x6D, 0xB7,
        0x76, 0xDB, 0x6D, 0xDD, 0xB6, 0xDB, 0x77, 0x6D, 0xB6, 0xDD, 0xDB, 0x6D, 0xB7, 0x76, 0xDB,
        0x6D, 0xDD, 0xB6, 0xDB, 0x77, 0x6D, 0xB6, 0xDD, 0xDB, 0x6D, 0xB7, 0x76, 0xDB, 0x6C, 0x00,
};

static void print_mode(void){
    switch (i2s_tx_mode) {
        case '1':
            printf("I2S TX Sine CVSD\n");
            break;
        case '2':
            printf("I2S TX Sine mSBC\n");
            break;
        case '3':
            printf("I2S TX Silence CVSD\n");
            break;
        case '4':
            printf("I2S TX Silence mSBC\n");
            break;
        default:
            break;
    }
}

static void handle_console_input(char c){
    switch (c) {
        case '1':
            i2s_tx_mode = SINE_CVSD;
            break;
        case '2':
            i2s_tx_mode = SINE_mSBC;
            break;
        case '3':
            i2s_tx_mode = SILENCE_CVSD;
            break;
        case '4':
            i2s_tx_mode = SILENCE_mSBC;
            break;
        default:
            break;
    }
    print_mode();
    phase = 0;
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
    MX_SAI1_Init();
    MX_USART2_UART_Init();
    /* USER CODE BEGIN 2 */
    printf("I2S Test Bridge\n");
    printf("1 - Sine    CVSD  8 kHz/16 bit, 266 Hz\n");
    printf("2 - Sine    mSBC 16 kHz/16 bit, 266 Hz\n");
    printf("3 - Silence CVSD  8 kHz/16 bit\n");
    printf("4 - Silence mSBC 16 kHz/16 bit\n");
    i2s_tx_mode = SINE_CVSD;
    print_mode();
    __HAL_SAI_ENABLE( &hsai_BlockA1);
    __HAL_SAI_ENABLE( &hsai_BlockB1);
    __HAL_UART_ENABLE(&huart2);
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    uint8_t i2s_rx_left_frame = 1;
    uint8_t i2s_tx_left_frame = 1;
    enum {
        UART_TX_IDLE,
        UART_TX_SEND_HIGH,
        UART_TX_SEND_LOW,
    } uart_tx_state = UART_TX_IDLE;
    uint16_t uart_tx_value = 0;
    while (1) {

        // RTT console
        int rtt_input = SEGGER_RTT_HasData(0);
        if (rtt_input > 0) {
            int rtt_key = SEGGER_RTT_GetKey();
            handle_console_input( (char) rtt_key);
        }

        // UART 'console'
        if ((__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE)) == SET) {
            uint8_t uart_key = huart2.Instance->RDR & 0xff;
            handle_console_input( (char) uart_key);
        }

        // Receive from I2S
        if ((hsai_BlockB1.Instance->SR & SAI_xSR_FLVL) != SAI_FIFOSTATUS_EMPTY){
            if (i2s_rx_left_frame) {
                uint32_t temp = hsai_BlockB1.Instance->DR;
                uart_tx_value = (uint16_t) temp;
                uart_tx_state = UART_TX_SEND_HIGH;
                i2s_rx_left_frame = 0;
            } else {
                (void) hsai_BlockB1.Instance->DR;
                i2s_rx_left_frame = 1;
            }
        }

        // Send to UART
        if ((__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TXE)) == SET){
            switch(uart_tx_state){
                case UART_TX_SEND_HIGH:
                    huart2.Instance->TDR = (uint8_t) (uart_tx_value >> 8);
                    uart_tx_state = UART_TX_SEND_LOW;
                    break;
                case UART_TX_SEND_LOW:
                    huart2.Instance->TDR = (uint8_t) (uart_tx_value & 0xff);
                    uart_tx_state = UART_TX_IDLE;
                    break;
                default:
                    break;
            }
        }

        // I2S TX
        if ((hsai_BlockA1.Instance->SR & SAI_xSR_FLVL) != SAI_FIFOSTATUS_FULL){
            uint16_t i2s_tx_value = 0;
            if (i2s_tx_left_frame) {
                switch (i2s_tx_mode){
                    case SINE_CVSD:
                        // 8 kHz, 16 bit
                        i2s_tx_value = (uint16_t) sine_int16_at_16000hz[phase];
                        phase += 2;
                        if (phase >= (sizeof(sine_int16_at_16000hz) / sizeof(int16_t))){
                            phase = 0;
                        }
                        break;
                    case SILENCE_CVSD:
                        i2s_tx_value = 0;
                        break;
                    case SINE_mSBC:
                        // pre-encoded mSBC data
                        i2s_tx_value = sco_msbc_sine_data[phase++];
                        if (phase >= (sizeof(sco_msbc_sine_data) / sizeof(uint8_t))){
                            phase = 0;
                        }
                        break;
                    case SILENCE_mSBC:
                        // pre-encoded mSBC data
                        i2s_tx_value = sco_msbc_silence_data[phase++];
                        if (phase >= (sizeof(sco_msbc_silence_data) / sizeof(uint8_t))){
                            phase = 0;
                        }
                        break;
                    default:
                        break;
                }
                i2s_tx_left_frame = 0;
            } else {
                i2s_tx_left_frame = 1;
            }
            hsai_BlockA1.Instance->DR = i2s_tx_value;
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
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
    RCC_OscInitStruct.MSIState = RCC_MSI_ON;
    RCC_OscInitStruct.MSICalibrationValue = 0;
    RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
    RCC_OscInitStruct.PLL.PLLM = 1;
    RCC_OscInitStruct.PLL.PLLN = 16;
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

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
    {
        Error_Handler();
    }
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_SAI1;
    PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
    PeriphClkInit.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLLSAI1;
    PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
    PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
    PeriphClkInit.PLLSAI1.PLLSAI1N = 16;
    PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
    PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
    PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
    PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_SAI1CLK;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        Error_Handler();
    }
    /** Configure the main internal regulator output voltage
    */
    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief SAI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SAI1_Init(void)
{

    /* USER CODE BEGIN SAI1_Init 0 */

    /* USER CODE END SAI1_Init 0 */

    /* USER CODE BEGIN SAI1_Init 1 */

    /* USER CODE END SAI1_Init 1 */
    hsai_BlockA1.Instance = SAI1_Block_A;
    hsai_BlockA1.Init.AudioMode = SAI_MODESLAVE_TX;
    hsai_BlockA1.Init.Synchro = SAI_ASYNCHRONOUS;
    hsai_BlockA1.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
    hsai_BlockA1.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
    hsai_BlockA1.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
    hsai_BlockA1.Init.MonoStereoMode = SAI_STEREOMODE;
    hsai_BlockA1.Init.CompandingMode = SAI_NOCOMPANDING;
    hsai_BlockA1.Init.TriState = SAI_OUTPUT_NOTRELEASED;
    if (HAL_SAI_InitProtocol(&hsai_BlockA1, SAI_I2S_STANDARD, SAI_PROTOCOL_DATASIZE_16BIT, 2) != HAL_OK)
    {
        Error_Handler();
    }
    hsai_BlockB1.Instance = SAI1_Block_B;
    hsai_BlockB1.Init.AudioMode = SAI_MODESLAVE_RX;
    hsai_BlockB1.Init.Synchro = SAI_SYNCHRONOUS;
    hsai_BlockB1.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
    hsai_BlockB1.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
    hsai_BlockB1.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
    hsai_BlockB1.Init.MonoStereoMode = SAI_STEREOMODE;
    hsai_BlockB1.Init.CompandingMode = SAI_NOCOMPANDING;
    hsai_BlockB1.Init.TriState = SAI_OUTPUT_NOTRELEASED;
    if (HAL_SAI_InitProtocol(&hsai_BlockB1, SAI_I2S_STANDARD, SAI_PROTOCOL_DATASIZE_16BIT, 2) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN SAI1_Init 2 */

    /* USER CODE END SAI1_Init 2 */

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
    huart2.Init.BaudRate = 230400;
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
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin : LD3_Pin */
    GPIO_InitStruct.Pin = LD3_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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
    while (1) {
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
