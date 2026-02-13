/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include <stdbool.h>
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define MAX_DATA_LEN 16 // Maksimum veri boyutu (başlık + veri boyu + tür + adres + veri)
uint8_t unsignedData[MAX_DATA_LEN] = {0x45, 0x43, 0x45, 0x00, 0x00};
uint8_t signedData[MAX_DATA_LEN] = {0x00, 0x45, 0x43, 0x45, 0x00, 0x00};
volatile bool dualMode = false;
volatile bool transmitFlag = false;

uint8_t _1uartHeader[1];          // Başlık (2) + Veri boyu (1) + Tür(1) + Adres + Veri
uint8_t _1uartData[MAX_DATA_LEN]; // Başlık (2) + Veri boyu (1) + Tür(1) + Adres + Veri

uint8_t _2uartHeader[1];          // Başlık (2) + Veri boyu (1) + Tür(1) + Adres + Veri
uint8_t _2uartData[MAX_DATA_LEN]; // Başlık (2) + Veri boyu (1) + Tür(1) + Adres + Veri

uint8_t _3uartHeader[1];          // Başlık (2) + Veri boyu (1) + Tür(1) + Adres + Veri
uint8_t _3uartData[MAX_DATA_LEN]; // Başlık (2) + Veri boyu (1) + Tür(1) + Adres + Veri

uint8_t _3MyDirection = 0;           // 01 -> 1.Makine ___ 02 -> 2.Makine ___ 03 -> MASTER
volatile bool _2MyDirection = false; // 01 -> 1.Makine ___ 02 -> 2.Makine ___ 03 -> MASTER
volatile bool _1MyDirection = false; // 01 -> 1.Makine ___ 02 -> 2.Makine ___ 03 -> MASTER

uint8_t _3dataLen = 0;               // Alınan veri boyutu
volatile bool _3headerCheck = false; // Başlık kontrolü için değişken
volatile bool _3header0 = false;     // başlık 1. bayte kontrolü için değişken
volatile bool _3header1 = false;     // başlık 2. bayte kontrolü için değişken
volatile bool _3header2 = false;     // başlık 2. bayte kontrolü için değişken

uint8_t _2dataLen = 0;               // Alınan veri boyutu
volatile bool _2headerCheck = false; // Başlık kontrolü için değişken
volatile bool _2header0 = false;     // başlık 1. bayte kontrolü için değişken
volatile bool _2header1 = false;     // başlık 2. bayte kontrolü için değişken
volatile bool _2header2 = false;     // başlık 2. bayte kontrolü için değişken

uint8_t _1dataLen = 0;               // Alınan veri boyutu
volatile bool _1headerCheck = false; // Başlık kontrolü için değişken
volatile bool _1header0 = false;     // başlık 1. bayte kontrolü için değişken
volatile bool _1header1 = false;     // başlık 2. bayte kontrolü için değişken
volatile bool _1header2 = false;     // başlık 2. bayte kontrolü için değişken
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
void TransmitToSlave(uint8_t *data, uint8_t len);  // VERİLERİN İŞLENME FONKSİYONU DÜZENLENECEK
void TransmitToMaster(uint8_t *data, uint8_t len); // VERİLERİN İŞLENME FONKSİYONU DÜZENLENECEK
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void TransmitToSlave(uint8_t *data, uint8_t len) // VERİLERİN SLAVE KARTLARA İLETİM FONKSİYONU
{
  unsignedData[3] = len;        // 0x06
  for (int i = 0; i < len; i++) // gönderilecek verinin hazırlanması
    unsignedData[i + 4] = data[i];

  if (_3MyDirection == 0x01 || _3MyDirection == 0x00)
    HAL_UART_Transmit(&huart1, unsignedData, len + 4, 2);
  else if (_3MyDirection == 0x02)
    HAL_UART_Transmit(&huart2, unsignedData, len + 4, 2);
  _3MyDirection = 0x00;
}

void TransmitToMaster(uint8_t *data, uint8_t len) // VERİLERİN MASTER KARTINA İLETİM FONKSİYONU
{
  if (dualMode)
  {                               // 0-01 / 1-42 / 2-45 / 3-Veri boyutu / 4-1.veri / 5-2.veri / 6-3.veri / 7-4.veri / 8-5.veri / 9-6.veri
    signedData[4] = len;          // 0x06
    for (int i = 0; i < len; i++) // gönderilecek verinin hazırlanması
    {
      signedData[i + 5] = data[i];
    }

    if (_1MyDirection) // Mesaj 1.slaveden mastera gidecek
    {
      signedData[0] = 0x01;
      _1MyDirection = false;
    }
    else if (_2MyDirection) // Mesaj 2.slaveden mastera gidecek
    {
      signedData[0] = 0x02;
      _2MyDirection = false;
    }

    HAL_UART_Transmit(&huart3, signedData, len + 5, 2);
    return;
  }

  unsignedData[3] = len;        // 0x06
  for (int i = 0; i < len; i++) // gönderilecek verinin hazırlanması
  {
    unsignedData[i + 4] = data[i];
  }
  HAL_UART_Transmit(&huart3, unsignedData, len + 4, 2);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3) // MASTER
  {

    // transmitFlag = true; // Masterdan mesaj geldikten sonra veri iletimini başlatır.
    if (!_3header0)
    {
      HAL_GPIO_TogglePin(GPIOC, BLINK_Pin);

      if (_3uartHeader[0] == 0x45) // Başlık kontrolü
      {
        transmitFlag = true;
        _3header0 = true; // Başlık kontrolü başarılı
      }
      else if (_3uartHeader[0] == 0x01 || _3uartHeader[0] == 0x02)
      {
        _3MyDirection = _3uartHeader[0];
        dualMode = true;
      }
      HAL_UART_Receive_DMA(&huart3, _3uartHeader, 1); // Başlık kısmını dinlemeye devam et
    }
    else if (_3header0 && !_3header1)
    {
      if (_3uartHeader[0] == 0x43) // Başlık kontrolü
        _3header1 = true;
      else
        _3header0 = false;                            // Başlık kontrolü başarısız, başlık kısmını sıfırla
      HAL_UART_Receive_DMA(&huart3, _3uartHeader, 1); // Başlık kısmını dinlemeye devam et
    }
    else if (_3header0 && _3header1 && !_3header2)
    {
      if (_3uartHeader[0] == 0x45) // Başlık kontrolü
        _3header2 = true;
      else
      {
        _3header0 = false; // Başlık kontrolü başarısız, başlık kısmını sıfırla
        _3header1 = false;
      }
      HAL_UART_Receive_DMA(&huart3, _3uartHeader, 1); // Başlık kısmını dinlemeye devam et
    }
    else if (_3header0 && _3header1 && _3header2 && !_3headerCheck) // Başlık kontrolü
    {
      _3dataLen = _3uartHeader[0];  // Veri boyutunu al
      if (_3dataLen > MAX_DATA_LEN) // Eğer veri boyutu MAX_DATA_LEN'den büyükse
      {
        _3header0 = false;                              // Başlık kısmını sıfırla
        _3header1 = false;                              // Başlık kısmını sıfırla
        _3header2 = false;                              // Başlık kısmını sıfırla
        HAL_UART_Receive_DMA(&huart3, _3uartHeader, 1); // Başlık kısmını dinlemeye devam et
      }
      else
      {
        _3headerCheck = true;                                 // Başlık kontrolü başarılı
        HAL_UART_Receive_DMA(&huart3, _3uartData, _3dataLen); // Gelen veriyi al
      }
    }
    else if (_3headerCheck)
    {
      _3headerCheck = false; // Başlık kontrolünü sıfırla
      _3header0 = false;     // Başlık kısmını sıfırla
      _3header1 = false;     // Başlık kısmını sıfırla
      _3header2 = false;     // Başlık kısmını sıfırla
      _3MyDirection = 0x01;

      TransmitToSlave(&_3uartData[0], _3dataLen);     // Gelen veriyi işleme fonksiyonunu çağırıyoruz
      HAL_UART_Receive_DMA(&huart3, _3uartHeader, 1); // Başlık kısmını dinlemeye devam et
    }
  }
  if (huart->Instance == USART1 && transmitFlag) // 1.Makine
  {
    if (!_1header0)
    {
      if (_1uartHeader[0] == 0x45)                    // Başlık kontrolü
        _1header0 = true;                             // Başlık kontrolü başarılı
      HAL_UART_Receive_DMA(&huart1, _1uartHeader, 1); // Başlık kısmını dinlemeye devam et
    }
    else if (_1header0 && !_1header1)
    {
      if (_1uartHeader[0] == 0x43) // Başlık kontrolü
        _1header1 = true;
      else
        _1header0 = false;                            // Başlık kontrolü başarısız, başlık kısmını sıfırla
      HAL_UART_Receive_DMA(&huart1, _1uartHeader, 1); // Başlık kısmını dinlemeye devam et
    }
    else if (_1header0 && _1header1 && !_1header2)
    {
      if (_1uartHeader[0] == 0x45) // Başlık kontrolü
        _1header2 = true;
      else
      {
        _1header0 = false; // Başlık kontrolü başarısız, başlık kısmını sıfırla
        _1header1 = false; // Başlık kontrolü başarısız, başlık kısmını sıfırla
      }
      HAL_UART_Receive_DMA(&huart1, _1uartHeader, 1); // Başlık kısmını dinlemeye devam et
    }
    else if (_1header0 && _1header1 && _1header2 && !_1headerCheck) // Başlık kontrolü
    {
      _1dataLen = _1uartHeader[0];  // Veri boyutunu al
      if (_1dataLen > MAX_DATA_LEN) // Eğer veri boyutu MAX_DATA_LEN'den büyükse
      {
        _1header0 = false;                              // Başlık kısmını sıfırla
        _1header1 = false;                              // Başlık kısmını sıfırla
        HAL_UART_Receive_DMA(&huart1, _1uartHeader, 1); // Başlık kısmını dinlemeye devam et
      }
      else
      {
        _1headerCheck = true;                                 // Başlık kontrolü başarılı
        HAL_UART_Receive_DMA(&huart1, _1uartData, _1dataLen); // Gelen veriyi al
      }
    }
    else if (_1headerCheck)
    {
      _1headerCheck = false; // Başlık kontrolünü sıfırla
      _1header0 = false;     // Başlık kısmını sıfırla
      _1header1 = false;     // Başlık kısmını sıfırla
      _1header2 = false;     // Başlık kısmını sıfırla

      _1MyDirection = true;
      TransmitToMaster(&_1uartData[0], _1dataLen);    // Gelen veriyi işleme fonksiyonunu çağırıyoruz
      HAL_UART_Receive_DMA(&huart1, _1uartHeader, 1); // Başlık kısmını dinlemeye devam et
    }
  }
  if (huart->Instance == USART2 && transmitFlag) // 2.Makine
  {
    if (!_2header0)
    {
      if (_2uartHeader[0] == 0x45)                    // Başlık kontrolü
        _2header0 = true;                             // Başlık kontrolü başarılı
      HAL_UART_Receive_DMA(&huart2, _2uartHeader, 1); // Başlık kısmını dinlemeye devam et
    }
    else if (_2header0 && !_2header1)
    {
      if (_2uartHeader[0] == 0x43) // Başlık kontrolü
        _2header1 = true;
      else
        _2header0 = false;                            // Başlık kontrolü başarısız, başlık kısmını sıfırla
      HAL_UART_Receive_DMA(&huart2, _2uartHeader, 1); // Başlık kısmını dinlemeye devam et
    }
    else if (_2header0 && _2header1 && !_2header2)
    {
      if (_2uartHeader[0] == 0x45) // Başlık kontrolü
        _2header2 = true;
      else
      {
        _2header0 = false; // Başlık kontrolü başarısız, başlık kısmını sıfırla
        _2header1 = false; // Başlık kontrolü başarısız, başlık kısmını sıfırla
      }
      HAL_UART_Receive_DMA(&huart2, _2uartHeader, 1); // Başlık kısmını dinlemeye devam et
    }
    else if (_2header0 && _2header1 && _2header2 && !_2headerCheck) // Başlık kontrolü
    {
      _2dataLen = _2uartHeader[0];  // Veri boyutunu al
      if (_2dataLen > MAX_DATA_LEN) // Eğer veri boyutu MAX_DATA_LEN'den büyükse
      {
        _2header0 = false;                              // Başlık kısmını sıfırla
        _2header1 = false;                              // Başlık kısmını sıfırla
        _2header2 = false;                              // Başlık kısmını sıfırla
        HAL_UART_Receive_DMA(&huart2, _2uartHeader, 1); // Başlık kısmını dinlemeye devam et
      }
      else
      {
        _2headerCheck = true;                                 // Başlık kontrolü başarılı
        HAL_UART_Receive_DMA(&huart2, _2uartData, _2dataLen); // Gelen veriyi al
      }
    }
    else if (_2headerCheck)
    {
      _2headerCheck = false; // Başlık kontrolünü sıfırla
      _2header0 = false;     // Başlık kısmını sıfırla
      _2header1 = false;     // Başlık kısmını sıfırla
      _2header2 = false;     // Başlık kısmını sıfırla

      _2MyDirection = true;
      TransmitToMaster(&_2uartData[0], _2dataLen);    // Gelen veriyi işleme fonksiyonunu çağırıyoruz
      HAL_UART_Receive_DMA(&huart2, _2uartHeader, 1); // Başlık kısmını dinlemeye devam et
    }
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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  // USART1 başlatma
  // HAL_UART_Init(&huart1);
  // USART1 için DMA alımı başlatma
  // HAL_UART_Receive_DMA(&huart1, _1uartHeader, 1);

  // USART2 başlatma
  // HAL_UART_Init(&huart2);
  // USART2 için DMA alımı başlatma
  // HAL_UART_Receive_DMA(&huart2, _2uartHeader, 1);

  // USART3 başlatma
  HAL_UART_Init(&huart3);
  // USART3 için DMA alımı başlatma
  HAL_UART_Receive_DMA(&huart3, _3uartHeader, 1);

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  if (HAL_UART_Init(&huart1) != HAL_OK)
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
  if (HAL_UART_Init(&huart2) != HAL_OK)
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
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BLINK_GPIO_Port, BLINK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BLINK_Pin */
  GPIO_InitStruct.Pin = BLINK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BLINK_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
