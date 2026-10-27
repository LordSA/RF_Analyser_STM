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
#include "ssd1306.h"
#include "ssd1306_fonts.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* nRF24L01+ pin definitions */
#define NRF_CSN_PORT GPIOA
#define NRF_CSN_PIN  GPIO_PIN_4

#define NRF_CE_PORT  GPIOB
#define NRF_CE_PIN   GPIO_PIN_0

/* nRF24L01+ register addresses */
#define NRF_REG_CONFIG      0x00
#define NRF_REG_EN_AA       0x01
#define NRF_REG_EN_RXADDR   0x02
#define NRF_REG_SETUP_AW    0x03
#define NRF_REG_SETUP_RETR  0x04
#define NRF_REG_RF_CH       0x05
#define NRF_REG_RF_SETUP    0x06
#define NRF_REG_STATUS      0x07
#define NRF_REG_RX_ADDR_P0  0x0A
#define NRF_REG_RX_PW_P0    0x11
#define NRF_REG_FIFO_STATUS 0x17

#define NRF_CMD_FLUSH_TX    0xE1
#define NRF_CMD_FLUSH_RX    0xE2
#define NRF_CMD_R_REGISTER  0x00
#define NRF_CMD_NOP         0xFF
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUTTON_PRESSED_STATE  GPIO_PIN_RESET
#define MAX_SCANNED_ITEMS     15
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
typedef enum {
    STATE_MAIN_MENU = 0,
    STATE_SCAN_MENU,
    STATE_WIFI_RESULTS,
    STATE_BT_RESULTS,
    STATE_JAM_MENU,
    STATE_JAM_ACTIVE,
    STATE_STATUS_DISPLAY,
    STATE_DIAG_DISPLAY
} SystemState_t;

static SystemState_t current_state = STATE_MAIN_MENU;
static int8_t menu_cursor = 0;
static int8_t list_scroll_offset = 0;
static uint8_t jam_active = 0;

typedef struct {
    char name[16];
    int8_t rssi;
} ScanItem_t;

static ScanItem_t wifi_list[15];
static uint8_t wifi_count = 0;

static ScanItem_t bt_list[15];
static uint8_t bt_count = 0;

static uint8_t nrf_channel = 42;
static uint32_t last_display_update = 0;
static uint32_t last_jam_sweep = 0;

static uint8_t uart_rx_byte;
static char uart_rx_buffer[128]; // Expanded to prevent truncation
static uint8_t uart_rx_index = 0;
static volatile uint32_t uart_rx_count = 0;
static volatile uint8_t esp32_connected = 0;
static char last_diag_msg[32] = "None";
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
static uint8_t NRF_ReadRegister(uint8_t reg);
static HAL_StatusTypeDef NRF_WriteRegister(uint8_t reg, uint8_t value);
static HAL_StatusTypeDef NRF_SendCommand(uint8_t command);
static HAL_StatusTypeDef NRF_ConfigureReceiver(void);
static void NRF_InitTest(void);
static void NRF_UpdateDiagnostics(void);
static void NRF_HandleButtons(void);
static void ESP32_StartUARTReception(void);
static void ESP32_ProcessMessage(char *message);
static void ESP32_SendCmd(const char *cmd);
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
  MX_USART1_UART_Init();
  ESP32_StartUARTReception();
  MX_I2C1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  ssd1306_Init();

  ssd1306_Fill(Black);

  ssd1306_SetCursor(15, 10);
  ssd1306_WriteString("STM32 READY",
                     Font_7x10,
                     White);

  ssd1306_SetCursor(15, 30);
  ssd1306_WriteString("I2C1 Initialized",
                     Font_7x10,
                     White);

  ssd1306_UpdateScreen();
  NRF_InitTest();

  uint8_t config_reg = NRF_ReadRegister(NRF_REG_CONFIG);
  uint8_t status_reg = NRF_ReadRegister(NRF_REG_STATUS);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  HAL_StatusTypeDef radio_result = NRF_ConfigureReceiver();

  uint8_t configured_channel = NRF_ReadRegister(NRF_REG_RF_CH);
  uint8_t configured_config = NRF_ReadRegister(NRF_REG_CONFIG);

  /* Test writing and reading the RF channel register */
  uint8_t test_channel = 42;

  HAL_StatusTypeDef write_result =
      NRF_WriteRegister(NRF_REG_RF_CH, test_channel);

  HAL_Delay(2);

  uint8_t read_channel = NRF_ReadRegister(NRF_REG_RF_CH);

  ssd1306_Fill(Black);
  ssd1306_SetCursor(0, 0);
  ssd1306_WriteString("NRF SPI TEST", Font_7x10, White);

  ssd1306_SetCursor(0, 16);

  if (config_reg == 0x08)
  {
      ssd1306_WriteString("SPI: CHECK", Font_7x10, White);
  }
  else
  {
      ssd1306_WriteString("SPI: VERIFY", Font_7x10, White);
  }

  ssd1306_SetCursor(0, 32);

  char nrf_line[24];

  snprintf(nrf_line, sizeof(nrf_line),
           "CFG:%02X ST:%02X",
           config_reg, status_reg);

  ssd1306_WriteString(nrf_line, Font_6x8, White);

  ssd1306_SetCursor(0, 48);

  char channel_line[24];

  if (write_result == HAL_OK && read_channel == test_channel)
  {
      snprintf(channel_line, sizeof(channel_line), "RF CH TEST: PASS");
  }
  else
  {
      snprintf(channel_line, sizeof(channel_line), "RF CH TEST: FAIL");
  }

  ssd1306_WriteString(channel_line, Font_6x8, White);

  ssd1306_UpdateScreen();

  ssd1306_Fill(Black);

  ssd1306_SetCursor(0, 0);
  ssd1306_WriteString("NRF RECEIVER", Font_7x10, White);

  ssd1306_SetCursor(0, 16);

  if (radio_result == HAL_OK &&
      configured_channel == 42 &&
      configured_config == 0x0F)
  {
      ssd1306_WriteString("CONFIG: PASS", Font_7x10, White);
  }
  else
  {
      ssd1306_WriteString("CONFIG: FAIL", Font_7x10, White);
  }

  char radio_line[24];

  snprintf(radio_line, sizeof(radio_line),
           "CH:%u CFG:%02X",
           configured_channel, configured_config);

  ssd1306_SetCursor(0, 32);
  ssd1306_WriteString(radio_line, Font_6x8, White);

  ssd1306_SetCursor(0, 48);
  ssd1306_WriteString("WAITING FOR PEER", Font_6x8, White);

  ssd1306_UpdateScreen();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

      NRF_HandleButtons();

      // If jammer active, fast sweep channels every 2ms
      if (jam_active && (HAL_GetTick() - last_jam_sweep >= 2))
      {
          last_jam_sweep = HAL_GetTick();
          static uint8_t sweep_ch = 1;
          sweep_ch = (sweep_ch >= 80) ? 1 : sweep_ch + 2;
          NRF_WriteRegister(NRF_REG_RF_CH, sweep_ch);
      }

      if (HAL_GetTick() - last_display_update >= 150)
      {
          last_display_update = HAL_GetTick();
          NRF_UpdateDiagnostics();
      }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
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
  huart1.Init.BaudRate = 9600;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

static uint8_t NRF_ReadRegister(uint8_t reg)
{
    uint8_t command = NRF_CMD_R_REGISTER | (reg & 0x1F);
    uint8_t value = 0;

    HAL_GPIO_WritePin(NRF_CSN_PORT, NRF_CSN_PIN, GPIO_PIN_RESET);

    if (HAL_SPI_Transmit(&hspi1, &command, 1, 100) != HAL_OK)
    {
        HAL_GPIO_WritePin(NRF_CSN_PORT, NRF_CSN_PIN, GPIO_PIN_SET);
        return 0xFF;
    }

    if (HAL_SPI_Receive(&hspi1, &value, 1, 100) != HAL_OK)
    {
        HAL_GPIO_WritePin(NRF_CSN_PORT, NRF_CSN_PIN, GPIO_PIN_SET);
        return 0xFF;
    }

    HAL_GPIO_WritePin(NRF_CSN_PORT, NRF_CSN_PIN, GPIO_PIN_SET);
    return value;
}

static void NRF_InitTest(void)
{
    HAL_GPIO_WritePin(NRF_CE_PORT, NRF_CE_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(NRF_CSN_PORT, NRF_CSN_PIN, GPIO_PIN_SET);
    HAL_Delay(5);
}

static HAL_StatusTypeDef NRF_WriteRegister(uint8_t reg, uint8_t value)
{
    uint8_t data[2];
    data[0] = 0x20 | (reg & 0x1F);
    data[1] = value;

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_StatusTypeDef result = HAL_SPI_Transmit(&hspi1, data, 2, 100);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

    return result;
}

static HAL_StatusTypeDef NRF_SendCommand(uint8_t command)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_StatusTypeDef result = HAL_SPI_Transmit(&hspi1, &command, 1, 100);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

    return result;
}

static HAL_StatusTypeDef NRF_ConfigureReceiver(void)
{
    HAL_StatusTypeDef result;

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

    result = NRF_WriteRegister(NRF_REG_RF_CH, 42);
    if (result != HAL_OK) return result;

    result = NRF_WriteRegister(NRF_REG_CONFIG, 0x0F);
    if (result != HAL_OK) return result;

    result = NRF_WriteRegister(NRF_REG_EN_AA, 0x00);
    if (result != HAL_OK) return result;

    result = NRF_WriteRegister(NRF_REG_EN_RXADDR, 0x01);
    if (result != HAL_OK) return result;

    result = NRF_WriteRegister(NRF_REG_SETUP_AW, 0x03);
    if (result != HAL_OK) return result;

    result = NRF_WriteRegister(NRF_REG_RF_SETUP, 0x06);
    if (result != HAL_OK) return result;

    result = NRF_WriteRegister(NRF_REG_SETUP_RETR, 0x00);
    if (result != HAL_OK) return result;

    result = NRF_WriteRegister(NRF_REG_RX_PW_P0, 32);
    if (result != HAL_OK) return result;

    uint8_t address[6] = {0xD2, 0xD2, 0xD2, 0xD2, 0xD2, 0};
    uint8_t command[6] = {
        0x20 | NRF_REG_RX_ADDR_P0,
        address[0], address[1], address[2], address[3], address[4]
    };

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    result = HAL_SPI_Transmit(&hspi1, command, 6, 100);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

    if (result != HAL_OK) return result;

    result = NRF_WriteRegister(NRF_REG_STATUS, 0x70);
    if (result != HAL_OK) return result;

    result = NRF_SendCommand(NRF_CMD_FLUSH_RX);
    if (result != HAL_OK) return result;

    result = NRF_SendCommand(NRF_CMD_FLUSH_TX);
    if (result != HAL_OK) return result;

    HAL_Delay(5);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

    return HAL_OK;
}

static void ESP32_StartUARTReception(void)
{
    HAL_StatusTypeDef result = HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);
    if (result != HAL_OK)
    {
        Error_Handler();
    }
}
static void ESP32_SendCmd(const char *cmd)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)cmd, strlen(cmd), 100);
}

static void ESP32_ProcessMessage(char *message)
{
    int count = 0;
    int idx = 0;
    int rssi = 0;
    char name_buf[32];

    char *cr = strchr(message, '\r');
    if (cr) *cr = '\0';

    if (strncmp(message, "$STATUS,READY", 13) == 0)
    {
        esp32_connected = 1;
    }
    else if (sscanf(message, "$WIFI,COUNT,%d", &count) == 1)
    {
        wifi_count = 0;
        esp32_connected = 1;
    }
    else if (sscanf(message, "$WIFI,ITEM,%d,%[^,],%d", &idx, name_buf, &rssi) >= 3)
    {
        if (idx < 15)
        {
            strncpy(wifi_list[idx].name, name_buf, sizeof(wifi_list[idx].name) - 1);
            wifi_list[idx].name[sizeof(wifi_list[idx].name) - 1] = '\0';
            wifi_list[idx].rssi = (int8_t)rssi;
            if (idx + 1 > wifi_count) wifi_count = idx + 1;
        }
    }
    else if (sscanf(message, "$BT,COUNT,%d", &count) == 1)
    {
        bt_count = 0;
        esp32_connected = 1;
    }
    else if (sscanf(message, "$BT,ITEM,%d,%*[^,],%d,%[^,\r\n]", &idx, &rssi, name_buf) >= 2)
    {
        if (idx < 15)
        {
            strncpy(bt_list[idx].name, name_buf, sizeof(bt_list[idx].name) - 1);
            bt_list[idx].name[sizeof(bt_list[idx].name) - 1] = '\0';
            bt_list[idx].rssi = (int8_t)rssi;
            if (idx + 1 > bt_count) bt_count = idx + 1;
        }
    }
    else if (strncmp(message, "$LOCK", 5) == 0 && message[5] == ',')
    {
       ESP32_Lock();
    }
    else if (strncmp(message, "$UNLOCK", 7) == 0 && message[7] == ',')
    {
       ESP32_Unlock();
    }
}

//static void NRF_StartJammer(uint8_t channel)
//{
//    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET); // CE Low
//    NRF_WriteRegister(NRF_REG_RF_CH, channel);
//    NRF_WriteRegister(NRF_REG_RF_SETUP, 0x06); // 1Mbps, 0dBm (Max)
//    NRF_WriteRegister(NRF_REG_CONFIG, 0x02);   // PWR_UP = 1, PRIM_RX = 0 (Transmit mode)
//    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);   // CE High (Transmitting carrier)
//    jam_active = 1;
//}
//
//static void NRF_StopJammer(void)
//{
//    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET); // CE Low
//    NRF_ConfigureReceiver(); // Return to safe RX listening mode
//    jam_active = 0;
//}

/* Stable debounced button handler with guaranteed release detection */
static void NRF_HandleButtons(void)
{
    static uint8_t pa0_released = 1;
    static uint8_t pa1_released = 1;
    static uint32_t last_btn_time = 0;

    uint32_t now = HAL_GetTick();
    if (now - last_btn_time < 150) return;

    uint8_t pa0_pressed = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == BUTTON_PRESSED_STATE);
    uint8_t pa1_pressed = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == BUTTON_PRESSED_STATE);

    // PA0: SCROLL / NEXT
    if (pa0_pressed && pa0_released)
    {
        last_btn_time = now;
        pa0_released = 0;

        if (current_state == STATE_MAIN_MENU) {
            menu_cursor = (menu_cursor + 1) % 3; // 0:SCAN, 1:JAM, 2:STATUS
        }
        else if (current_state == STATE_SCAN_MENU) {
            menu_cursor = (menu_cursor + 1) % 3; // 0:Wi-Fi, 1:BLE, 2:Back
        }
        else if (current_state == STATE_JAM_MENU) {
            menu_cursor = (menu_cursor + 1) % 3; // 0:Carrier Jam, 1:Sweep Jam, 2:Back
        }
        else if (current_state == STATE_WIFI_RESULTS) {
            uint8_t total = (wifi_count == 0) ? 1 : (wifi_count + 1);
            menu_cursor = (menu_cursor + 1) % total;
            if (menu_cursor >= list_scroll_offset + 3) list_scroll_offset = menu_cursor - 2;
            if (menu_cursor < list_scroll_offset) list_scroll_offset = menu_cursor;
        }
        else if (current_state == STATE_BT_RESULTS) {
            uint8_t total = (bt_count == 0) ? 1 : (bt_count + 1);
            menu_cursor = (menu_cursor + 1) % total;
            if (menu_cursor >= list_scroll_offset + 3) list_scroll_offset = menu_cursor - 2;
            if (menu_cursor < list_scroll_offset) list_scroll_offset = menu_cursor;
        }
    }
    else if (!pa0_pressed) {
        pa0_released = 1;
    }

    // PA1: ENTER / SELECT
    if (pa1_pressed && pa1_released)
    {
        last_btn_time = now;
        pa1_released = 0;

        if (current_state == STATE_MAIN_MENU) {
            if (menu_cursor == 0) { // SCAN
                current_state = STATE_SCAN_MENU;
                menu_cursor = 0;
            } else if (menu_cursor == 1) { // JAM
                current_state = STATE_JAM_MENU;
                menu_cursor = 0;
            } else { // STATUS
                current_state = STATE_STATUS_DISPLAY;
            }
        }
        else if (current_state == STATE_SCAN_MENU) {
            if (menu_cursor == 0) { // Wi-Fi Scan
                wifi_count = 0;
                ESP32_SendCmd("$WIFI,SCAN,START\n");
                current_state = STATE_WIFI_RESULTS;
                menu_cursor = 0;
                list_scroll_offset = 0;
            } else if (menu_cursor == 1) { // BLE Scan
                bt_count = 0;
                ESP32_SendCmd("$BT,SCAN,START,5\n");
                current_state = STATE_BT_RESULTS;
                menu_cursor = 0;
                list_scroll_offset = 0;
            } else { // Back
                current_state = STATE_MAIN_MENU;
                menu_cursor = 0;
            }
        }
        else if (current_state == STATE_JAM_MENU) {
            if (menu_cursor == 0) { // Single CH Jam
                ESP32_SendCmd("$LOCK\n");
                current_state = STATE_JAM_ACTIVE;
//            } else if (menu_cursor == 1) { // Sweep Jam
//            	ESP32_SendCmd("$UNLOCK\n");
//                current_state = STATE_JAM_INACTIVE;
            } else { // Back
                current_state = STATE_MAIN_MENU;
                menu_cursor = 0;
            }
        }
        else if (current_state == STATE_JAM_ACTIVE) {
            // Any button press stops the jammer safely
        	ESP32_SendCmd("$UNLOCK\n");
            current_state = STATE_JAM_MENU;
            menu_cursor = 0;
        }
        else if (current_state == STATE_WIFI_RESULTS || current_state == STATE_BT_RESULTS) {
            uint8_t count = (current_state == STATE_WIFI_RESULTS) ? wifi_count : bt_count;
            if (menu_cursor >= count) { // Clicked on [< BACK]
                current_state = STATE_SCAN_MENU;
                menu_cursor = 0;
            }
        }
        else {
            current_state = STATE_MAIN_MENU;
            menu_cursor = 0;
        }
    }
    else if (!pa1_pressed) {
        pa1_released = 1;
    }
}

static void NRF_UpdateDiagnostics(void)
{
    char line[32];
    ssd1306_Fill(Black);

    if (current_state == STATE_MAIN_MENU)
    {
        ssd1306_SetCursor(0, 0);
        ssd1306_WriteString("SYSTEM CONTROL", Font_7x10, White);

        ssd1306_SetCursor(0, 16);
        ssd1306_WriteString(menu_cursor == 0 ? "> SCAN" : "  SCAN", Font_7x10, White);

        ssd1306_SetCursor(0, 30);
        ssd1306_WriteString(menu_cursor == 1 ? "> JAMMER" : "  JAMMER", Font_7x10, White);

        ssd1306_SetCursor(0, 44);
        ssd1306_WriteString(menu_cursor == 2 ? "> STATUS" : "  STATUS", Font_7x10, White);
    }
    else if (current_state == STATE_JAM_MENU)
    {
        ssd1306_SetCursor(0, 0);
        ssd1306_WriteString("RF JAMMER", Font_7x10, White);

        ssd1306_SetCursor(0, 16);
        snprintf(line, sizeof(line), menu_cursor == 0 ? "> Start CH %u" : "  Start CH %u", nrf_channel);
        ssd1306_WriteString(line, Font_7x10, White);

        ssd1306_SetCursor(0, 30);
        ssd1306_WriteString(menu_cursor == 1 ? "> Sweep 2.4GHz" : "  Sweep 2.4GHz", Font_7x10, White);

        ssd1306_SetCursor(0, 44);
        ssd1306_WriteString(menu_cursor == 2 ? "> [< BACK]" : "  [< BACK]", Font_7x10, White);
    }
    else if (current_state == STATE_JAM_ACTIVE)
    {
        ssd1306_SetCursor(0, 0);
        ssd1306_WriteString("!! JAMMING !!", Font_7x10, White);

        ssd1306_SetCursor(0, 20);
        ssd1306_WriteString("nRF24 EMITTING...", Font_6x8, White);

        ssd1306_SetCursor(0, 35);
        snprintf(line, sizeof(line), "Active Channel: %u", nrf_channel);
        ssd1306_WriteString(line, Font_6x8, White);

        ssd1306_SetCursor(0, 52);
        ssd1306_WriteString("PA1: STOP JAM", Font_6x8, White);
    }
    else if (current_state == STATE_SCAN_MENU)
    {
        ssd1306_SetCursor(0, 0);
        ssd1306_WriteString("SCAN MODE", Font_7x10, White);

        ssd1306_SetCursor(0, 16);
        ssd1306_WriteString(menu_cursor == 0 ? "> Wi-Fi Scan" : "  Wi-Fi Scan", Font_7x10, White);

        ssd1306_SetCursor(0, 30);
        ssd1306_WriteString(menu_cursor == 1 ? "> BLE Discovery" : "  BLE Discovery", Font_7x10, White);

        ssd1306_SetCursor(0, 44);
        ssd1306_WriteString(menu_cursor == 2 ? "> [< BACK]" : "  [< BACK]", Font_7x10, White);
    }
    else if (current_state == STATE_WIFI_RESULTS)
    {
        ssd1306_SetCursor(0, 0);
        snprintf(line, sizeof(line), "WI-FI (%u Found)", wifi_count);
        ssd1306_WriteString(line, Font_7x10, White);

        // Render 3 visible lines with scrolling
        for (uint8_t row = 0; row < 3; row++)
        {
            uint8_t item_idx = list_scroll_offset + row;
            uint8_t y_pos = 16 + (row * 12);

            ssd1306_SetCursor(0, y_pos);
            if (item_idx < wifi_count)
            {
                char prefix = (menu_cursor == item_idx) ? '>' : ' ';
                snprintf(line, sizeof(line), "%c %s %ddB", prefix, wifi_list[item_idx].name, wifi_list[item_idx].rssi);
                ssd1306_WriteString(line, Font_6x8, White);
            }
            else if (item_idx == wifi_count)
            {
                char prefix = (menu_cursor == item_idx) ? '>' : ' ';
                snprintf(line, sizeof(line), "%c [< BACK]", prefix);
                ssd1306_WriteString(line, Font_6x8, White);
            }
        }

        ssd1306_SetCursor(0, 55);
        ssd1306_WriteString("PA0:Scroll  PA1:Select", Font_6x8, White);
    }
    else if (current_state == STATE_BT_RESULTS)
    {
        ssd1306_SetCursor(0, 0);
        snprintf(line, sizeof(line), "BLE (%u Found)", bt_count);
        ssd1306_WriteString(line, Font_7x10, White);

        for (uint8_t row = 0; row < 3; row++)
        {
            uint8_t item_idx = list_scroll_offset + row;
            uint8_t y_pos = 16 + (row * 12);

            ssd1306_SetCursor(0, y_pos);
            if (item_idx < bt_count)
            {
                char prefix = (menu_cursor == item_idx) ? '>' : ' ';
                snprintf(line, sizeof(line), "%c %s %ddB", prefix, bt_list[item_idx].name, bt_list[item_idx].rssi);
                ssd1306_WriteString(line, Font_6x8, White);
            }
            else if (item_idx == bt_count)
            {
                char prefix = (menu_cursor == item_idx) ? '>' : ' ';
                snprintf(line, sizeof(line), "%c [< BACK]", prefix);
                ssd1306_WriteString(line, Font_6x8, White);
            }
        }

        ssd1306_SetCursor(0, 55);
        ssd1306_WriteString("PA0:Scroll  PA1:Select", Font_6x8, White);
    }
    else if (current_state == STATE_STATUS_DISPLAY)
    {
        ssd1306_SetCursor(0, 0);
        ssd1306_WriteString("SYSTEM STATUS", Font_7x10, White);

        ssd1306_SetCursor(0, 16);
        ssd1306_WriteString(esp32_connected ? "ESP32: CONNECTED" : "ESP32: WAITING", Font_6x8, White);

        ssd1306_SetCursor(0, 28);
        ssd1306_WriteString("nRF24: ACTIVE", Font_6x8, White);

        snprintf(line, sizeof(line), "nRF CH: %u", nrf_channel);
        ssd1306_SetCursor(0, 40);
        ssd1306_WriteString(line, Font_6x8, White);

        ssd1306_SetCursor(0, 52);
        ssd1306_WriteString("> [< BACK]", Font_6x8, White);
    }
    else if (current_state == STATE_DIAG_DISPLAY)
    {
        ssd1306_SetCursor(0, 0);
        ssd1306_WriteString("DIAGNOSTICS", Font_7x10, White);

        ssd1306_SetCursor(0, 20);
        ssd1306_WriteString(last_diag_msg, Font_6x8, White);

        ssd1306_SetCursor(0, 52);
        ssd1306_WriteString("> [< BACK]", Font_6x8, White);
    }

    ssd1306_UpdateScreen();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        uart_rx_count++;
        char c = (char)uart_rx_byte;

        if (c == '\n')
        {
            uart_rx_buffer[uart_rx_index] = '\0';
            ESP32_ProcessMessage(uart_rx_buffer);
            uart_rx_index = 0;
        }
        else if (c != '\r')
        {
            if (uart_rx_index < sizeof(uart_rx_buffer) - 1)
            {
                uart_rx_buffer[uart_rx_index++] = c;
            }
            else
            {
                uart_rx_index = 0;
            }
        }

        HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);
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
