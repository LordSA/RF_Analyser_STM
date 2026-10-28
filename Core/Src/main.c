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
    STATE_DIAG_DISPLAY,
	STATE_ERROR,
} SystemState_t;

static SystemState_t current_state = STATE_MAIN_MENU;
static int8_t menu_cursor = 0;
static int8_t list_scroll_offset = 0;

typedef struct {
    char name[16];
    int8_t rssi;
} ScanItem_t;

static ScanItem_t wifi_list[15];
static uint8_t wifi_count = 0;

static ScanItem_t bt_list[15];
static uint8_t bt_count = 0;

static uint32_t last_display_update = 0;


static uint8_t uart_rx_byte;
static char uart_rx_buffer[128]; // Expanded to prevent truncation
static uint8_t uart_rx_index = 0;
static volatile uint32_t uart_rx_count = 0;
static volatile uint8_t esp32_connected = 0;
static char last_diag_msg[32] = "None";

static volatile uint8_t esp32_cmd_ack = 0;   // set by ISR when we get the ack
static char esp32_ack_buf[32] = {0};

static uint32_t error_enter_tick = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
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
  /* USER CODE BEGIN 2 */
  ssd1306_Init();

  ssd1306_Fill(Black);
  ssd1306_SetCursor(15, 10);
  ssd1306_WriteString("STM32 READY", Font_7x10, White);
  ssd1306_SetCursor(15, 30);
  ssd1306_WriteString("Waiting for ESP32...", Font_6x8, White);
  ssd1306_UpdateScreen();

  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      NRF_HandleButtons();

      // Auto-recover from error after 3s
      if (current_state == STATE_ERROR) {
          if (HAL_GetTick() - error_enter_tick > 3000) {
              error_enter_tick = 0;
              current_state = STATE_MAIN_MENU;
              menu_cursor = 0;
          }
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


  /*Configure GPIO pin Output Level */


  /*Configure GPIO pins : PA0 PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */

  /*Configure GPIO pin : PB0 */


  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

static int WaitForESP32Response(const char *expected, uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    while (HAL_GetTick() - start < timeout_ms)
    {
        if (esp32_cmd_ack)
        {
            esp32_cmd_ack = 0;
            return (strstr(esp32_ack_buf, expected) != NULL) ? 1 : 0;
        }
    }
    return 0; // timeout
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
}

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
                if (WaitForESP32Response("OK", 2000)) {   // wait up to 2s
                    current_state = STATE_JAM_ACTIVE;
                } else {
                    current_state = STATE_ERROR;
                    error_enter_tick = HAL_GetTick();
                }
            } else { // Back
                current_state = STATE_MAIN_MENU;
                menu_cursor = 0;
            }
        }

        else if (current_state == STATE_JAM_ACTIVE) {
            ESP32_SendCmd("$UNLOCK\n");
            if (WaitForESP32Response("OK", 2000)) {
                current_state = STATE_JAM_MENU;
                menu_cursor = 0;
            } else {
                current_state = STATE_ERROR;
                error_enter_tick = HAL_GetTick();
            }
        }

        else if (current_state == STATE_ERROR) {
            current_state = STATE_MAIN_MENU;
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
        ssd1306_WriteString(menu_cursor == 0 ? "> Start Jam" : "  Start Jam", Font_7x10, White);

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
        ssd1306_WriteString("ESP32 RF Active", Font_6x8, White);    

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
        ssd1306_WriteString("nRF24: ON ESP32", Font_6x8, White);

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

    else if (current_state == STATE_ERROR)
    {
        ssd1306_SetCursor(0, 0);
        ssd1306_WriteString("!! ERROR !!", Font_7x10, White);

        ssd1306_SetCursor(0, 20);
        ssd1306_WriteString("ESP32 No Response", Font_6x8, White);

        ssd1306_SetCursor(0, 40);
        ssd1306_WriteString("Returning to menu...", Font_6x8, White);
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

            if (strncmp(uart_rx_buffer, "$LOCK,", 6) == 0 ||
                strncmp(uart_rx_buffer, "$UNLOCK,", 8) == 0)
            {
                strncpy(esp32_ack_buf, uart_rx_buffer, sizeof(esp32_ack_buf) - 1);
                esp32_ack_buf[sizeof(esp32_ack_buf) - 1] = '\0';
                esp32_cmd_ack = 1;
            }

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
