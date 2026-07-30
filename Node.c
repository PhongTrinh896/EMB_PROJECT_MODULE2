/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Sensor Node LoRa - STM32 HAL (Real Sensor Data & Error Handling)
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Khấu cấu hình Giao thức LoRa (Khớp 100% với Gateway) */
#define LORA_SOF                       0xAAU
#define LORA_EOF                       0xBBU

#define LORA_TX_FRAME_SIZE             8U
#define LORA_RX_FRAME_SIZE             3U

#define TX_INDEX_SOF                   0U
#define TX_INDEX_HUMIDITY              1U
#define TX_INDEX_TEMPERATURE           2U
#define TX_INDEX_BATTERY_1             3U
#define TX_INDEX_BATTERY_2             4U
#define TX_INDEX_MOTOR                 5U
#define TX_INDEX_CHECKSUM              6U
#define TX_INDEX_EOF                   7U

#define MOTOR_OFF                      0U
#define MOTOR_ON                       1U

/* Cấu hình phần cứng & Thời gian */
#define DHT12_I2C_ADDRESS              (0x5C << 1)
#define LCD_ADDRESS                    0x4EU
#define LCD_COLUMNS                    16U
#define TELEMETRY_SEND_PERIOD_MS       2000U
#define LCD_UPDATE_PERIOD_MS           500U
#define UART_RX_TIMEOUT_MS             50U   /* Thời gian timeout nhận khung tin UART */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Khối dữ liệu cảm biến & Trạng thái Node */
static uint8_t sensor_humidity = 0U;     
static uint8_t sensor_temperature = 0U;  
static uint8_t battery_sensor_1 = 0U;    
static uint8_t battery_sensor_2 = 0U;
static uint8_t motor_state = MOTOR_OFF;

/* Nhận lệnh qua LoRa USART1 */
static volatile uint8_t lora_rx_byte = 0U;
static uint8_t lora_rx_frame[LORA_RX_FRAME_SIZE] = {0U};
static volatile uint8_t lora_rx_index = 0U;

/* Biến phục vụ Timing */
static uint32_t last_telemetry_tick = 0U;
static uint32_t last_lcd_tick = 0U;
static volatile uint32_t last_rx_tick = 0U;

/* Biến Debug & Đếm lỗi */
static uint32_t uart_error_count = 0U;
static uint32_t sensor_error_count = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
static uint8_t calculate_checksum(const uint8_t *data, uint8_t length);
static void send_telemetry_to_gateway(void);
static void set_motor_pwm(uint8_t state);
static void read_sensors_and_batteries(void);
static uint8_t read_adc_channel(uint32_t channel);

/* Khối hàm LCD I2C */
static void lcd_send_cmd(char cmd);
static void lcd_send_data(char data);
static void lcd_init(void);
static void lcd_send_string(const char *text);
static void lcd_put_cur(uint8_t row, uint8_t column);
static void lcd_print_line(uint8_t row, const char *text);
static void update_node_lcd(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* Callback ngắt nhận dữ liệu lệnh từ Gateway (3 Bytes: SOF, MOTOR_STATE, EOF) */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart1)
  {
    uint8_t received_byte = lora_rx_byte;
    last_rx_tick = HAL_GetTick(); /* Cập nhật mốc thời gian nhận byte */

    if (lora_rx_index == 0U)
    {
      if (received_byte == LORA_SOF)
      {
        lora_rx_frame[0] = received_byte;
        lora_rx_index = 1U;
      }
    }
    else
    {
      lora_rx_frame[lora_rx_index++] = received_byte;

      if (lora_rx_index >= LORA_RX_FRAME_SIZE)
      {
        /* Kiểm tra tính hợp lệ của lệnh từ Gateway */
        if (lora_rx_frame[2] == LORA_EOF)
        {
          uint8_t cmd_motor = lora_rx_frame[1];
          if (cmd_motor == MOTOR_ON || cmd_motor == MOTOR_OFF)
          {
            motor_state = cmd_motor;
            set_motor_pwm(motor_state);
          }
        }
        lora_rx_index = 0U; /* Reset sau khi xử lý xong khung tin */
      }
    }

    /* Tiếp tục ngắt nhận byte tiếp theo */
    (void)HAL_UART_Receive_IT(&huart1, (uint8_t *)&lora_rx_byte, 1U);
  }
}

/* Xử lý lỗi USART1 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart1)
  {
    lora_rx_index = 0U;
    uart_error_count++;
    __HAL_UART_CLEAR_OREFLAG(&huart1);
    __HAL_UART_CLEAR_NEFLAG(&huart1);
    __HAL_UART_CLEAR_FEFLAG(&huart1);
    __HAL_UART_CLEAR_PEFLAG(&huart1);
    (void)HAL_UART_Receive_IT(&huart1, (uint8_t *)&lora_rx_byte, 1U);
  }
}

/* Tính Checksum XOR từ byte Humidity đến Motor (5 bytes: index 1 -> 5) */
static uint8_t calculate_checksum(const uint8_t *data, uint8_t length)
{
  uint8_t checksum = 0U;
  if (data == NULL) return 0U;

  for (uint8_t i = 0U; i < length; i++)
  {
    checksum ^= data[i];
  }
  return checksum;
}

/* Đóng gói và gửi khung tin Telemetry 8-byte lên Gateway */
static void send_telemetry_to_gateway(void)
{
  uint8_t tx_frame[LORA_TX_FRAME_SIZE];

  tx_frame[TX_INDEX_SOF]         = LORA_SOF;             
  tx_frame[TX_INDEX_HUMIDITY]    = sensor_humidity;      
  tx_frame[TX_INDEX_TEMPERATURE] = sensor_temperature;   
  tx_frame[TX_INDEX_BATTERY_1]   = battery_sensor_1;     
  tx_frame[TX_INDEX_BATTERY_2]   = battery_sensor_2;     
  tx_frame[TX_INDEX_MOTOR]       = motor_state;          
  
  tx_frame[TX_INDEX_CHECKSUM]    = calculate_checksum(&tx_frame[TX_INDEX_HUMIDITY], 5U);
  tx_frame[TX_INDEX_EOF]         = LORA_EOF;             

  /* Kiểm tra kết quả truyền UART và tăng biến đếm nếu lỗi */
  if (HAL_UART_Transmit(&huart1, tx_frame, LORA_TX_FRAME_SIZE, 100U) != HAL_OK)
  {
    uart_error_count++;
  }
}

/* Điều khiển PWM Motor */
static void set_motor_pwm(uint8_t state)
{
  if (state == MOTOR_ON)
  {
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 800);
  }
  else
  {
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
  }
}

/* Đọc ADC tùy chọn kênh (Polling Mode) */
static uint8_t read_adc_channel(uint32_t channel)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = channel;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  
  /* Cấu hình kênh muốn đo */
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) == HAL_OK)
  {
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
    {
      uint32_t adc_val = HAL_ADC_GetValue(&hadc1);
      HAL_ADC_Stop(&hadc1);
      return (uint8_t)((adc_val * 100U) / 4095U); /* Quy đổi ra % (Giả lập) */
    }
    HAL_ADC_Stop(&hadc1);
  }
  return 0xFF; /* Lỗi ADC */
}

/* Đọc dữ liệu DHT12 thực tế và 2 Pin độc lập qua ADC */
static void read_sensors_and_batteries(void)
{
  uint8_t rx_data[5];
  
  /* 1. Đọc dữ liệu thực từ DHT12 qua I2C1 */
  if (HAL_I2C_Mem_Read(&hi2c1, DHT12_I2C_ADDRESS, 0x00, I2C_MEMADD_SIZE_8BIT, rx_data, 5, 100) == HAL_OK)
  {
    float humidity = rx_data[0] + (rx_data[1] / 10.0f);
    float temperature = rx_data[2] + (rx_data[3] / 10.0f);

    /* Giới hạn giá trị trước khi truyền để tránh tràn số học */
    if (humidity > 100.0f) humidity = 100.0f;
    if (temperature > 125.0f) temperature = 125.0f; /* Ngưỡng Max nhiệt độ */

    sensor_humidity = (uint8_t)humidity;
    sensor_temperature = (uint8_t)temperature;
  }
  else
  {
    /* Đánh dấu lỗi bằng 0xFF và tăng biến đếm */
    sensor_humidity = 0xFF;
    sensor_temperature = 0xFF;
    sensor_error_count++;
  }

  /* 2. Đọc 2 kênh ADC độc lập cho Pin 1 và Pin 2 */
  battery_sensor_1 = read_adc_channel(ADC_CHANNEL_4); /* PA4 */
  battery_sensor_2 = read_adc_channel(ADC_CHANNEL_5); /* PA5 */
}

/* Hiển thị thông tin lên màn hình LCD */
static void update_node_lcd(void)
{
  char line1[LCD_COLUMNS + 1U];
  char line2[LCD_COLUMNS + 1U];

  if (sensor_humidity == 0xFF || sensor_temperature == 0xFF) {
    snprintf(line1, sizeof(line1), "H:ERR T:ERR M:%s", (motor_state == MOTOR_ON) ? "ON " : "OFF");
  } else {
    snprintf(line1, sizeof(line1), "H:%2u%% T:%2uC M:%s", 
             sensor_humidity, sensor_temperature, (motor_state == MOTOR_ON) ? "ON " : "OFF");
  }
  
  snprintf(line2, sizeof(line2), "B1:%2u%% B2:%2u%%", 
           (battery_sensor_1 == 0xFF) ? 0 : battery_sensor_1, 
           (battery_sensor_2 == 0xFF) ? 0 : battery_sensor_2);

  lcd_print_line(0U, line1);
  lcd_print_line(1U, line2);
}

/* --- Khối điều khiển LCD I2C --- */
static void lcd_send_cmd(char cmd)
{
  char u = (char)(cmd & 0xF0), l = (char)((cmd << 4) & 0xF0);
  uint8_t d[4] = {(uint8_t)u | 0x0C, (uint8_t)u | 0x08, (uint8_t)l | 0x0C, (uint8_t)l | 0x08};
  (void)HAL_I2C_Master_Transmit(&hi2c2, LCD_ADDRESS, d, 4U, 100U);
}

static void lcd_send_data(char data)
{
  char u = (char)(data & 0xF0), l = (char)((data << 4) & 0xF0);
  uint8_t d[4] = {(uint8_t)u | 0x0D, (uint8_t)u | 0x09, (uint8_t)l | 0x0D, (uint8_t)l | 0x09};
  (void)HAL_I2C_Master_Transmit(&hi2c2, LCD_ADDRESS, d, 4U, 100U);
}

static void lcd_init(void)
{
  HAL_Delay(50U);
  lcd_send_cmd(0x30); HAL_Delay(5U);
  lcd_send_cmd(0x30); HAL_Delay(1U);
  lcd_send_cmd(0x30); HAL_Delay(10U);
  lcd_send_cmd(0x20); HAL_Delay(10U);
  lcd_send_cmd(0x28); HAL_Delay(1U);
  lcd_send_cmd(0x08); HAL_Delay(1U);
  lcd_send_cmd(0x01); HAL_Delay(2U);
  lcd_send_cmd(0x06); HAL_Delay(1U);
  lcd_send_cmd(0x0C);
}

static void lcd_send_string(const char *text)
{
  while (text && *text) lcd_send_data(*text++);
}

static void lcd_put_cur(uint8_t row, uint8_t column)
{
  uint8_t addr = (row == 0U) ? (0x80U | column) : (0xC0U | column);
  lcd_send_cmd((char)addr);
}

static void lcd_print_line(uint8_t row, const char *text)
{
  char line[LCD_COLUMNS + 1U];
  memset(line, ' ', LCD_COLUMNS);
  line[LCD_COLUMNS] = '\0';
  if (text) memcpy(line, text, (strlen(text) < LCD_COLUMNS) ? strlen(text) : LCD_COLUMNS);
  lcd_put_cur(row, 0U);
  lcd_send_string(line);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
  /* Khởi tạo phần cứng */
  lcd_init();
  lcd_print_line(0U, "NODE LORA START");
  HAL_Delay(1000);

  /* Chạy Timer PWM cho Motor */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  set_motor_pwm(MOTOR_OFF);

  /* Bắt đầu lắng nghe USART1 từ Gateway */
  last_rx_tick = HAL_GetTick();
  HAL_UART_Receive_IT(&huart1, (uint8_t *)&lora_rx_byte, 1U);

  last_telemetry_tick = HAL_GetTick();
  last_lcd_tick = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t now = HAL_GetTick();

    /* 1. Timeout cho UART Receive */
    if (lora_rx_index > 0U && (now - last_rx_tick) > UART_RX_TIMEOUT_MS)
    {
      /* Xảy ra nghẽn/lỗi khung tin (mất EOF hoặc SOF), reset lại index */
      lora_rx_index = 0U;
      uart_error_count++;
    }

    /* 2. Chu kỳ gửi dữ liệu Telemetry lên Gateway */
    if ((now - last_telemetry_tick) >= TELEMETRY_SEND_PERIOD_MS)
    {
      last_telemetry_tick = now;
      read_sensors_and_batteries();
      send_telemetry_to_gateway();
    }

    /* 3. Chu kỳ cập nhật LCD */
    if ((now - last_lcd_tick) >= LCD_UPDATE_PERIOD_MS)
    {
      last_lcd_tick = now;
      update_node_lcd();
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

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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

  __HAL_RCC_ADC1_CLK_ENABLE();

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /* Khởi tạo kênh mặc định ADC_CHANNEL_4 */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
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
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{
  __HAL_RCC_I2C2_CLK_ENABLE();

  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  __HAL_RCC_TIM2_CLK_ENABLE();

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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
  __HAL_RCC_USART1_CLK_ENABLE();

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

  HAL_NVIC_SetPriority(USART1_IRQn, 1U, 0U);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* Cấu hình 2 kênh Analog cho Pin 1 (PA4) và Pin 2 (PA5) */
  GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* GPIO I2C2 (PB10, PB11) */
  GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* GPIO USART1 (PA9, PA10) */
  GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* GPIO TIM2 PWM (PA0) */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */