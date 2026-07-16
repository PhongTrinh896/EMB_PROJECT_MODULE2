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
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SLAVE_ADDRESS_LCD 0x4E
#define UPPER_LEVEL 98
#define LOWER_LEVEL 50
#define CUTECOM_COUNTER 6
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint16_t sampling_interval = 10000; // in millisecond
uint8_t motor_state = 0;
volatile uint8_t cutecom_counter;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
// ================== LCD FUNCTIONS ===================
void lcd_put_cur(int row, int col);
void lcd_send_data (char data);
void lcd_send_cmd (char cmd);
void lcd_init (void);
void lcd_send_string (char *str);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// ===================== LORA ===================
#define LORA_BUFFER_SIZE 6
#define LORA_BUFFER_TX_SIZE 3

uint8_t LoRa_TxData[LORA_BUFFER_SIZE];
uint8_t LoRa_RxData[LORA_BUFFER_SIZE];
uint8_t LoRa_RxBuffer; // Biến tạm hứng từng byte từ ngắt UART
uint8_t rx_index = 0;


volatile uint8_t lora_rx_flag = 0;

// ==================== INPUT VARIABLES ====================
// HUM
volatile char str_buffer1[3];
volatile char str_buffer2[3];

// TEMP
volatile char str_buffer3[3];
volatile char str_buffer4[3];


// Ngắt nhận từng byte UART từ LoRa
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) { // Đảm bảo đúng bộ UART nối với LoRa
        LoRa_RxData[rx_index++] = LoRa_RxBuffer;
        if (rx_index >= LORA_BUFFER_SIZE) {
            rx_index = 0;
            lora_rx_flag = 1; // Bật cờ thông báo đã nhận đủ gói tin
        }
        // Tiếp tục kích hoạt ngắt chờ byte tiếp theo (Bắt buộc phải gọi lại)
        HAL_UART_Receive_IT(&huart1, &LoRa_RxBuffer, 1);
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
  MX_USART1_UART_Init();
  MX_I2C2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  cutecom_counter = 0;
  lcd_init ();
  lcd_put_cur(0, 0);
  lcd_send_string ("HUM:");
  lcd_put_cur(1, 0);
  lcd_send_string ("TEM:");

  HAL_UART_Receive_IT(&huart1, &LoRa_RxBuffer, 1);
  uint32_t sampling_time = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // HUM
	  char str_buffer1[3];
	  char str_buffer2[3];

	  // TEMP
	  char str_buffer3[3];
	  char str_buffer4[3];
	  if ((HAL_GetTick() - sampling_time) >= sampling_interval){
		  lora_rx_flag = 0;
		            // Kiểm tra tính toàn vẹn của gói tin (Đúng cấu trúc Header và Footer không)
		  if (LoRa_RxData[0] == 0xAA && LoRa_RxData[7] == 0xBB) {
		                // Nếu lệnh là 0x01, thực hiện nháy LED PB1 tương ứng số lần trong gói tin

			  uint16_t hum = LoRa_RxData[1];
			  uint16_t temp = LoRa_RxData[2];
			  uint16_t battery_hum = LoRa_RxData[3];
			  uint16_t battery_temp = LoRa_RxData[4];
			  motor_state = LoRa_RxData[3];

			  // Chuyển số nguyên ở hệ thập phân (cơ số 10) sang chuỗi
			  itoa(hum, str_buffer1, 10);
			  itoa(battery_hum, str_buffer2, 10);
			  itoa(temp, str_buffer3, 10);
			  itoa(battery_temp, str_buffer4, 10);

		      lcd_put_cur(0, 4);
		      lcd_send_string(str_buffer1);
		      lcd_send_string("%");

		      lcd_put_cur(0, 12);
		      lcd_send_string(str_buffer2);
		      lcd_send_string("%");

		      lcd_put_cur(1, 4);
		      lcd_send_string(str_buffer3);
		      lcd_send_string("%");

		      lcd_put_cur(1, 12);
		      lcd_send_string(str_buffer4);
		      lcd_send_string("%");

		      if (hum <= LOWER_LEVEL){
		    	  // send data to the motor
		    	  LoRa_TxData[0] = 0xAA; // Byte mốc (Header)
		    	  LoRa_TxData[1] = 1; // Lệnh điều khiển: 1 la bat motor, 0 la tat
		    	  LoRa_TxData[2] = 0xBB; // Byte kết thúc (Footer)
		    	  motor_state = 1;
		    	            // Gửi dữ liệu qua mạch LoRa bằng hàm truyền Blocking an toàn
		    	  HAL_UART_Transmit(&huart1, LoRa_TxData, LORA_BUFFER_TX_SIZE, 100);
		    	            // Nháy LED PB3 trên chính board phát để báo hiệu đã bấm gửi thành công
		    	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
		    	  HAL_Delay(100);
		    	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
		      }
		      else if (hum >= UPPER_LEVEL && motor_state == 1){
		    	  motor_state = 0;
		    	  LoRa_TxData[0] = 0xAA; // Byte mốc (Header)
		    	  LoRa_TxData[1] = 0; // Lệnh điều khiển: 1 la bat motor, 0 la tat
		    	  LoRa_TxData[2] = 0xBB; // Byte kết thúc (Footer)
		    	            // Gửi dữ liệu qua mạch LoRa bằng hàm truyền Blocking an toàn
		    	  HAL_UART_Transmit(&huart1, LoRa_TxData, LORA_BUFFER_TX_SIZE, 100);
		    	            // Nháy LED PB3 trên chính board phát để báo hiệu đã bấm gửi thành công
		    	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
		    	  HAL_Delay(100);
		    	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
		      }
		  }

		  sampling_time = HAL_GetTick();
		  if (cutecom_counter == CUTECOM_COUNTER){
			  cutecom_counter = 0;
			  char comp_tx_string[30];
			  sprintf(comp_tx_string, "%s%s | %s%s | %s%s | %s%s", "Do am: ", str_buffer1, "PIN: ", str_buffer2, "Nhiet do: ", str_buffer3, "PIN: ", str_buffer4);
			  HAL_UART_Transmit(&huart2, (uint8_t*)comp_tx_string, strlen(comp_tx_string),100);
		  }
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
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

  /** Initializes the CPU, AHB and APB buses clocks
  */
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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
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
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11; // PB10 = SCL, PB11 = SDA
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;        // Chế độ Open-Drain bắt buộc cho I2C
  GPIO_InitStruct.Pull = GPIO_PULLUP;            // Bật điện trở kéo lên bên trong
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;     // Chọn chức năng I2C2
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  MX_I2C2_Init();
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// ====================== LCD FUNCTIONS ========================
void lcd_send_cmd (char cmd)
{
 char data_u, data_l;
 data_u = (cmd & 0xF0); // extract upper 4 bits
 data_l = ((cmd << 4) & 0xF0); // extract lower 4 bits
 uint8_t data_t[4];
 // send upper 4 bits with enable pulse
 data_t[0] = data_u | 0x0C; // EN=1, RS=0 -> bxxxx1100
 data_t[1] = data_u | 0x08; // EN=0, RS=0 -> bxxxx1000
 // send lower 4 bits with enable pulse
  data_t[2] = data_l | 0x0C; // EN=1, RS=0 -> bxxxx1100
  data_t[3] = data_l | 0x08; // EN=0, RS=0 -> bxxxx1000
  HAL_I2C_Master_Transmit(&hi2c2, SLAVE_ADDRESS_LCD, (uint8_t *) data_t, 4, 100);
}

void lcd_send_data (char data){
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (data&0xf0);
	data_l = ((data<<4)&0xf0);
	data_t[0] = data_u|0x0D; //en=1, rs=1 -> bxxxx1101
	data_t[1] = data_u|0x09; //en=0, rs=1 -> bxxxx1001
	data_t[2] = data_l|0x0D; //en=1, rs=1 -> bxxxx1101
	data_t[3] = data_l|0x09; //en=0, rs=1 -> bxxxx1001
	HAL_I2C_Master_Transmit (&hi2c2, SLAVE_ADDRESS_LCD,(uint8_t *)data_t, 4, 100);
}

void lcd_init (void)
{
 // 4 bit initialisation
 HAL_Delay(50); // wait for >40ms
 lcd_send_cmd (0x30);
 HAL_Delay(5); // wait for >4.1ms
 lcd_send_cmd (0x30);
 HAL_Delay(1); // wait for >100us
 lcd_send_cmd (0x30);
 HAL_Delay(10);
 lcd_send_cmd (0x20); // 4bit mode
 HAL_Delay(10);
  // display initialisation
 lcd_send_cmd (0x28); // Function set --> DL=0 (4 bit mode), N = 1 (2 line display) F = 0(5x8 characters)
 HAL_Delay(1);
 lcd_send_cmd (0x08); //Display on/off control --> D=0,C=0, B=0 ---> display off
  HAL_Delay(1);
  lcd_send_cmd (0x01); // clear display
  HAL_Delay(2);
  lcd_send_cmd (0x06); //Entry mode set --> I/D = 1 (increment cursor) & S = 0 (no shift)
  HAL_Delay(1);
  lcd_send_cmd (0x0C); //Display on/off control --> D = 1, C and B = 0. (Cursor and blink, last two bits)
}

void lcd_send_string (char *str)
{
 while (*str) lcd_send_data (*str++);
}

void lcd_put_cur(int row, int col)
{
 switch (row)
 {
 case 0:
 col |= 0x80;
 break;
 case 1:
 col |= 0xC0;
 break;
 }
 lcd_send_cmd (col);
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
