/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Gateway LoRa
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <ctype.h>
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum
{
    GATEWAY_MODE_AUTO = 0U,
    GATEWAY_MODE_MANUAL
} GatewayMode_t;

typedef struct
{
    uint8_t humidity;
    uint8_t temperature;
    uint8_t battery_1;
    uint8_t battery_2;
    uint8_t motor_state;
} GatewayTelemetry_t;

typedef struct
{
    bool active;
    uint8_t target_motor_state;
    uint8_t retry_count;
    uint32_t last_transmit_tick;
} GatewayCommand_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* Khối cấu hình LCD */

#define LCD_ADDRESS                    0x4EU
#define LCD_COLUMNS                    16U

/* Khối cấu hình giao thức LoRa */

#define LORA_SOF                       0xAAU
#define LORA_EOF                       0xBBU

#define LORA_RX_FRAME_SIZE             8U
#define LORA_TX_FRAME_SIZE             3U

#define RX_INDEX_SOF                   0U
#define RX_INDEX_HUMIDITY              1U
#define RX_INDEX_TEMPERATURE           2U
#define RX_INDEX_BATTERY_1             3U
#define RX_INDEX_BATTERY_2             4U
#define RX_INDEX_MOTOR                 5U
#define RX_INDEX_CHECKSUM              6U
#define RX_INDEX_EOF                   7U

/* Khối cấu hình điều khiển motor */

#define MOTOR_OFF                      0U
#define MOTOR_ON                       1U

#define HUMIDITY_LOWER_LEVEL           50U
#define HUMIDITY_UPPER_LEVEL           98U

/* Khối cấu hình thời gian */

#define LCD_UPDATE_PERIOD_MS           500U
#define LINK_TIMEOUT_MS                6000U

#define COMMAND_RETRY_PERIOD_MS        1000U
#define COMMAND_MAX_RETRIES            3U

#define EMERGENCY_DEBOUNCE_MS          50U
#define STATUS_LED_PULSE_MS            100U

/* Khối cấu hình UART máy tính */

#define PC_LINE_BUFFER_SIZE            48U

/* Khối cấu hình chân phần cứng */

#define STATUS_LED_GPIO_PORT           GPIOB
#define STATUS_LED_PIN                 GPIO_PIN_3

#define ESTOP_GPIO_PORT                GPIOB
#define ESTOP_PIN                      GPIO_PIN_0
#define ESTOP_IRQn                     EXTI0_IRQn

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* Khối lưu dữ liệu Node */

static GatewayTelemetry_t telemetry =
{
    .humidity = 0U,
    .temperature = 0U,
    .battery_1 = 0U,
    .battery_2 = 0U,
    .motor_state = MOTOR_OFF
};

static bool telemetry_valid = false;

static uint32_t last_telemetry_tick = 0U;

static uint32_t valid_frame_count = 0U;
static uint32_t invalid_frame_count = 0U;
static uint32_t format_error_count = 0U;

/* Khối quản lý trạng thái Gateway */

static GatewayMode_t gateway_mode = GATEWAY_MODE_AUTO;

static bool emergency_latched = false;
static bool link_lost_reported = false;

/* Khối quản lý lệnh motor */

static GatewayCommand_t pending_command =
{
    .active = false,
    .target_motor_state = MOTOR_OFF,
    .retry_count = 0U,
    .last_transmit_tick = 0U
};

/* Khối nhận dữ liệu LoRa */

static volatile uint8_t lora_rx_byte = 0U;

static uint8_t lora_build_frame[LORA_RX_FRAME_SIZE] = {0U};

static volatile uint8_t lora_build_index = 0U;

static volatile uint8_t
lora_ready_frame[LORA_RX_FRAME_SIZE] = {0U};

static volatile uint8_t lora_frame_ready = 0U;

static volatile uint32_t lora_dropped_frame_count = 0U;
static volatile uint32_t lora_uart_error_count = 0U;

/* Khối nhận lệnh từ CuteCom */

static volatile uint8_t pc_rx_byte = 0U;

static char pc_build_line[PC_LINE_BUFFER_SIZE] = {0};

static volatile uint8_t pc_build_index = 0U;

static volatile char
pc_ready_line[PC_LINE_BUFFER_SIZE] = {0};

static volatile uint8_t pc_line_ready = 0U;

static volatile uint32_t pc_dropped_line_count = 0U;

/* Khối xử lý nút dừng khẩn cấp */

static volatile bool emergency_irq_pending = false;

static uint32_t last_emergency_tick = 0U;

/* Khối cập nhật LCD và LED */

static uint32_t last_lcd_tick = 0U;

static bool status_led_active = false;

static uint32_t status_led_start_tick = 0U;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

void SystemClock_Config(void);

static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */

/* Khối hàm LCD */

static void lcd_send_cmd(char cmd);
static void lcd_send_data(char data);
static void lcd_init(void);
static void lcd_send_string(const char *text);
static void lcd_put_cur(uint8_t row, uint8_t column);
static void lcd_print_line(uint8_t row, const char *text);

/* Khối hàm giao thức */

static uint8_t calculate_checksum(
    const uint8_t *data,
    uint8_t length
);

static bool validate_lora_frame(
    const uint8_t frame[LORA_RX_FRAME_SIZE]
);

/* Khối hàm nhận dữ liệu */

static bool take_lora_frame(
    uint8_t frame[LORA_RX_FRAME_SIZE]
);

static bool take_pc_line(
    char line[PC_LINE_BUFFER_SIZE]
);

static void start_uart_reception(void);

/* Khối hàm UART máy tính */

static void pc_write(const char *text);

static void send_status_to_pc(void);

static void convert_to_uppercase(char *text);

static void process_pc_command(char *command);

/* Khối hàm điều khiển motor */

static HAL_StatusTypeDef transmit_motor_command(
    uint8_t motor_state
);

static bool request_motor_state(
    uint8_t motor_state,
    bool replace_pending
);

static void confirm_motor_command(void);

static void service_motor_command(void);

/* Khối hàm xử lý dữ liệu Node */

static void process_lora_frame(
    const uint8_t frame[LORA_RX_FRAME_SIZE]
);

/* Khối hàm điều khiển hệ thống */

static bool node_link_alive(void);

static void apply_auto_control(void);

static void activate_emergency_stop(void);

static void clear_emergency_stop(void);

static void service_emergency_event(void);

static void service_link_status(void);

/* Khối hàm hiển thị */

static void update_lcd(void);

static void pulse_status_led(void);

static void service_status_led(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Khối callback nhận UART */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1)
    {
        uint8_t received_byte = lora_rx_byte;

        if (lora_build_index == 0U)
        {
            if (received_byte == LORA_SOF)
            {
                lora_build_frame[0] = received_byte;
                lora_build_index = 1U;
            }
        }
        else
        {
            if ((received_byte == LORA_SOF) &&
                (lora_build_index != RX_INDEX_CHECKSUM))
            {
                lora_build_frame[0] = received_byte;
                lora_build_index = 1U;
            }
            else
            {
                if (lora_build_index < LORA_RX_FRAME_SIZE)
                {
                    lora_build_frame[lora_build_index] =
                        received_byte;

                    lora_build_index++;
                }

                if (lora_build_index >= LORA_RX_FRAME_SIZE)
                {
                    if (lora_build_frame[RX_INDEX_EOF] ==
                        LORA_EOF)
                    {
                        if (lora_frame_ready == 0U)
                        {
                            for (uint8_t index = 0U;
                                 index < LORA_RX_FRAME_SIZE;
                                 index++)
                            {
                                lora_ready_frame[index] =
                                    lora_build_frame[index];
                            }

                            lora_frame_ready = 1U;
                        }
                        else
                        {
                            lora_dropped_frame_count++;
                        }
                    }
                    else
                    {
                        format_error_count++;
                    }

                    lora_build_index = 0U;
                }
            }
        }

        (void)HAL_UART_Receive_IT(
            &huart1,
            (uint8_t *)&lora_rx_byte,
            1U
        );
    }
    else if (huart == &huart2)
    {
        uint8_t received_byte = pc_rx_byte;

        if ((received_byte == '\r') ||
            (received_byte == '\n'))
        {
            if (pc_build_index > 0U)
            {
                pc_build_line[pc_build_index] = '\0';

                if (pc_line_ready == 0U)
                {
                    for (uint8_t index = 0U;
                         index <= pc_build_index;
                         index++)
                    {
                        pc_ready_line[index] =
                            pc_build_line[index];
                    }

                    pc_line_ready = 1U;
                }
                else
                {
                    pc_dropped_line_count++;
                }

                pc_build_index = 0U;
            }
        }
        else if ((received_byte >= 32U) &&
                 (received_byte <= 126U))
        {
            if (pc_build_index <
                (PC_LINE_BUFFER_SIZE - 1U))
            {
                pc_build_line[pc_build_index] =
                    (char)received_byte;

                pc_build_index++;
            }
            else
            {
                pc_build_index = 0U;
            }
        }

        (void)HAL_UART_Receive_IT(
            &huart2,
            (uint8_t *)&pc_rx_byte,
            1U
        );
    }
}

/* Khối callback xử lý lỗi UART */

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1)
    {
        lora_uart_error_count++;
        lora_build_index = 0U;

        __HAL_UART_CLEAR_OREFLAG(&huart1);
        __HAL_UART_CLEAR_NEFLAG(&huart1);
        __HAL_UART_CLEAR_FEFLAG(&huart1);
        __HAL_UART_CLEAR_PEFLAG(&huart1);

        (void)HAL_UART_Receive_IT(
            &huart1,
            (uint8_t *)&lora_rx_byte,
            1U
        );
    }
    else if (huart == &huart2)
    {
        pc_build_index = 0U;

        __HAL_UART_CLEAR_OREFLAG(&huart2);
        __HAL_UART_CLEAR_NEFLAG(&huart2);
        __HAL_UART_CLEAR_FEFLAG(&huart2);
        __HAL_UART_CLEAR_PEFLAG(&huart2);

        (void)HAL_UART_Receive_IT(
            &huart2,
            (uint8_t *)&pc_rx_byte,
            1U
        );
    }
}

/* Khối callback nút dừng khẩn cấp */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == ESTOP_PIN)
    {
        emergency_irq_pending = true;
    }
}

/* Khối tính checksum */

static uint8_t calculate_checksum(
    const uint8_t *data,
    uint8_t length
)
{
    uint8_t checksum = 0U;

    if (data == NULL)
    {
        return 0U;
    }

    for (uint8_t index = 0U;
         index < length;
         index++)
    {
        checksum ^= data[index];
    }

    return checksum;
}

/* Khối kiểm tra frame LoRa */

static bool validate_lora_frame(
    const uint8_t frame[LORA_RX_FRAME_SIZE]
)
{
    if (frame == NULL)
    {
        return false;
    }

    if (frame[RX_INDEX_SOF] != LORA_SOF)
    {
        return false;
    }

    if (frame[RX_INDEX_EOF] != LORA_EOF)
    {
        return false;
    }

    if (frame[RX_INDEX_HUMIDITY] > 100U)
    {
        return false;
    }

    if (frame[RX_INDEX_TEMPERATURE] > 100U)
    {
        return false;
    }

    if (frame[RX_INDEX_BATTERY_1] > 100U)
    {
        return false;
    }

    if (frame[RX_INDEX_BATTERY_2] > 100U)
    {
        return false;
    }

    if (frame[RX_INDEX_MOTOR] > MOTOR_ON)
    {
        return false;
    }

    uint8_t checksum =
        calculate_checksum(
            &frame[RX_INDEX_HUMIDITY],
            5U
        );

    if (checksum != frame[RX_INDEX_CHECKSUM])
    {
        return false;
    }

    return true;
}

/* Khối sao chép frame từ ngắt sang vòng lặp chính */

static bool take_lora_frame(
    uint8_t frame[LORA_RX_FRAME_SIZE]
)
{
    bool available = false;

    uint32_t interrupt_state = __get_PRIMASK();

    __disable_irq();

    if (lora_frame_ready != 0U)
    {
        for (uint8_t index = 0U;
             index < LORA_RX_FRAME_SIZE;
             index++)
        {
            frame[index] = lora_ready_frame[index];
        }

        lora_frame_ready = 0U;
        available = true;
    }

    if (interrupt_state == 0U)
    {
        __enable_irq();
    }

    return available;
}

/* Khối sao chép lệnh CuteCom sang vòng lặp chính */

static bool take_pc_line(
    char line[PC_LINE_BUFFER_SIZE]
)
{
    bool available = false;

    uint32_t interrupt_state = __get_PRIMASK();

    __disable_irq();

    if (pc_line_ready != 0U)
    {
        for (uint8_t index = 0U;
             index < PC_LINE_BUFFER_SIZE;
             index++)
        {
            line[index] = pc_ready_line[index];

            if (line[index] == '\0')
            {
                break;
            }
        }

        line[PC_LINE_BUFFER_SIZE - 1U] = '\0';

        pc_line_ready = 0U;
        available = true;
    }

    if (interrupt_state == 0U)
    {
        __enable_irq();
    }

    return available;
}

/* Khối khởi động nhận UART */

static void start_uart_reception(void)
{
    if (HAL_UART_Receive_IT(
            &huart1,
            (uint8_t *)&lora_rx_byte,
            1U
        ) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_UART_Receive_IT(
            &huart2,
            (uint8_t *)&pc_rx_byte,
            1U
        ) != HAL_OK)
    {
        Error_Handler();
    }
}

/* Khối gửi chuỗi lên CuteCom */

static void pc_write(const char *text)
{
    if (text == NULL)
    {
        return;
    }

    (void)HAL_UART_Transmit(
        &huart2,
        (uint8_t *)text,
        (uint16_t)strlen(text),
        200U
    );
}

/* Khối điều khiển LED trạng thái */

static void pulse_status_led(void)
{
    HAL_GPIO_WritePin(
        STATUS_LED_GPIO_PORT,
        STATUS_LED_PIN,
        GPIO_PIN_SET
    );

    status_led_active = true;
    status_led_start_tick = HAL_GetTick();
}

static void service_status_led(void)
{
    if (!status_led_active)
    {
        return;
    }

    if ((uint32_t)(
            HAL_GetTick() -
            status_led_start_tick
        ) >= STATUS_LED_PULSE_MS)
    {
        HAL_GPIO_WritePin(
            STATUS_LED_GPIO_PORT,
            STATUS_LED_PIN,
            GPIO_PIN_RESET
        );

        status_led_active = false;
    }
}

/* Khối truyền lệnh motor cho Node */

static HAL_StatusTypeDef transmit_motor_command(
    uint8_t motor_state
)
{
    if (motor_state > MOTOR_ON)
    {
        return HAL_ERROR;
    }

    uint8_t frame[LORA_TX_FRAME_SIZE];

    frame[0] = LORA_SOF;
    frame[1] = motor_state;
    frame[2] = LORA_EOF;

    HAL_StatusTypeDef status =
        HAL_UART_Transmit(
            &huart1,
            frame,
            LORA_TX_FRAME_SIZE,
            100U
        );

    if (status == HAL_OK)
    {
        pulse_status_led();
    }

    return status;
}

/* Khối tạo yêu cầu thay đổi trạng thái motor */

static bool request_motor_state(
    uint8_t motor_state,
    bool replace_pending
)
{
    if (motor_state > MOTOR_ON)
    {
        return false;
    }

    if (emergency_latched &&
        (motor_state == MOTOR_ON))
    {
        pc_write(
            "ERROR: ESTOP ACTIVE\r\n"
        );

        return false;
    }

    if (pending_command.active &&
        !replace_pending)
    {
        if (pending_command.target_motor_state ==
            motor_state)
        {
            return true;
        }

        return false;
    }

    if (!replace_pending &&
        telemetry_valid &&
        telemetry.motor_state == motor_state)
    {
        return true;
    }

    if (transmit_motor_command(motor_state) != HAL_OK)
    {
        pc_write(
            "ERROR: MOTOR COMMAND TRANSMIT FAILED\r\n"
        );

        return false;
    }

    pending_command.active = true;

    pending_command.target_motor_state =
        motor_state;

    pending_command.retry_count = 0U;

    pending_command.last_transmit_tick =
        HAL_GetTick();

    char message[64];

    (void)snprintf(
        message,
        sizeof(message),
        "TX MOTOR=%s\r\n",
        (motor_state == MOTOR_ON) ?
        "ON" :
        "OFF"
    );

    pc_write(message);

    return true;
}

/* Khối xác nhận Node đã thực hiện lệnh motor */

static void confirm_motor_command(void)
{
    if (!pending_command.active)
    {
        return;
    }

    if (telemetry.motor_state ==
        pending_command.target_motor_state)
    {
        char message[64];

        (void)snprintf(
            message,
            sizeof(message),
            "ACK MOTOR=%s\r\n",
            (pending_command.target_motor_state ==
             MOTOR_ON) ?
            "ON" :
            "OFF"
        );

        pc_write(message);

        pending_command.active = false;
        pending_command.retry_count = 0U;
    }
}

/* Khối gửi lại lệnh khi chưa nhận xác nhận */

static void service_motor_command(void)
{
    if (!pending_command.active)
    {
        return;
    }

    uint32_t now = HAL_GetTick();

    if ((uint32_t)(
            now -
            pending_command.last_transmit_tick
        ) < COMMAND_RETRY_PERIOD_MS)
    {
        return;
    }

    if (pending_command.retry_count >=
        COMMAND_MAX_RETRIES)
    {
        pc_write(
            "ERROR: MOTOR COMMAND TIMEOUT\r\n"
        );

        pending_command.active = false;
        pending_command.retry_count = 0U;

        return;
    }

    if (transmit_motor_command(
            pending_command.target_motor_state
        ) == HAL_OK)
    {
        pending_command.retry_count++;

        pending_command.last_transmit_tick = now;

        char message[64];

        (void)snprintf(
            message,
            sizeof(message),
            "RETRY MOTOR=%s COUNT=%u\r\n",
            (pending_command.target_motor_state ==
             MOTOR_ON) ?
            "ON" :
            "OFF",
            pending_command.retry_count
        );

        pc_write(message);
    }
}

/* Khối xử lý dữ liệu nhận từ Node */

static void process_lora_frame(
    const uint8_t frame[LORA_RX_FRAME_SIZE]
)
{
    if (!validate_lora_frame(frame))
    {
        invalid_frame_count++;

        pc_write(
            "DROP: INVALID LORA FRAME\r\n"
        );

        return;
    }

    telemetry.humidity =
        frame[RX_INDEX_HUMIDITY];

    telemetry.temperature =
        frame[RX_INDEX_TEMPERATURE];

    telemetry.battery_1 =
        frame[RX_INDEX_BATTERY_1];

    telemetry.battery_2 =
        frame[RX_INDEX_BATTERY_2];

    telemetry.motor_state =
        frame[RX_INDEX_MOTOR];

    telemetry_valid = true;

    last_telemetry_tick = HAL_GetTick();

    valid_frame_count++;

    if (link_lost_reported)
    {
        link_lost_reported = false;

        pc_write(
            "NODE LINK RESTORED\r\n"
        );
    }

    confirm_motor_command();

    send_status_to_pc();
}

/* Khối kiểm tra kết nối Node */

static bool node_link_alive(void)
{
    if (!telemetry_valid)
    {
        return false;
    }

    return (
        (uint32_t)(
            HAL_GetTick() -
            last_telemetry_tick
        ) <= LINK_TIMEOUT_MS
    );
}

/* Khối điều khiển tự động theo độ ẩm */

static void apply_auto_control(void)
{
    if (gateway_mode != GATEWAY_MODE_AUTO)
    {
        return;
    }

    if (emergency_latched)
    {
        return;
    }

    if (!node_link_alive())
    {
        return;
    }

    if (pending_command.active)
    {
        return;
    }

    if ((telemetry.humidity <=
         HUMIDITY_LOWER_LEVEL) &&
        (telemetry.motor_state == MOTOR_OFF))
    {
        (void)request_motor_state(
            MOTOR_ON,
            false
        );
    }
    else if ((telemetry.humidity >=
              HUMIDITY_UPPER_LEVEL) &&
             (telemetry.motor_state == MOTOR_ON))
    {
        (void)request_motor_state(
            MOTOR_OFF,
            false
        );
    }
}

/* Khối kích hoạt dừng khẩn cấp */

static void activate_emergency_stop(void)
{
    emergency_latched = true;

    gateway_mode = GATEWAY_MODE_MANUAL;

    (void)request_motor_state(
        MOTOR_OFF,
        true
    );

    pc_write(
        "EMERGENCY STOP LATCHED\r\n"
    );
}

/* Khối xóa dừng khẩn cấp */

static void clear_emergency_stop(void)
{
    emergency_latched = false;

    gateway_mode = GATEWAY_MODE_AUTO;

    pc_write(
        "EMERGENCY CLEARED\r\n"
    );

    apply_auto_control();
}

/* Khối xử lý sự kiện nút dừng khẩn cấp */

static void service_emergency_event(void)
{
    if (!emergency_irq_pending)
    {
        return;
    }

    emergency_irq_pending = false;

    uint32_t now = HAL_GetTick();

    if ((uint32_t)(
            now -
            last_emergency_tick
        ) < EMERGENCY_DEBOUNCE_MS)
    {
        return;
    }

    last_emergency_tick = now;

    activate_emergency_stop();
}

/* Khối kiểm tra mất kết nối */

static void service_link_status(void)
{
    if (node_link_alive())
    {
        return;
    }

    if (telemetry_valid &&
        !link_lost_reported)
    {
        link_lost_reported = true;

        pc_write(
            "ERROR: NODE LINK TIMEOUT\r\n"
        );
    }
}

/* Khối chuyển lệnh CuteCom thành chữ hoa */

static void convert_to_uppercase(char *text)
{
    if (text == NULL)
    {
        return;
    }

    while (*text != '\0')
    {
        *text =
            (char)toupper(
                (unsigned char)*text
            );

        text++;
    }
}

/* Khối gửi trạng thái hệ thống lên CuteCom */

static void send_status_to_pc(void)
{
    if (!telemetry_valid)
    {
        pc_write(
            "STATUS: NO TELEMETRY\r\n"
        );

        return;
    }

    char message[220];

    int length =
        snprintf(
            message,
            sizeof(message),
            "HUM=%u%% | TEMP=%uC | BAT1=%u%% | "
            "BAT2=%u%% | MOTOR=%s | MODE=%s | "
            "ESTOP=%u | VALID=%lu | INVALID=%lu | "
            "FORMAT=%lu | DROP=%lu | UARTERR=%lu\r\n",
            telemetry.humidity,
            telemetry.temperature,
            telemetry.battery_1,
            telemetry.battery_2,
            (telemetry.motor_state == MOTOR_ON) ?
            "ON" :
            "OFF",
            (gateway_mode == GATEWAY_MODE_AUTO) ?
            "AUTO" :
            "MANUAL",
            emergency_latched ? 1U : 0U,
            (unsigned long)valid_frame_count,
            (unsigned long)invalid_frame_count,
            (unsigned long)format_error_count,
            (unsigned long)lora_dropped_frame_count,
            (unsigned long)lora_uart_error_count
        );

    if (length > 0)
    {
        uint16_t transmit_length =
            (length < (int)sizeof(message)) ?
            (uint16_t)length :
            (uint16_t)(sizeof(message) - 1U);

        (void)HAL_UART_Transmit(
            &huart2,
            (uint8_t *)message,
            transmit_length,
            200U
        );
    }
}

/* Khối xử lý lệnh từ CuteCom */

static void process_pc_command(char *command)
{
    if (command == NULL)
    {
        return;
    }

    convert_to_uppercase(command);

    if (strcmp(command, "AUTO") == 0)
    {
        if (emergency_latched)
        {
            pc_write(
                "ERROR: CLEAR ESTOP FIRST\r\n"
            );
        }
        else
        {
            gateway_mode = GATEWAY_MODE_AUTO;

            pc_write(
                "MODE=AUTO\r\n"
            );

            apply_auto_control();
        }
    }
    else if ((strcmp(command, "ON") == 0) ||
             (strcmp(command, "MOTOR ON") == 0))
    {
        if (emergency_latched)
        {
            pc_write(
                "ERROR: ESTOP ACTIVE\r\n"
            );
        }
        else
        {
            gateway_mode =
                GATEWAY_MODE_MANUAL;

            (void)request_motor_state(
                MOTOR_ON,
                true
            );
        }
    }
    else if ((strcmp(command, "OFF") == 0) ||
             (strcmp(command, "MOTOR OFF") == 0))
    {
        gateway_mode =
            GATEWAY_MODE_MANUAL;

        (void)request_motor_state(
            MOTOR_OFF,
            true
        );
    }
    else if (strcmp(command, "STOP") == 0)
    {
        activate_emergency_stop();
    }
    else if (strcmp(command, "CLEAR") == 0)
    {
        clear_emergency_stop();
    }
    else if (strcmp(command, "STATUS") == 0)
    {
        send_status_to_pc();
    }
    else
    {
        pc_write(
            "COMMANDS: AUTO | ON | OFF | STOP | "
            "CLEAR | STATUS\r\n"
        );
    }
}

/* Khối cập nhật LCD */

static void update_lcd(void)
{
    char line_1[LCD_COLUMNS + 1U];
    char line_2[LCD_COLUMNS + 1U];

    if (!node_link_alive())
    {
        (void)snprintf(
            line_1,
            sizeof(line_1),
            "NODE NO SIGNAL"
        );

        if (emergency_latched)
        {
            (void)snprintf(
                line_2,
                sizeof(line_2),
                "ESTOP LATCHED"
            );
        }
        else if (pending_command.active)
        {
            (void)snprintf(
                line_2,
                sizeof(line_2),
                "WAIT MOTOR ACK"
            );
        }
        else
        {
            (void)snprintf(
                line_2,
                sizeof(line_2),
                "CHECK LORA"
            );
        }

        lcd_print_line(0U, line_1);
        lcd_print_line(1U, line_2);

        return;
    }

    (void)snprintf(
        line_1,
        sizeof(line_1),
        "H:%3u%% T:%3uC",
        telemetry.humidity,
        telemetry.temperature
    );

    if (emergency_latched)
    {
        (void)snprintf(
            line_2,
            sizeof(line_2),
            "ESTOP B:%3u%%",
            telemetry.battery_1
        );
    }
    else
    {
        (void)snprintf(
            line_2,
            sizeof(line_2),
            "B:%3u M:%s %c",
            telemetry.battery_1,
            (telemetry.motor_state == MOTOR_ON) ?
            "ON" :
            "OFF",
            (gateway_mode == GATEWAY_MODE_AUTO) ?
            'A' :
            'M'
        );
    }

    lcd_print_line(0U, line_1);
    lcd_print_line(1U, line_2);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    /* USER CODE BEGIN 1 */

    uint8_t received_frame[LORA_RX_FRAME_SIZE];

    char received_pc_line[PC_LINE_BUFFER_SIZE];

    /* USER CODE END 1 */

    HAL_Init();

    SystemClock_Config();

    MX_GPIO_Init();
    MX_USART1_UART_Init();
    MX_I2C2_Init();
    MX_USART2_UART_Init();

    /* USER CODE BEGIN 2 */

    lcd_init();

    lcd_print_line(
        0U,
        "GATEWAY START"
    );

    lcd_print_line(
        1U,
        "WAIT NODE"
    );

    HAL_GPIO_WritePin(
        STATUS_LED_GPIO_PORT,
        STATUS_LED_PIN,
        GPIO_PIN_RESET
    );

    start_uart_reception();

    last_lcd_tick = HAL_GetTick();

    last_emergency_tick =
        HAL_GetTick() -
        EMERGENCY_DEBOUNCE_MS;

    pc_write(
        "\r\nGATEWAY READY\r\n"
        "COMMANDS: AUTO | ON | OFF | STOP | "
        "CLEAR | STATUS\r\n"
    );

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */

    while (1)
    {
        if (take_lora_frame(received_frame))
        {
            process_lora_frame(received_frame);
        }

        if (take_pc_line(received_pc_line))
        {
            process_pc_command(received_pc_line);
        }

        service_emergency_event();

        service_motor_command();

        apply_auto_control();

        service_link_status();

        if ((uint32_t)(
                HAL_GetTick() -
                last_lcd_tick
            ) >= LCD_UPDATE_PERIOD_MS)
        {
            last_lcd_tick = HAL_GetTick();

            update_lcd();
        }

        service_status_led();

        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }

    /* USER CODE END 3 */
}

/* Khối cấu hình clock */

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};

    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();

    __HAL_PWR_VOLTAGESCALING_CONFIG(
        PWR_REGULATOR_VOLTAGE_SCALE1
    );

    RCC_OscInitStruct.OscillatorType =
        RCC_OSCILLATORTYPE_HSI;

    RCC_OscInitStruct.HSIState =
        RCC_HSI_ON;

    RCC_OscInitStruct.HSICalibrationValue =
        RCC_HSICALIBRATION_DEFAULT;

    RCC_OscInitStruct.PLL.PLLState =
        RCC_PLL_ON;

    RCC_OscInitStruct.PLL.PLLSource =
        RCC_PLLSOURCE_HSI;

    RCC_OscInitStruct.PLL.PLLM = 8;

    RCC_OscInitStruct.PLL.PLLN = 168;

    RCC_OscInitStruct.PLL.PLLP =
        RCC_PLLP_DIV2;

    RCC_OscInitStruct.PLL.PLLQ = 4;

    if (HAL_RCC_OscConfig(
            &RCC_OscInitStruct
        ) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType =
        RCC_CLOCKTYPE_HCLK |
        RCC_CLOCKTYPE_SYSCLK |
        RCC_CLOCKTYPE_PCLK1 |
        RCC_CLOCKTYPE_PCLK2;

    RCC_ClkInitStruct.SYSCLKSource =
        RCC_SYSCLKSOURCE_PLLCLK;

    RCC_ClkInitStruct.AHBCLKDivider =
        RCC_SYSCLK_DIV1;

    RCC_ClkInitStruct.APB1CLKDivider =
        RCC_HCLK_DIV4;

    RCC_ClkInitStruct.APB2CLKDivider =
        RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(
            &RCC_ClkInitStruct,
            FLASH_LATENCY_5
        ) != HAL_OK)
    {
        Error_Handler();
    }
}

/* Khối cấu hình I2C2 */

static void MX_I2C2_Init(void)
{
    __HAL_RCC_I2C2_CLK_ENABLE();

    hi2c2.Instance = I2C2;

    hi2c2.Init.ClockSpeed = 100000;

    hi2c2.Init.DutyCycle =
        I2C_DUTYCYCLE_2;

    hi2c2.Init.OwnAddress1 = 0;

    hi2c2.Init.AddressingMode =
        I2C_ADDRESSINGMODE_7BIT;

    hi2c2.Init.DualAddressMode =
        I2C_DUALADDRESS_DISABLE;

    hi2c2.Init.OwnAddress2 = 0;

    hi2c2.Init.GeneralCallMode =
        I2C_GENERALCALL_DISABLE;

    hi2c2.Init.NoStretchMode =
        I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(&hi2c2) != HAL_OK)
    {
        Error_Handler();
    }
}

/* Khối cấu hình USART1 cho LoRa */

static void MX_USART1_UART_Init(void)
{
    __HAL_RCC_USART1_CLK_ENABLE();

    huart1.Instance = USART1;

    huart1.Init.BaudRate = 115200;

    huart1.Init.WordLength =
        UART_WORDLENGTH_8B;

    huart1.Init.StopBits =
        UART_STOPBITS_1;

    huart1.Init.Parity =
        UART_PARITY_NONE;

    huart1.Init.Mode =
        UART_MODE_TX_RX;

    huart1.Init.HwFlowCtl =
        UART_HWCONTROL_NONE;

    huart1.Init.OverSampling =
        UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_NVIC_SetPriority(
        USART1_IRQn,
        1U,
        0U
    );

    HAL_NVIC_EnableIRQ(
        USART1_IRQn
    );
}

/* Khối cấu hình USART2 cho CuteCom */

static void MX_USART2_UART_Init(void)
{
    __HAL_RCC_USART2_CLK_ENABLE();

    huart2.Instance = USART2;

    huart2.Init.BaudRate = 115200;

    huart2.Init.WordLength =
        UART_WORDLENGTH_8B;

    huart2.Init.StopBits =
        UART_STOPBITS_1;

    huart2.Init.Parity =
        UART_PARITY_NONE;

    huart2.Init.Mode =
        UART_MODE_TX_RX;

    huart2.Init.HwFlowCtl =
        UART_HWCONTROL_NONE;

    huart2.Init.OverSampling =
        UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_NVIC_SetPriority(
        USART2_IRQn,
        2U,
        0U
    );

    HAL_NVIC_EnableIRQ(
        USART2_IRQn
    );
}

/* Khối cấu hình GPIO */

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    HAL_GPIO_WritePin(
        STATUS_LED_GPIO_PORT,
        STATUS_LED_PIN,
        GPIO_PIN_RESET
    );

    GPIO_InitStruct.Pin =
        STATUS_LED_PIN;

    GPIO_InitStruct.Mode =
        GPIO_MODE_OUTPUT_PP;

    GPIO_InitStruct.Pull =
        GPIO_NOPULL;

    GPIO_InitStruct.Speed =
        GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(
        STATUS_LED_GPIO_PORT,
        &GPIO_InitStruct
    );

    GPIO_InitStruct.Pin =
        GPIO_PIN_10 |
        GPIO_PIN_11;

    GPIO_InitStruct.Mode =
        GPIO_MODE_AF_OD;

    GPIO_InitStruct.Pull =
        GPIO_PULLUP;

    GPIO_InitStruct.Speed =
        GPIO_SPEED_FREQ_VERY_HIGH;

    GPIO_InitStruct.Alternate =
        GPIO_AF4_I2C2;

    HAL_GPIO_Init(
        GPIOB,
        &GPIO_InitStruct
    );

    GPIO_InitStruct.Pin =
        GPIO_PIN_9 |
        GPIO_PIN_10;

    GPIO_InitStruct.Mode =
        GPIO_MODE_AF_PP;

    GPIO_InitStruct.Pull =
        GPIO_PULLUP;

    GPIO_InitStruct.Speed =
        GPIO_SPEED_FREQ_VERY_HIGH;

    GPIO_InitStruct.Alternate =
        GPIO_AF7_USART1;

    HAL_GPIO_Init(
        GPIOA,
        &GPIO_InitStruct
    );

    GPIO_InitStruct.Pin =
        GPIO_PIN_2 |
        GPIO_PIN_3;

    GPIO_InitStruct.Mode =
        GPIO_MODE_AF_PP;

    GPIO_InitStruct.Pull =
        GPIO_PULLUP;

    GPIO_InitStruct.Speed =
        GPIO_SPEED_FREQ_VERY_HIGH;

    GPIO_InitStruct.Alternate =
        GPIO_AF7_USART2;

    HAL_GPIO_Init(
        GPIOA,
        &GPIO_InitStruct
    );

    GPIO_InitStruct.Pin =
        ESTOP_PIN;

    GPIO_InitStruct.Mode =
        GPIO_MODE_IT_FALLING;

    GPIO_InitStruct.Pull =
        GPIO_PULLUP;

    HAL_GPIO_Init(
        ESTOP_GPIO_PORT,
        &GPIO_InitStruct
    );

    HAL_NVIC_SetPriority(
        ESTOP_IRQn,
        0U,
        0U
    );

    HAL_NVIC_EnableIRQ(
        ESTOP_IRQn
    );
}

/* USER CODE BEGIN 4 */

/* Khối điều khiển LCD */

static void lcd_send_cmd(char cmd)
{
    char upper_data =
        (char)(cmd & 0xF0);

    char lower_data =
        (char)((cmd << 4) & 0xF0);

    uint8_t transmit_data[4];

    transmit_data[0] =
        (uint8_t)upper_data | 0x0CU;

    transmit_data[1] =
        (uint8_t)upper_data | 0x08U;

    transmit_data[2] =
        (uint8_t)lower_data | 0x0CU;

    transmit_data[3] =
        (uint8_t)lower_data | 0x08U;

    (void)HAL_I2C_Master_Transmit(
        &hi2c2,
        LCD_ADDRESS,
        transmit_data,
        4U,
        100U
    );
}

static void lcd_send_data(char data)
{
    char upper_data =
        (char)(data & 0xF0);

    char lower_data =
        (char)((data << 4) & 0xF0);

    uint8_t transmit_data[4];

    transmit_data[0] =
        (uint8_t)upper_data | 0x0DU;

    transmit_data[1] =
        (uint8_t)upper_data | 0x09U;

    transmit_data[2] =
        (uint8_t)lower_data | 0x0DU;

    transmit_data[3] =
        (uint8_t)lower_data | 0x09U;

    (void)HAL_I2C_Master_Transmit(
        &hi2c2,
        LCD_ADDRESS,
        transmit_data,
        4U,
        100U
    );
}

static void lcd_init(void)
{
    HAL_Delay(50U);

    lcd_send_cmd(0x30);
    HAL_Delay(5U);

    lcd_send_cmd(0x30);
    HAL_Delay(1U);

    lcd_send_cmd(0x30);
    HAL_Delay(10U);

    lcd_send_cmd(0x20);
    HAL_Delay(10U);

    lcd_send_cmd(0x28);
    HAL_Delay(1U);

    lcd_send_cmd(0x08);
    HAL_Delay(1U);

    lcd_send_cmd(0x01);
    HAL_Delay(2U);

    lcd_send_cmd(0x06);
    HAL_Delay(1U);

    lcd_send_cmd(0x0C);
}

static void lcd_send_string(const char *text)
{
    if (text == NULL)
    {
        return;
    }

    while (*text != '\0')
    {
        lcd_send_data(*text);

        text++;
    }
}

static void lcd_put_cur(
    uint8_t row,
    uint8_t column
)
{
    uint8_t address;

    if (row == 0U)
    {
        address =
            (uint8_t)(0x80U | column);
    }
    else
    {
        address =
            (uint8_t)(0xC0U | column);
    }

    lcd_send_cmd((char)address);
}

static void lcd_print_line(
    uint8_t row,
    const char *text
)
{
    char line[LCD_COLUMNS + 1U];

    (void)memset(
        line,
        ' ',
        LCD_COLUMNS
    );

    line[LCD_COLUMNS] = '\0';

    if (text != NULL)
    {
        size_t text_length = strlen(text);

        size_t copy_length =
            (text_length < LCD_COLUMNS) ?
            text_length :
            LCD_COLUMNS;

        (void)memcpy(
            line,
            text,
            copy_length
        );
    }

    lcd_put_cur(row, 0U);

    lcd_send_string(line);
}

/* USER CODE END 4 */

/* Khối xử lý lỗi hệ thống */

void Error_Handler(void)
{
    __disable_irq();

    while (1)
    {
    }
}

#ifdef USE_FULL_ASSERT

void assert_failed(
    uint8_t *file,
    uint32_t line
)
{
    (void)file;
    (void)line;
}

#endif
