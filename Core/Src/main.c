#include "main.h"
#include "vl53l0x_api.h"
#include <stdio.h>
#include <string.h>



I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
VL53L0X_RangingMeasurementData_t RangingData;
VL53L0X_Dev_t vl53l0x_c;
VL53L0X_DEV Dev = &vl53l0x_c;
/* USER CODE END PV */

char uart_tx_buffer[100];
uint8_t uart_rx_buffer[1];
volatile uint8_t flag_measure = 1;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN 0 */

// =============================================================
// PHẦN CODE SERVO (NHÚNG TRỰC TIẾP ĐỂ TRÁNH LỖI FILE)
// =============================================================

// 1. Hàm tính toán xung
uint32_t map_val(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// 2. Hàm khởi tạo Servo (TIM2, PA0) - Dùng thanh ghi
void Servo_Init(void) {
    // Cấp Clock cho GPIOA và TIM2
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    // Cấu hình chân PA0 là Alternate Function (AF1 - TIM2)
    GPIOA->MODER &= ~GPIO_MODER_MODE0;      // Xóa mode cũ
    GPIOA->MODER |= GPIO_MODER_MODE0_1;     // Set AF mode
    GPIOA->AFR[0] &= ~0xF;                  // Xóa 4 bit đầu AFRL
    GPIOA->AFR[0] |= 0x1;                   // Chọn AF1

    // Cấu hình Timer
    // Mặc định HSI 16MHz -> PSC = 15 -> 1MHz (1us/tick)
    TIM2->PSC = 15;
    TIM2->ARR = 19999;    // 20ms

    // Cấu hình PWM Mode 1
    TIM2->CCMR1 &= ~TIM_CCMR1_OC1M;
    TIM2->CCMR1 |= (6 << TIM_CCMR1_OC1M_Pos);
    TIM2->CCMR1 |= TIM_CCMR1_OC1PE;
    TIM2->CCER |= TIM_CCER_CC1E;
    TIM2->CR1 |= TIM_CR1_CEN;
}

// 3. Hàm điều khiển góc
void Servo_Write(int angle) {
    if(angle < 0) angle = 0;
    if(angle > 180) angle = 180;
    // 0 độ = 500us, 180 độ = 2500us
    TIM2->CCR1 = map_val(angle, 0, 180, 500, 2500);
}

// =============================================================
// KẾT THÚC CODE SERVO
// =============================================================

VL53L0X_Error LidarInit(void) {
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    uint32_t refSpadCount;
    uint8_t isApertureSpads;
    uint8_t VhvSettings;
    uint8_t PhaseCal;

    HAL_GPIO_WritePin(Lidar_xshutdown_GPIO_Port, Lidar_xshutdown_Pin, GPIO_PIN_RESET);
    HAL_Delay(20);
    HAL_GPIO_WritePin(Lidar_xshutdown_GPIO_Port, Lidar_xshutdown_Pin, GPIO_PIN_SET);
    HAL_Delay(20);

    status = VL53L0X_DataInit(Dev);
    if (status != VL53L0X_ERROR_NONE) return status;

    status = VL53L0X_StaticInit(Dev);
    if (status != VL53L0X_ERROR_NONE) return status;

    status = VL53L0X_PerformRefSpadManagement(Dev, &refSpadCount, &isApertureSpads);
    if (status != VL53L0X_ERROR_NONE) return status;

    status = VL53L0X_PerformRefCalibration(Dev, &VhvSettings, &PhaseCal);
    if (status != VL53L0X_ERROR_NONE) return status;

    status = VL53L0X_SetDeviceMode(Dev, VL53L0X_DEVICEMODE_SINGLE_RANGING);
    if (status != VL53L0X_ERROR_NONE) return status;

    VL53L0X_SetLimitCheckEnable(Dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
    VL53L0X_SetLimitCheckValue(Dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t)(0.25 * 65536));
    VL53L0X_SetLimitCheckEnable(Dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 0);
    VL53L0X_SetLimitCheckValue(Dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t)(18 * 65536));

    status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(Dev, 200000);

    return status;
}

int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart2, (uint8_t*) ptr, len, HAL_MAX_DELAY);
    return len;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        if (uart_rx_buffer[0] == 's') {
            flag_measure = 1;
        } else if (uart_rx_buffer[0] == 'p') {
            flag_measure = 0;
        }
        HAL_UART_Receive_IT(&huart2, uart_rx_buffer, 1);
    }
}
/* USER CODE END 0 */

int main(void) {
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_USART2_UART_Init();

    /* USER CODE BEGIN 2 */
    Dev->I2cHandle = &hi2c1;
    Dev->I2cDevAddr = 0x52;

    // --- KHỞI TẠO SERVO ---
    Servo_Init();
    Servo_Write(0);
    HAL_Delay(500);

    char *msg = "System Init...\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), 100);

    VL53L0X_Error Status = LidarInit();

    if (Status == VL53L0X_ERROR_NONE) {
        msg = "Radar Ready! Waiting for command...\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), 100);
        HAL_UART_Receive_IT(&huart2, uart_rx_buffer, 1);
    } else {
        sprintf(uart_tx_buffer, "Lidar Failed! Err: %d\r\n", Status);
        HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buffer, strlen(uart_tx_buffer), 100);
        while (1);
    }
    /* USER CODE END 2 */

    /* Infinite loop */
    while (1) {
        /* USER CODE BEGIN 3 */

        if (flag_measure == 1) {

            // ====================================================
            // CHIỀU ĐI: 0 -> 180 ĐỘ
            // ====================================================
            for (int i = 0; i <= 180; i += 2) {
                if (flag_measure == 0) break;

                // 1. RA LỆNH SERVO
                Servo_Write(i);

                // 2. CHỜ ĐỒNG BỘ (35ms)
                HAL_Delay(35);

                // 3. ĐO KHOẢNG CÁCH
                VL53L0X_PerformSingleRangingMeasurement(Dev, &RangingData);

                // 4. GỬI DỮ LIỆU NGAY LẬP TỨC
                if (RangingData.RangeStatus == 0 && RangingData.RangeMilliMeter < 8000) {
                    int len = sprintf(uart_tx_buffer, "%d,%d\r\n", i, RangingData.RangeMilliMeter);
                    HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buffer, len, 100);
                }
            }

            // ====================================================
            // CHIỀU VỀ: 180 -> 0 ĐỘ
            // ====================================================
            for (int i = 180; i >= 0; i -= 2) {
                if (flag_measure == 0) break;

                Servo_Write(i);
                HAL_Delay(35);

                VL53L0X_PerformSingleRangingMeasurement(Dev, &RangingData);

                if (RangingData.RangeStatus == 0 && RangingData.RangeMilliMeter < 8000) {
                    int len = sprintf(uart_tx_buffer, "%d,%d\r\n", i, RangingData.RangeMilliMeter);
                    HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buffer, len, 100);
                }
            }

        } else {
            HAL_Delay(100);
        }
        /* USER CODE END 3 */
    }
}

// ... CÁC HÀM CẤU HÌNH HỆ THỐNG GIỮ NGUYÊN BÊN DƯỚI ...
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
    RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) { Error_Handler(); }
}
static void MX_I2C1_Init(void) {
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 400000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) { Error_Handler(); }
}
static void MX_USART2_UART_Init(void) {
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK) { Error_Handler(); }
}
static void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    HAL_GPIO_WritePin(Lidar_xshutdown_GPIO_Port, Lidar_xshutdown_Pin, GPIO_PIN_SET);
    GPIO_InitStruct.Pin = Lidar_xshutdown_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(Lidar_xshutdown_GPIO_Port, &GPIO_InitStruct);
}
void Error_Handler(void) { __disable_irq(); while (1) {} }
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {}
#endif
