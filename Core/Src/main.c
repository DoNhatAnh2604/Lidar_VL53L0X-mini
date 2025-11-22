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
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
VL53L0X_Error LidarInit(void) {
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    uint32_t refSpadCount;
    uint8_t isApertureSpads;
    uint8_t VhvSettings;
    uint8_t PhaseCal;

    // 1. Hardware Reset (Quan trọng để module ổn định sau khi nạp code)
    HAL_GPIO_WritePin(Lidar_xshutdown_GPIO_Port, Lidar_xshutdown_Pin, GPIO_PIN_RESET);
    HAL_Delay(20);
    HAL_GPIO_WritePin(Lidar_xshutdown_GPIO_Port, Lidar_xshutdown_Pin, GPIO_PIN_SET);
    HAL_Delay(20);

    // 2. Data Init (Kiểm tra kết nối và khởi tạo SW)
    // Có thể thêm VL53L0X_WaitDeviceBooted(Dev) nếu cần, nhưng DataInit thường đã đủ
    status = VL53L0X_DataInit(Dev);
    if (status != VL53L0X_ERROR_NONE) return status;

    // 3. Static Init (Nạp cấu hình mặc định)
    status = VL53L0X_StaticInit(Dev);
    if (status != VL53L0X_ERROR_NONE) return status;

    // 4. Calibration (BẮT BUỘC: Hiệu chuẩn tham chiếu SPAD và nhiệt độ)
    status = VL53L0X_PerformRefSpadManagement(Dev, &refSpadCount, &isApertureSpads);
    if (status != VL53L0X_ERROR_NONE) return status;

    status = VL53L0X_PerformRefCalibration(Dev, &VhvSettings, &PhaseCal);
    if (status != VL53L0X_ERROR_NONE) return status;

    // 5. Cấu hình Mode hoạt động
    status = VL53L0X_SetDeviceMode(Dev, VL53L0X_DEVICEMODE_SINGLE_RANGING);
    if (status != VL53L0X_ERROR_NONE) return status;

    // 6. Cấu hình bộ lọc và Timing Budget
    // Bật giới hạn tín hiệu (tránh đo nhiễu khi quá tối)
    VL53L0X_SetLimitCheckEnable(Dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
    VL53L0X_SetLimitCheckValue(Dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t)(0.25 * 65536));

    // Bật giới hạn Sigma (độ lệch chuẩn - lọc kết quả không tin cậy)
    VL53L0X_SetLimitCheckEnable(Dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 0);
    VL53L0X_SetLimitCheckValue(Dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t)(60 * 65536));

    // Set Timing Budget:
    // 33000 (33ms) = Standard (Nhanh, chính xác vừa phải)
    // 200000 (200ms) = High Accuracy (Chậm, chính xác cao - dùng cái này nếu muốn ổn định như test trước)
    status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(Dev, 66000);

    return status;
}

int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart2, (uint8_t*) ptr, len, HAL_MAX_DELAY);
    return len;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        // Kiểm tra ký tự nhận được
        if (uart_rx_buffer[0] == 's') {
            flag_measure = 1; // 's' = Start
        } else if (uart_rx_buffer[0] == 'p') {
            flag_measure = 0; // 'p' = Pause
        }

        // Tiếp tục bật ngắt để nhận ký tự tiếp theo
        HAL_UART_Receive_IT(&huart2, uart_rx_buffer, 1);
    }
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_USART2_UART_Init();
    /* USER CODE BEGIN 2 */
    Dev->I2cHandle = &hi2c1;
    Dev->I2cDevAddr = 0x52;

    char *msg = "System Init...\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), 100);

    // Gọi hàm cấu hình Lidar
    VL53L0X_Error Status = LidarInit();

    if (Status == VL53L0X_ERROR_NONE) {
        msg = "Lidar OK! Ready.\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), 100);

        // Chỉ bật ngắt nhận khi Lidar đã sẵn sàng
        HAL_UART_Receive_IT(&huart2, uart_rx_buffer, 1);
    } else {
        // Nếu lỗi, in mã lỗi và treo chương trình (hoặc xử lý khác)
        sprintf(uart_tx_buffer, "Lidar Init Failed! Error: %d\r\n", Status);
        HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buffer,
                strlen(uart_tx_buffer), 100);
        while (1)
            ; // Dừng tại đây
    }

    while (1) {

        if (flag_measure == 1) {
            // 1. Thực hiện đo
            VL53L0X_PerformSingleRangingMeasurement(Dev, &RangingData);

            // 2. Kiểm tra dữ liệu hợp lệ (RangeStatus = 0 là OK)
            if (RangingData.RangeStatus == 0) {
                if (RangingData.RangeMilliMeter < 8000) {
                    int len = sprintf(uart_tx_buffer, "Dist: %d mm\r\n",
                            RangingData.RangeMilliMeter);
                    HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buffer, len,
                            100);
                }
            } else {
                // Báo lỗi nếu đo sai (ví dụ quá xa hoặc bị che)
                int len = sprintf(uart_tx_buffer, "Error Status: %d\r\n",
                        RangingData.RangeStatus);
                HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buffer, len, 100);
            }
        }

        // Delay giữa các lần đo
        HAL_Delay(100);
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
    RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

    /** Configure the main internal regulator output voltage
     */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
            | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
        Error_Handler();
    }
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
    hi2c1.Init.ClockSpeed = 400000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN I2C1_Init 2 */

    /* USER CODE END I2C1_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

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
    if (HAL_UART_Init(&huart2) != HAL_OK) {
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
static void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };
    /* USER CODE BEGIN MX_GPIO_Init_1 */

    /* USER CODE END MX_GPIO_Init_1 */

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(Lidar_xshutdown_GPIO_Port, Lidar_xshutdown_Pin,
            GPIO_PIN_SET);

    /*Configure GPIO pin : Lidar_xshutdown_Pin */
    GPIO_InitStruct.Pin = Lidar_xshutdown_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(Lidar_xshutdown_GPIO_Port, &GPIO_InitStruct);

    /* USER CODE BEGIN MX_GPIO_Init_2 */

    /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
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
