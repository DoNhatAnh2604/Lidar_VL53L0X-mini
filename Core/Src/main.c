/* USER CODE BEGIN Header */
/**
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "vl53l0x_api.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* --- CẤU HÌNH TIMING --- */
#define SERVO_SETTLE_MS     35
#define POWER_RECOVERY_MS   10
#define SERVO_EDGE_PAUSE    150
#define LIDAR_INVALID_MM    8190

/* --- CẤU HÌNH FILTER TẦNG 2 (ANTI-PULL-IN) --- */
#define NEAR_PULL_MM        120
#define CONFIRM_NEAR        3

/* --- CẤU HÌNH FILTER TẦNG 3 (SPATIAL) --- */
#define GAP_FILL_LIMIT_MM   80

/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
VL53L0X_RangingMeasurementData_t RangingData;
VL53L0X_Dev_t vl53l0x_c;
VL53L0X_DEV Dev = &vl53l0x_c;

char uart_tx_buffer[100];
uint8_t uart_rx_buffer[1];
volatile uint8_t flag_measure = 1;

// --- BIẾN CHO FILTER TẦNG 2 (Anti-Pull-In) ---
int last_valid = -1;
int near_candidate = -1;
uint8_t near_count = 0;

// --- BIẾN CHO FILTER TẦNG 3 (Spatial - Median) ---
int d_prev = -1;
int d_curr = -1;
int d_next = -1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN 0 */

// =============================================================
// 1. MODULE SERVO
// =============================================================
uint32_t map_val(long x, long in_min, long in_max, long out_min, long out_max) {
   return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void Servo_Init(void) {
   RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
   RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
   GPIOA->MODER &= ~GPIO_MODER_MODE0; GPIOA->MODER |= GPIO_MODER_MODE0_1;
   GPIOA->AFR[0] &= ~0xF; GPIOA->AFR[0] |= 0x1;
   TIM2->PSC = 15; TIM2->ARR = 19999;
   TIM2->CCMR1 &= ~TIM_CCMR1_OC1M; TIM2->CCMR1 |= (6 << TIM_CCMR1_OC1M_Pos);
   TIM2->CCMR1 |= TIM_CCMR1_OC1PE; TIM2->CCER |= TIM_CCER_CC1E;
   TIM2->CR1 |= TIM_CR1_CEN;
}

void Servo_Write(int angle) {
   if(angle < 0) angle = 0; if(angle > 180) angle = 180;
   TIM2->CCR1 = map_val(angle, 0, 180, 500, 2500);
}

// =============================================================
// 2. MODULE LIDAR
// =============================================================
VL53L0X_Error LidarInit(void) {
   VL53L0X_Error status = VL53L0X_ERROR_NONE;
   uint32_t refSpadCount; uint8_t isApertureSpads; uint8_t VhvSettings; uint8_t PhaseCal;

   HAL_GPIO_WritePin(Lidar_xshutdown_GPIO_Port, Lidar_xshutdown_Pin, GPIO_PIN_RESET);
   HAL_Delay(30);
   HAL_GPIO_WritePin(Lidar_xshutdown_GPIO_Port, Lidar_xshutdown_Pin, GPIO_PIN_SET);
   HAL_Delay(30);

   status = VL53L0X_DataInit(Dev); if (status != VL53L0X_ERROR_NONE) return status;
   status = VL53L0X_StaticInit(Dev); if (status != VL53L0X_ERROR_NONE) return status;
   status = VL53L0X_PerformRefSpadManagement(Dev, &refSpadCount, &isApertureSpads); if (status != VL53L0X_ERROR_NONE) return status;
   status = VL53L0X_PerformRefCalibration(Dev, &VhvSettings, &PhaseCal); if (status != VL53L0X_ERROR_NONE) return status;
   status = VL53L0X_SetDeviceMode(Dev, VL53L0X_DEVICEMODE_SINGLE_RANGING); if (status != VL53L0X_ERROR_NONE) return status;

   // Tuning cho Radar
   VL53L0X_SetVcselPulsePeriod(Dev, VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
   VL53L0X_SetVcselPulsePeriod(Dev, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);
   VL53L0X_SetLimitCheckEnable(Dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
   VL53L0X_SetLimitCheckValue(Dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t)(0.1 * 65536));
   VL53L0X_SetLimitCheckEnable(Dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
   VL53L0X_SetLimitCheckValue(Dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t)(30 * 65536));
   status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(Dev, 50000);
   return status;
}

// =============================================================
// 3. TẦNG 1: RAW SAMPLING (Lấy mẫu trung bình)
// =============================================================
int Lidar_Read_Stable(void) {
    int valid_count = 0;
    long sum = 0;
    for (int i = 0; i < 5; i++) {
        VL53L0X_PerformSingleRangingMeasurement(Dev, &RangingData);
        if (RangingData.RangeStatus == 0 && RangingData.RangeMilliMeter > 30 && RangingData.RangeMilliMeter < 2000) {
            sum += RangingData.RangeMilliMeter;
            valid_count++;
        }
        HAL_Delay(5);
    }
    if (valid_count >= 3) return (int)(sum / valid_count);
    VL53L0X_ClearInterruptMask(Dev, 0);
    return -1;
}

// =============================================================
// 4. TẦNG 2: ANTI-PULL-IN FILTER (Temporal - Giữ mặt xa)
// =============================================================
int AntiPullInFilter(int current) {
    if (current <= 0) { if (last_valid > 0) return last_valid; return 0; }
    if (last_valid < 0) { last_valid = current; return current; }

    if (current < last_valid - NEAR_PULL_MM) {
        if (near_candidate < 0 || abs(current - near_candidate) > (NEAR_PULL_MM / 2)) {
            near_candidate = current;
            near_count = 1;
            return last_valid; // Reject near, keep far
        }
        near_count++;
        if (near_count >= CONFIRM_NEAR) {
            last_valid = near_candidate; near_candidate = -1; near_count = 0;
            return last_valid;
        }
        return last_valid;
    }
    near_candidate = -1; near_count = 0; last_valid = current;
    return current;
}

// =============================================================
// 5. TẦNG 3: SPATIAL FILTER (Median + Gap Fill)
// =============================================================

// Hàm tìm trung vị của 3 số
int median3(int a, int b, int c) {
    if ((a > b) ^ (a > c)) return a;
    if ((b > a) ^ (b > c)) return b;
    return c;
}

// Hàm Reset toàn bộ trạng thái khi đổi chiều quét
void Reset_All_Filters(void) {
    // Reset Anti-Pull-In
    last_valid = -1; near_candidate = -1; near_count = 0;
    // Reset Spatial Buffer
    d_prev = -1; d_curr = -1; d_next = -1;
}

int _write(int file, char *ptr, int len) {
   HAL_UART_Transmit(&huart2, (uint8_t*) ptr, len, HAL_MAX_DELAY);
   return len;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
   if (huart->Instance == USART2) {
       if (uart_rx_buffer[0] == 's') flag_measure = 1;
       if (uart_rx_buffer[0] == 'p') flag_measure = 0;
       HAL_UART_Receive_IT(&huart2, uart_rx_buffer, 1);
   }
}
/* USER CODE END 0 */

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  Dev->I2cHandle = &hi2c1; Dev->I2cDevAddr = 0x52;
  Servo_Init(); Servo_Write(0); HAL_Delay(1000);

  sprintf(uart_tx_buffer, "Radar V3: Full Spatial Filter (Median + GapFill)...\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);

  if (LidarInit() == VL53L0X_ERROR_NONE) {
      HAL_UART_Receive_IT(&huart2, uart_rx_buffer, 1);
  } else {
      while (1) HAL_Delay(100);
  }

  /* USER CODE END 2 */

  /* USER CODE BEGIN WHILE */
  while (1)
  {
      if (flag_measure == 1) {

           // ============================================
           // ✅ CHIỀU ĐI (0 -> 180)
           // ============================================
           Reset_All_Filters();

           for (int i = 0; i <= 180; i++) {
               if (!flag_measure) break;

               Servo_Write(i);
               HAL_Delay(SERVO_SETTLE_MS); HAL_Delay(POWER_RECOVERY_MS);

               // --- PIPELINE XỬ LÝ 3 TẦNG ---

               // 1. Raw Read + Averaging
               int raw = Lidar_Read_Stable();

               // 2. Temporal Filter (Anti-Pull-In)
               int anti_pull_val = AntiPullInFilter(raw);

               // 3. Spatial Filter (Windowing)
               d_prev = d_curr;
               d_curr = d_next;
               d_next = anti_pull_val;

               int d_final = d_curr; // Mặc định lấy giá trị giữa

               // Chỉ lọc khi bộ đệm đã đầy (có đủ 3 mẫu: prev, curr, next)
               if (d_prev > 0 && d_curr >= 0 && d_next > 0) {

                   // A. Gap Fill: Nếu giữa bị mất (0) nhưng 2 bên có giá trị gần nhau
                   if (d_curr <= 0 && abs(d_prev - d_next) < GAP_FILL_LIMIT_MM) {
                       d_final = (d_prev + d_next) / 2; // Vá lỗ hổng bằng trung bình cộng
                   }
                   // B. Median Filter: Nếu cả 3 đều có giá trị
                   else if (d_curr > 0) {
                       d_final = median3(d_prev, d_curr, d_next);
                   }
               }

               // In kết quả (Lưu ý: d_final thực chất là kết quả của góc i-1, nhưng để vẽ realtime ta gửi luôn)
               int len = sprintf(uart_tx_buffer, "%d,%d\r\n", i, d_final);
               HAL_UART_Transmit(&huart2, (uint8_t*)uart_tx_buffer, len, 100);
           }
           HAL_Delay(SERVO_EDGE_PAUSE);

           // ============================================
           // ✅ CHIỀU VỀ (180 -> 0)
           // ============================================
           Reset_All_Filters();

           for (int i = 180; i >= 0; i--) {
               if (!flag_measure) break;

               Servo_Write(i);
               HAL_Delay(SERVO_SETTLE_MS); HAL_Delay(POWER_RECOVERY_MS);

               int raw = Lidar_Read_Stable();
               int anti_pull_val = AntiPullInFilter(raw);

               d_prev = d_curr;
               d_curr = d_next;
               d_next = anti_pull_val;

               int d_final = d_curr;
               if (d_prev > 0 && d_curr >= 0 && d_next > 0) {
                   if (d_curr <= 0 && abs(d_prev - d_next) < GAP_FILL_LIMIT_MM) {
                       d_final = (d_prev + d_next) / 2;
                   } else if (d_curr > 0) {
                       d_final = median3(d_prev, d_curr, d_next);
                   }
               }

               int len = sprintf(uart_tx_buffer, "%d,%d\r\n", i, d_final);
               HAL_UART_Transmit(&huart2, (uint8_t*)uart_tx_buffer, len, 100);
           }
           HAL_Delay(SERVO_EDGE_PAUSE);

       } else {
           HAL_Delay(200);
       }
  }
  /* USER CODE END 3 */
}

/* Các hàm SystemConfig, MX_Init giữ nguyên như cũ */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0}; RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  __HAL_RCC_PWR_CLK_ENABLE(); __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI; RCC_OscInitStruct.HSIState = RCC_HSI_ON; RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT; RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2; RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI; RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1; RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1; RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) Error_Handler();
}
static void MX_I2C1_Init(void) { hi2c1.Instance = I2C1; hi2c1.Init.ClockSpeed = 400000; hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2; hi2c1.Init.OwnAddress1 = 0; hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT; hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE; hi2c1.Init.OwnAddress2 = 0; hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE; hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE; HAL_I2C_Init(&hi2c1); }
static void MX_USART2_UART_Init(void) { huart2.Instance = USART2; huart2.Init.BaudRate = 115200; huart2.Init.WordLength = UART_WORDLENGTH_8B; huart2.Init.StopBits = UART_STOPBITS_1; huart2.Init.Parity = UART_PARITY_NONE; huart2.Init.Mode = UART_MODE_TX_RX; huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE; huart2.Init.OverSampling = UART_OVERSAMPLING_16; HAL_UART_Init(&huart2); }
static void MX_GPIO_Init(void) { GPIO_InitTypeDef GPIO_InitStruct = {0}; __HAL_RCC_GPIOH_CLK_ENABLE(); __HAL_RCC_GPIOA_CLK_ENABLE(); __HAL_RCC_GPIOB_CLK_ENABLE(); HAL_GPIO_WritePin(Lidar_xshutdown_GPIO_Port, Lidar_xshutdown_Pin, GPIO_PIN_SET); GPIO_InitStruct.Pin = Lidar_xshutdown_Pin; GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; GPIO_InitStruct.Pull = GPIO_NOPULL; GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; HAL_GPIO_Init(Lidar_xshutdown_GPIO_Port, &GPIO_InitStruct); }
void Error_Handler(void) { __disable_irq(); while (1) {} }
