# 🔍 Hệ Thống LiDAR Mini Quét 2D  
**(STM32F401RE & VL53L0X)**

## 📌 Giới thiệu

Đây là mã nguồn cho đồ án **“Thiết kế và chế tạo hệ thống quét khoảng cách 2D (LiDAR Mini)”**.  
Hệ thống sử dụng cảm biến đo khoảng cách **Time-of-Flight (ToF) VL53L0X** gắn trên **động cơ Servo** để quét môi trường xung quanh.

Dữ liệu đo được sẽ gửi về máy tính thông qua **UART (USB)** hoặc **Bluetooth**, sau đó **phần mềm Python** sẽ dựng lại **bản đồ radar 2D theo thời gian thực**.

📚 Dự án phù hợp cho:
- Nghiên cứu cơ bản về **LiDAR**
- Làm quen **SLAM**
- Ứng dụng trong **robot tự hành**

---

## 🚀 Tính năng nổi bật

- ⏱️ **Quét thời gian thực**  
  Quét góc **180°** và cập nhật bản đồ liên tục

- 🎯 **Độ chính xác cao**  
  Điều khiển Servo bằng **lập trình thanh ghi (Register)** cho chuyển động mượt

- 🧠 **Hiển thị thông minh**  
  Phần mềm Python tích hợp:
  - Lọc nhiễu
  - Làm phẳng bề mặt
  - Tự động đo kích thước vật thể

- 📡 **Đa kết nối**  
  - USB (UART)

---

## 🛠️ Phần cứng yêu cầu

| Linh kiện | Số lượng | Ghi chú |
|---------|---------|--------|
| STM32 Nucleo-F401RE | 1 | Vi điều khiển trung tâm |
| VL53L0X | 1 | Cảm biến ToF đo khoảng cách |
| Servo MG90S | 1 | Quay quét 0–180° |
| Bluetooth HC-05 | 1 *(tùy chọn)* | Kết nối không dây |
| Dây jumper | - | Đực–Đực, Đực–Cái |
| Cáp USB Mini-B | 1 | Nạp code & cấp nguồn |

---

## 🔌 Sơ đồ kết nối (Pinout)

### 1️⃣ Servo MG90S

| Chân Servo | Chân STM32 | Chức năng |
|----------|-----------|----------|
| Cam (Signal) | PA0 | PWM (TIM2 CH1) |
| Đỏ (VCC) | 5V | Nguồn động lực |
| Nâu (GND) | GND | Mass chung |

---

### 2️⃣ Cảm biến VL53L0X (I2C)

| Chân Cảm biến | Chân STM32 | Chức năng |
|-------------|-----------|----------|
| SCL | PB8 | I2C1 Clock |
| SDA | PB9 | I2C1 Data |
| VIN / VCC | 3.3V | Nguồn logic |
| GND | GND | Mass chung |
| XSHUT | PB3 | Reset / Enable *(tùy chọn)* |

---

### 3️⃣ Bluetooth HC-05 *(Tùy chọn)*

| Chân HC-05 | Chân STM32 | Chức năng |
|-----------|-----------|----------|
| TX | PA10 | USART1 RX |
| RX | PA9 | USART1 TX |
| VCC | 5V | Nguồn nuôi |
| GND | GND | Mass chung |



