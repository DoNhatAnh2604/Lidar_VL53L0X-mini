# 🔍 Hệ Thống LiDAR Mini Quét 2D  
**(STM32 Nucleo F401RE & VL53L0X)**  

> 📷 *(Thay thế bằng ảnh thực tế của dự án)*

## 📌 Giới thiệu
Đây là mã nguồn cho đồ án **“Thiết kế và chế tạo hệ thống quét khoảng cách 2D (LiDAR mini)”**.  
Hệ thống sử dụng cảm biến **Time-of-Flight (ToF) VL53L0X** gắn trên **Servo MG90** để quét môi trường xung quanh trong mặt phẳng 2D.

Dữ liệu đo được được **truyền trực tiếp về máy tính qua UART2 (Virtual COM Port qua cáp USB)**.  
Phần mềm **Python** tiếp nhận dữ liệu và hiển thị **Radar 2D theo thời gian thực**.

Dự án phù hợp cho nghiên cứu cơ bản về **LiDAR**, tiền đề cho **SLAM** và **robot tự hành**.

---

## 🚀 Tính năng nổi bật
- ⏱️ Quét thời gian thực trong dải **0° – 180°**
- 🎯 Điều khiển Servo bằng **lập trình thanh ghi (Register-level PWM)** cho chuyển động mượt
- 🧠 Phần mềm Python lọc nhiễu, gom nhóm điểm đo và tự động tính **khoảng cách – độ rộng – vị trí vật thể**
- 🔌 Kết nối ổn định qua **UART2 → USB (không sử dụng Bluetooth)**

---

## 🛠️ Phần cứng sử dụng

| Linh kiện | Số lượng | Ghi chú |
|---------|---------|--------|
| STM32 Nucleo-F401RE | 1 | Vi điều khiển trung tâm |
| VL53L0X | 1 | Cảm biến ToF |
| Servo MG90 | 1 | Quay quét 0–180° |
| Dây Jumper | - | Kết nối |
| Cáp USB | 1 | Nạp code & truyền dữ liệu |

---

## 🔌 Sơ đồ kết nối (Pinout)

**Servo MG90**
- Signal → **PA0** (TIM2 CH1 – PWM)  
- VCC → **5V**  
- GND → **GND**

**VL53L0X (I2C)**
- SCL → **PB8**  
- SDA → **PB9**  
- XSHUT → **PB3**  
- VCC → **3.3V**  
- GND → **GND**

**UART2 (PC)**
- TX → **PA2**  
- RX → **PA3**

---

## 💻 Cấu trúc thư mục
