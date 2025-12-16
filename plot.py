import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
import sys
import time 
import math

# ==============================================================================
# CẤU HÌNH HỆ THỐNG
# ==============================================================================
BAUD_RATE = 115200     
MAX_DISTANCE = 800    # Tầm quan sát 2m
SCAN_DURATION = 60     # Thời gian quét (giây)
OUTPUT_FILE = 'radar_dots_analysis.png'

# Cấu hình phân tích vật thể
FLATTEN_THRESHOLD = 5       # Gom nhóm nếu chênh lệch <= 5mm (theo yêu cầu)
MIN_OBJECT_SIZE = 20        # Chỉ báo cáo vật thể rộng hơn 20mm
# ==============================================================================

print(f"\n--- RADAR MAPPING V9.1 (DOTS + ANALYSIS < {FLATTEN_THRESHOLD}mm) ---")

# 1. TỰ ĐỘNG DÒ TÌM CỔNG COM
ports = serial.tools.list_ports.comports()
available_ports = [p.device for p in ports]

if len(available_ports) == 0:
    print("\n[LOI] Khong tim thay cong COM nao!")
    print("Hay cam day USB vao may tinh.")
    input("Nhan Enter de thoat...")
    sys.exit()

SELECTED_PORT = available_ports[0]
print(f"-> Tu dong chon cong: {SELECTED_PORT}")

# 2. KẾT NỐI SERIAL
ser = None
try:
    ser = serial.Serial(SELECTED_PORT, BAUD_RATE, timeout=0.1)
    ser.reset_input_buffer() 
    print(f"-> KET NOI THANH CONG!")
except Exception as e:
    print(f"\n[LOI KET NOI] Khong the mo cong {SELECTED_PORT}!")
    input("Nhan Enter de thoat...")
    sys.exit()

# 3. THIẾT LẬP GIAO DIỆN
fig = plt.figure(figsize=(13, 7), facecolor='black')
# Đẩy biểu đồ sang phải để nhường chỗ cho text
ax = fig.add_axes([0.35, 0.1, 0.6, 0.8], projection='polar', facecolor='black')

ax.set_thetamin(0)
ax.set_thetamax(180)            
ax.set_theta_zero_location("W") 
ax.set_rlim(0, MAX_DISTANCE)    
ax.set_title(f"RADAR DOTS ANALYSIS ({SELECTED_PORT})", color='white', va='bottom', fontsize=14)

ax.tick_params(axis='x', colors='gray', labelsize=10)
ax.tick_params(axis='y', colors='green', labelsize=8)
ax.grid(color='green', alpha=0.3)

# Text hiển thị thông tin phân tích (Bên trái)
analysis_text = fig.text(0.02, 0.90, "DANG KHOI TAO...", color='#00ff00', fontsize=10, family='monospace', va='top')

# Dữ liệu gốc
angles = np.linspace(0, np.pi, 181) 
distances = np.full(181, np.nan)

# --- CÁC ĐỐI TƯỢNG VẼ ---
# Chỉ vẽ các chấm tròn (Dots)
dots, = ax.plot(angles, distances, color='#00ff00', marker='o', linestyle='None', markersize=3, alpha=1.0)

# Kim quét
scan_line, = ax.plot([0, 0], [0, MAX_DISTANCE], color='red', linewidth=1, alpha=0.8)

current_angle_rad = 0
start_time = time.time()
is_finished = False

# --- HÀM TÍNH TOÁN VẬT THỂ ---
def analyze_object(group):
    """
    Tính toán vị trí, khoảng cách và kích thước của nhóm điểm
    """
    if not group or len(group) < 2: return None
    
    # 1. Tính độ rộng vật thể (Euclidean Distance giữa điểm đầu và cuối)
    theta1, r1 = group[0]
    theta2, r2 = group[-1]
    
    x1 = r1 * np.cos(theta1); y1 = r1 * np.sin(theta1)
    x2 = r2 * np.cos(theta2); y2 = r2 * np.sin(theta2)
    
    width_mm = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    
    # 2. Tính khoảng cách trung bình tới vật
    sum_dist = sum(point[1] for point in group)
    avg_dist_mm = sum_dist / len(group)
    
    # 3. Xác định vị trí (Góc trung bình)
    avg_theta = (theta1 + theta2) / 2
    avg_degree = np.degrees(avg_theta)
    
    position = "TRAI" if avg_degree <= 90 else "PHAI"
    
    # Chỉ báo cáo nếu vật thể đủ lớn (lọc nhiễu)
    if width_mm >= MIN_OBJECT_SIZE:
        return f"- {position}: Cach {avg_dist_mm:.0f}mm | Rong {width_mm:.0f}mm"
    return None

# 4. HÀM CẬP NHẬT
def update(frame):
    global current_angle_rad, is_finished
    
    # --- XỬ LÝ THỜI GIAN ---
    elapsed = time.time() - start_time
    remaining = SCAN_DURATION - elapsed
    
    if remaining <= 0 and not is_finished:
        is_finished = True
        ax.set_title(f"KET QUA: {OUTPUT_FILE}", color='yellow', va='bottom', fontsize=14)
        plt.savefig(OUTPUT_FILE, facecolor='black')
        print(f"-> DA LUU: {OUTPUT_FILE}")
        ani.event_source.stop()
        if ser.is_open: ser.close()
        return dots, scan_line, analysis_text

    if not is_finished:
        ax.set_title(f"DANG QUET... CON LAI: {remaining:.1f}s", color='white', va='bottom', fontsize=12)

    # --- ĐỌC DỮ LIỆU ---
    reads = 0
    while ser.in_waiting > 0 and reads < 50:
        reads += 1
        try:
            line_data = ser.readline().decode('utf-8', errors='ignore').strip()
            if ',' in line_data:
                parts = line_data.split(',')
                if len(parts) >= 2:
                    angle_deg = int(parts[0])
                    dist_mm = int(parts[1])

                    if 0 <= angle_deg <= 180:
                        current_angle_rad = np.radians(angle_deg)
                        
                        # Logic lưu Map: Chỉ cập nhật khi có vật cản hợp lệ
                        if 10 < dist_mm <= MAX_DISTANCE:
                            distances[angle_deg] = dist_mm
                        else:
                            # Nếu quá xa hoặc lỗi -> Gán NaN để ẩn chấm đi
                            distances[angle_deg] = np.nan
        except:
            pass

    # --- PHÂN TÍCH DỮ LIỆU (ĐỂ HIỂN THỊ TEXT) ---
    valid_points = []
    for i in range(181):
        if not np.isnan(distances[i]):
            valid_points.append( (angles[i], distances[i]) )
    
    detected_objects_str = ["--- KET QUA PHAN TICH ---"]

    if len(valid_points) > 0:
        current_group = [valid_points[0]]
        
        for i in range(1, len(valid_points)):
            curr_angle, curr_dist = valid_points[i]
            prev_angle, prev_dist = valid_points[i-1]
            
            diff = abs(curr_dist - prev_dist)
            
            # Logic gom nhóm (dựa trên Threshold 5mm)
            if diff <= FLATTEN_THRESHOLD:
                current_group.append((curr_angle, curr_dist))
            else:
                # Phân tích nhóm cũ trước khi tạo nhóm mới
                info = analyze_object(current_group)
                if info: detected_objects_str.append(info)
                
                # Bắt đầu nhóm mới
                current_group = [(curr_angle, curr_dist)]
        
        # Xử lý nhóm cuối cùng
        if current_group:
            info = analyze_object(current_group)
            if info: detected_objects_str.append(info)

    # Cập nhật hiển thị
    dots.set_ydata(distances)
    scan_line.set_data([current_angle_rad, current_angle_rad], [0, MAX_DISTANCE])
    
    # Hiển thị thông tin phân tích
    if len(detected_objects_str) > 1:
        analysis_text.set_text("\n".join(detected_objects_str))
    else:
        analysis_text.set_text("DANG TIM VAT CAN...")

    return dots, scan_line, analysis_text

# 5. CHẠY
print(f"-> Bat dau dem nguoc {SCAN_DURATION} giay...")
ani = FuncAnimation(fig, update, interval=20, blit=False, cache_frame_data=False)

try:
    plt.show()
except KeyboardInterrupt:
    pass

if ser.is_open:
    ser.close()