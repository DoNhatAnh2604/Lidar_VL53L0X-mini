# ==========================
# IMPORT THƯ VIỆN
# ==========================
import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button
import sys
import time
import math

# ==============================================================================
# CẤU HÌNH HỆ THỐNG
# ==============================================================================

BAUD_RATE = 115200
SCAN_DURATION = 120              
OUTPUT_FILE = 'radar_map_result.png'

# --- CẤU HÌNH THUẬT TOÁN GOM VẬT THỂ ---
# Tăng lên 50mm để không bị đứt khi quét tường nghiêng
NEIGHBOR_THRESHOLD_MM = 50      
# Cho phép mất tín hiệu tối đa 5 độ vẫn nối lại được
ANGLE_GAP_ALLOWANCE = 5         
# Vật phải có ít nhất 3 điểm mới hiện
MIN_POINTS_IN_OBJECT = 3        

# Biến toàn cục
current_max_dist = 2000 
start_time = time.time()
finished = False

print("\n--- RADAR MAPPING V8 (FINAL FIX) ---")

# ==============================================================================
# 1. KẾT NỐI
# ==============================================================================
ports = serial.tools.list_ports.comports()
if not ports:
    print("[CANH BAO] Khong tim thay cong COM (Che do Demo)")
    SELECTED_PORT = "COM1" 
else:
    SELECTED_PORT = ports[0].device

print(f"-> Su dung cong: {SELECTED_PORT}")

ser = None
try:
    ser = serial.Serial(SELECTED_PORT, BAUD_RATE, timeout=0.1)
    ser.reset_input_buffer()
except:
    pass

# ==============================================================================
# 2. GIAO DIỆN & NÚT BẤM
# ==============================================================================

fig = plt.figure(figsize=(13, 8), facecolor='black') 
ax = fig.add_axes([0.35, 0.2, 0.6, 0.70], projection='polar', facecolor='black')
ax.set_thetamin(0); ax.set_thetamax(180)
ax.set_theta_zero_location("W")
ax.set_rlim(0, current_max_dist)
ax.tick_params(axis='x', colors='gray'); ax.tick_params(axis='y', colors='green')
ax.grid(color='green', alpha=0.3)

mode_text = fig.text(0.65, 0.95, f"CHE DO: XA ({current_max_dist}mm)", color='yellow', fontsize=14, weight='bold', ha='center')
analysis_text = fig.text(0.02, 0.95, "DANG KHOI TAO...", color='#00ff00', fontsize=10, family='monospace', va='top')

angles = np.linspace(0, np.pi, 181)
distances = np.full(181, np.nan)
dots, = ax.plot(angles, distances, linestyle='None', marker='o', markersize=4, color='#00ff00')
scan_line, = ax.plot([0, 0], [0, current_max_dist], color='red', linewidth=1)

current_angle_rad = 0

# Tạo nút bấm
ax_near, ax_med, ax_far, ax_reset = plt.axes([0.35, 0.05, 0.12, 0.07]), plt.axes([0.48, 0.05, 0.12, 0.07]), plt.axes([0.61, 0.05, 0.12, 0.07]), plt.axes([0.80, 0.05, 0.12, 0.07])
btn_near, btn_med, btn_far, btn_reset = Button(ax_near, 'Gần (50cm)'), Button(ax_med, 'TB (120cm)'), Button(ax_far, 'Xa (200cm)'), Button(ax_reset, 'QUÉT LẠI', color='#800000', hovercolor='#ff0000')

for btn in [btn_near, btn_med, btn_far, btn_reset]:
    btn.label.set_color('white')
    if btn != btn_reset: btn.color = '#303030'; btn.hovercolor = '#505050'

def set_dist(d, name):
    global current_max_dist
    current_max_dist = d
    ax.set_rlim(0, current_max_dist)
    mode_text.set_text(f"CHE DO: {name} ({current_max_dist}mm)")

def reset_map(event):
    global start_time, finished
    distances.fill(np.nan) 
    dots.set_ydata(distances)
    start_time = time.time()
    finished = False
    ax.set_title("DA RESET...", color='white')
    if ser and ser.is_open: ser.reset_input_buffer()

btn_near.on_clicked(lambda x: set_dist(500, "GAN"))
btn_med.on_clicked(lambda x: set_dist(1200, "TRUNG BINH"))
btn_far.on_clicked(lambda x: set_dist(2000, "XA"))
btn_reset.on_clicked(reset_map)

# ==============================================================================
# 3. LOGIC GOM VẬT THỂ & PHÂN TÍCH
# ==============================================================================

def analyze_object(group):
    if len(group) < MIN_POINTS_IN_OBJECT: return None
    
    (t1, r1) = group[0]   # Điểm đầu chuỗi
    (t2, r2) = group[-1]  # Điểm cuối chuỗi
    
    # Tính độ rộng không gian
    x1, y1 = r1 * np.cos(t1), r1 * np.sin(t1)
    x2, y2 = r2 * np.cos(t2), r2 * np.sin(t2)
    width = math.hypot(x2 - x1, y2 - y1)
    
    avg_dist = sum(p[1] for p in group) / len(group)
    deg_start = np.degrees(t1)
    deg_end = np.degrees(t2)
    
    side = "PHAI" if (deg_start + deg_end)/2 <= 90 else "TRAI"
    
    # --- ĐÃ XÓA PHẦN (Sau ...) ---
    return f"- {side} [{deg_start:.0f}-{deg_end:.0f}do]: Cach~{avg_dist:.0f} | Rong {width:.0f}mm"

def update(frame):
    global current_angle_rad, finished

    elapsed = time.time() - start_time
    remain = SCAN_DURATION - elapsed
    if remain <= 0:
        if not finished:
            finished = True 
            ax.set_title("HOAN TAT (BAM 'QUET LAI' DE RESET)", color='yellow')
            plt.savefig(OUTPUT_FILE, facecolor='black')
        if ser and ser.is_open: ser.reset_input_buffer()
        return dots, scan_line, analysis_text, mode_text

    ax.set_title(f"DANG QUET... {remain:.1f}s", color='white')

    # Đọc dữ liệu
    if ser and ser.is_open:
        cnt = 0
        while ser.in_waiting and cnt < 100:
            cnt += 1
            try:
                line = ser.readline().decode().strip()
                if ',' in line:
                    ang, dist = map(int, line.split(',')[:2])
                    if 0 <= ang <= 180:
                        current_angle_rad = np.radians(ang)
                        # Chỉ nhận dữ liệu hợp lệ
                        distances[ang] = dist if (20 < dist <= 2500) else np.nan
            except: pass

    # --- THUẬT TOÁN CHAIN LINK (NÂNG CẤP) ---
    valid = []
    for i in range(181):
        d = distances[i]
        if not np.isnan(d) and d <= current_max_dist: 
            valid.append((angles[i], d, i))

    results = ["--- KET QUA PHAN TICH ---"]
    
    if len(valid) > 0:
        group = [(valid[0][0], valid[0][1])] 
        
        for i in range(1, len(valid)):
            ct, cd, c_idx = valid[i]   # Current
            pt, pd, p_idx = valid[i-1] # Previous
            
            # 1. Kiểm tra góc: Nếu 2 điểm cách nhau <= 5 độ (ANGLE_GAP_ALLOWANCE)
            angle_diff = abs(c_idx - p_idx)
            is_angle_connected = angle_diff <= ANGLE_GAP_ALLOWANCE

            # 2. Kiểm tra khoảng cách: 
            # Dùng ngưỡng 50mm để chấp nhận độ dốc của tường/góc nhà
            dist_diff = abs(cd - pd)
            is_dist_connected = dist_diff <= NEIGHBOR_THRESHOLD_MM
            
            # Logic nối: Góc phải gần nhau VÀ Khoảng cách phải liền mạch
            if is_angle_connected and is_dist_connected:
                group.append((ct, cd))
            else:
                # Đứt chuỗi -> Chốt vật thể cũ
                info = analyze_object(group)
                if info: results.append(info)
                # Bắt đầu vật thể mới
                group = [(ct, cd)]
        
        # Chốt vật thể cuối cùng
        info = analyze_object(group)
        if info: results.append(info)

    # Cập nhật hiển thị
    dots.set_ydata(distances)
    scan_line.set_data([current_angle_rad, current_angle_rad], [0, current_max_dist]) 
    
    # Hiển thị tối đa 15 dòng
    analysis_text.set_text("\n".join(results[:15]))

    return dots, scan_line, analysis_text, mode_text

# ==============================================================================
# CHẠY CHƯƠNG TRÌNH
# ==============================================================================
ani = FuncAnimation(fig, update, interval=20, blit=False)
plt.show()
if ser and ser.is_open: ser.close()