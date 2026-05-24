import cv2
import numpy as np
import threading
import time
import serial

# 固定ROI区域
x1, y1 = 238, 196
x2, y2 = 385, 295
xmin, xmax = min(x1, x2), max(x1, x2)
ymin, ymax = min(y1, y2), max(y1, y2)

# 全局变量
latest_point = None
lock = threading.Lock()

# 串口初始化（根据实际修改端口名）
ser = serial.Serial('/dev/ttyTHS1', 9600, timeout=1)
time.sleep(2)  # 等待串口稳定

def get_brightest_point(gray, thresh=230, min_area=2):
    _, mask = cv2.threshold(gray, thresh, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    largest = max(contours, key=cv2.contourArea)
    if cv2.contourArea(largest) < min_area:
        return None
    M = cv2.moments(largest)
    if M['m00'] == 0:
        return None
    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00'])
    return (cx, cy)

def send_data():
    while True:
        time.sleep(0.2)
        with lock:
            if latest_point:
                x_roi, y_roi = latest_point
                msg = f"a{x_roi},{y_roi}z\n"
                ser.write(msg.encode('utf-8'))
                print("发送：", msg.strip())

# 开启发送线程
send_thread = threading.Thread(target=send_data, daemon=True)
send_thread.start()

# 摄像头处理
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,480)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    roi = frame[ymin:ymax, xmin:xmax]
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    bright_point = get_brightest_point(gray, thresh=230, min_area=2)

    with lock:
        latest_point = bright_point

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
