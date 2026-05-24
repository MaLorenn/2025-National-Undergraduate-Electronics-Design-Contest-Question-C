#转换思路：识别白色矩形
import cv2
import numpy as np
import serial
import time

SERIAL_PORT = '/dev/ttyTHS1'
SERIAL_BAUD = 115200

ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1)

cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FPS, 60)

kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))

def sort_pts(pts):
    pts = np.array(pts)
    s = pts.sum(axis=1)
    diff = np.diff(pts, axis=1)
    top_left = pts[np.argmin(s)]
    bottom_right = pts[np.argmax(s)]
    top_right = pts[np.argmin(diff)]
    bottom_left = pts[np.argmax(diff)]
    return np.array([top_left, top_right, bottom_right, bottom_left], dtype=np.float32)

def line_intersection(p1, p2, q1, q2):
    p = np.array(p1, dtype=np.float64)
    r = np.array(p2, dtype=np.float64) - p
    q = np.array(q1, dtype=np.float64)
    s = np.array(q2, dtype=np.float64) - q
    r_cross_s = np.cross(r, s)
    q_minus_p = q - p
    if abs(r_cross_s) < 1e-8:
        return None
    t = np.cross(q_minus_p, s) / r_cross_s
    u = np.cross(q_minus_p, r) / r_cross_s
    if 0 <= t <= 1 and 0 <= u <= 1:
        return p + t * r
    else:
        return None

def outward_quad(quad, offset=15):
    out_quad = []
    for i in range(4):
        p1 = quad[i]
        p0 = quad[(i-1)%4]
        p2 = quad[(i+1)%4]
        v1 = p1 - p0
        v2 = p2 - p1
        n1 = np.array([v1[1], -v1[0]])
        n1 = n1 / (np.linalg.norm(n1) + 1e-6)
        n2 = np.array([v2[1], -v2[0]])
        n2 = n2 / (np.linalg.norm(n2) + 1e-6)
        n = n1 + n2
        n = n / (np.linalg.norm(n) + 1e-6)
        out_pt = p1 + n * offset
        out_quad.append(out_pt)
    return np.array(out_quad, dtype=np.float32)

width, height = 261, 174  # mm

last_send_time = 0
frame_count = 0
fps = 0
fps_time = time.time()

while True:
    ret, img_raw = cap.read()
    if not ret:
        break

    # CUDA加速图像预处理
    gpu_frame = cv2.cuda_GpuMat()
    gpu_frame.upload(img_raw)
    gpu_gray = cv2.cuda.cvtColor(gpu_frame, cv2.COLOR_BGR2GRAY)
    gaussian_filter = cv2.cuda.createGaussianFilter(cv2.CV_8UC1, cv2.CV_8UC1, (3,3), 0)
    gpu_blur = gaussian_filter.apply(gpu_gray)
    blur_cpu = gpu_blur.download()

    blur_close = cv2.morphologyEx(blur_cpu, cv2.MORPH_CLOSE, kernel)

    # 关键修改1：普通二值化，白色为白（阈值可按实际调节，比如140或150）
    _, binary = cv2.threshold(blur_close, 140, 255, cv2.THRESH_BINARY)

    # 关键修改2：只检测外层最大四边形
    contours, hierarchy = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    approx_outer = None
    max_area = 0
    tx = ty = None
    target_ok = False
    img_h, img_w = img_raw.shape[:2]
    img_center = (img_w // 2, img_h // 2)

    for contour in contours:
        epsilon = 0.005 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)
        if len(approx) == 4:
            area = cv2.contourArea(approx)
            if area > max_area and area >= 2500 and area <= 70000:
                max_area = area
                approx_outer = approx.reshape(-1, 2)
    if approx_outer is not None:
        pts = sort_pts(approx_outer)
        # 可选：仍然可用outward_quad等操作，但无需再判黑色占比
        cv2.drawContours(img_raw, [pts.astype(int)], 0, (0, 200, 255), 2)
        for x, y in pts:
            cv2.circle(img_raw, (int(x), int(y)), 4, (0, 0, 255), 2)
        pts_real = np.float32([
            [0, 0],
            [width, 0],
            [width, height],
            [0, height]
        ])
        inv_H = cv2.getPerspectiveTransform(pts_real, pts)
        real_center = np.array([[[width/2, height/2]]], dtype='float32')
        center_img = cv2.perspectiveTransform(real_center, inv_H)
        cx, cy = center_img[0,0]
        if not (np.isnan(cx) or np.isnan(cy)):
            cx, cy = int(round(cx)), int(round(cy))
            cv2.circle(img_raw, (cx, cy), 4, (0, 0, 255), -1)
            cv2.putText(img_raw, "True Center", (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255), 2)
            tx, ty = cx, cy
            target_ok = True

    # 始终显示图片中心点
    cv2.circle(img_raw, img_center, 4, (0, 255, 0), -1)
    cv2.putText(img_raw, "Image Center", (img_center[0]+10, img_center[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)

    now = time.time()
    if now - last_send_time > 0.07:
        last_send_time = now
        tx_str = str(tx) if (target_ok and tx is not None) else "-1"
        ty_str = str(ty) if (target_ok and ty is not None) else "-1"
        cx_str = str(img_center[0])
        cy_str = str(img_center[1])
        msg = f"a{tx_str},{ty_str}b"
        try:
            ser.write(msg.encode())
        except Exception as e:
            print(f"Serial send error: {e}")

    # FPS统计和显示
    frame_count += 1
    now_fps = time.time()
    elapsed = now_fps - fps_time
    if elapsed >= 1.0:
        fps = frame_count / elapsed
        frame_count = 0
        fps_time = now_fps
        print(f"FPS: {fps:.1f}")
    cv2.putText(img_raw, f"FPS: {fps:.1f}", (30, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)

    cv2.imshow('Rect True Center Detection', img_raw)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

ser.close()
cap.release()
cv2.destroyAllWindows()
