#向外延伸，检测黑色占比
import cv2
import numpy as np

cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))  # MJPG usually supports higher fps
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 960)   # 先设置为支持60fps的分辨率
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 540)
cap.set(cv2.CAP_PROP_FPS, 60)
kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))

def sort_pts(pts):
    # 返回顺序为：左上、右上、右下、左下
    pts = np.array(pts)
    s = pts.sum(axis=1)
    diff = np.diff(pts, axis=1)
    top_left = pts[np.argmin(s)]
    bottom_right = pts[np.argmax(s)]
    top_right = pts[np.argmin(diff)]
    bottom_left = pts[np.argmax(diff)]
    return np.array([top_left, top_right, bottom_right, bottom_left], dtype=np.float32)

def outward_quad(quad, offset=7):
    # quad: 4x2 (已顺序)点，逆时针
    out_quad = []
    for i in range(4):
        p1 = quad[i]
        p0 = quad[(i-1)%4]
        p2 = quad[(i+1)%4]
        v1 = p1 - p0
        v2 = p2 - p1
        # 法向量（对边外法线）
        n1 = np.array([v1[1], -v1[0]])
        n1 = n1 / (np.linalg.norm(n1)+1e-6)
        n2 = np.array([v2[1], -v2[0]])
        n2 = n2 / (np.linalg.norm(n2)+1e-6)
        n = n1 + n2
        n = n / (np.linalg.norm(n)+1e-6)
        out_pt = p1 + n * offset
        out_quad.append(out_pt)
    return np.array(out_quad, dtype=np.float32)

while True:
    ret, img_raw = cap.read()
    if not ret:
        break

    img = cv2.cvtColor(img_raw, cv2.COLOR_BGR2GRAY)
    img = cv2.bilateralFilter(img, 9, 10, 10)
    img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel)
    _, binary = cv2.threshold(img, 90, 255, cv2.THRESH_BINARY_INV)
    contours, hierarchy = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    approx_inner = None

    if hierarchy is not None and len(contours) > 0:
        max_area = 0
        for i, contour in enumerate(contours):
            if hierarchy[0][i][3] != -1:
                epsilon = 0.02 * cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, epsilon, True)
                if len(approx) == 4:
                    area = cv2.contourArea(approx)
                    if area > max_area:
                        max_area = area
                        approx_inner = approx.reshape(-1, 2)
        if approx_inner is not None:
            pts = sort_pts(approx_inner)
            # 画原矩形
            cv2.drawContours(img_raw, [pts.astype(int)], 0, (0, 200, 255), 2)
            for x, y in pts:
                cv2.circle(img_raw, (int(x), int(y)), 4, (0, 0, 255), 2)
            # 外扩7像素后的新四边形
            out_quad = outward_quad(pts, offset=15)
            cv2.drawContours(img_raw, [out_quad.astype(int)], 0, (255, 0, 0), 2)
            # 计算外扩区域黑色占比
            mask = np.zeros(img.shape, dtype=np.uint8)
            cv2.drawContours(mask, [out_quad.astype(int)], 0, 255, -1)
            cv2.drawContours(mask, [pts.astype(int)], 0, 0, -1)
            region = cv2.bitwise_and(img, img, mask=mask)
            region_pixels = region[mask==255]
            if len(region_pixels) > 0:
                black_ratio = np.sum(region_pixels < 40) / len(region_pixels)
            else:
                black_ratio = 0.0
            cv2.putText(img_raw, f"Black ratio: {black_ratio:.2f}", (30,60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)

    cv2.imshow('Rect Outward Black Ratio', img_raw)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
