#得到矩形面积
import cv2
import numpy as np

cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))  # MJPG usually supports higher fps
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 800)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 600)
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
                epsilon = 0.01 * cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, epsilon, True)
                if len(approx) == 4:
                    area = cv2.contourArea(approx)
                    if area > max_area:
                        max_area = area
                        approx_inner = approx.reshape(-1, 2)
        if approx_inner is not None:
            pts = sort_pts(approx_inner)
            cv2.drawContours(img_raw, [pts.astype(int)], 0, (0, 200, 255), 2)
            # 实时显示面积
            area = cv2.contourArea(pts)
            # 面积显示在左上角，也可自定义显示在pts[0]附近
            cv2.putText(img_raw, f"Area: {area:.1f}", (int(pts[0][0]), int(pts[0][1])-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,128,0), 2)

    cv2.imshow('Rect Area Detection', img_raw)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
