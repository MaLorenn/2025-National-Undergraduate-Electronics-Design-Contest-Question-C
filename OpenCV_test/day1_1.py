#标出矩形中心


import cv2
import numpy as np

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'YUYV'))
kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))

def sort_pts(pts):
    # 左上、右上、右下、左下排序
    pts = sorted(pts, key=lambda p: (p[1], p[0]))
    top = sorted(pts[:2], key=lambda p: p[0])
    bottom = sorted(pts[2:], key=lambda p: p[0])
    return np.array(top + bottom, dtype=np.float32)

width, height = 261, 174  # mm，实际内框水平和竖直尺寸

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
            cv2.drawContours(img_raw, [pts.astype(int)], 0, (0, 200, 255), 2)
            for x, y in pts:
                cv2.circle(img_raw, (int(x), int(y)), 7, (0, 0, 255), 2)

            # 透视变换还原物理中心
            pts_real = np.float32([
                [0, 0],
                [width, 0],
                [width, height],
                [0, height]
            ])
            H = cv2.getPerspectiveTransform(pts, pts_real)
            real_center = np.array([[[width/2, height/2]]], dtype='float32')
            inv_H = cv2.getPerspectiveTransform(pts_real, pts)
            center_img = cv2.perspectiveTransform(real_center, inv_H)
            cx, cy = center_img[0,0]
            cx, cy = int(round(cx)), int(round(cy))
            cv2.circle(img_raw, (cx, cy), 12, (0, 0, 255), -1)
            cv2.putText(img_raw, "True Center", (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255), 2)

    cv2.imshow('Rect True Center Detection', img_raw)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
