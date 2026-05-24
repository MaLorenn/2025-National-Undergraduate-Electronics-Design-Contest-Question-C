import cv2
import numpy as np

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'YUYV'))
kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))

def line_intersection(p1, p2, q1, q2):
    """两线段p1-p2, q1-q2的交点（存在则返回np数组，否则None）"""
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
            if np.isnan(cx) or np.isnan(cy):
                # 只显示图片中心点
                img_h, img_w = img_raw.shape[:2]
                img_center = (img_w // 2, img_h // 2)
                cv2.circle(img_raw, img_center, 4, (0, 255, 0), -1)
                cv2.putText(img_raw, "Image Center", (img_center[0]+10, img_center[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)
                cv2.imshow('Rect True Center Detection', img_raw)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                continue
            cx, cy = int(round(cx)), int(round(cy))
            cv2.circle(img_raw, (cx, cy), 4, (0, 0, 255), -1)
            cv2.putText(img_raw, "True Center", (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255), 2)

            v = pts[3] - pts[0]
            v = v / np.linalg.norm(v)
            edges = [(pts[i], pts[(i+1)%4]) for i in range(4)]
            hits = []
            for edge in edges:
                p1 = line_intersection((cx, cy), (cx + 1000*v[0], cy + 1000*v[1]), edge[0], edge[1])
                if p1 is not None:
                    hits.append(p1)
                p2 = line_intersection((cx, cy), (cx - 1000*v[0], cy - 1000*v[1]), edge[0], edge[1])
                if p2 is not None:
                    hits.append(p2)
            if len(hits) >= 2:
                hits = sorted(hits, key=lambda pt: np.linalg.norm(pt - np.array([cx, cy])))
                p1, p2 = hits[:2]
                vec1 = p1 - np.array([cx, cy])
                vec2 = p2 - np.array([cx, cy])
                if np.dot(vec1, -v) > np.dot(vec2, -v):
                    up_pt = p1
                    down_pt = p2
                else:
                    up_pt = p2
                    down_pt = p1
                l1 = np.linalg.norm(up_pt - down_pt)
                norm = np.linalg.norm(up_pt - np.array([cx, cy]))
                if norm == 0:
                    # 跳过本帧
                    img_h, img_w = img_raw.shape[:2]
                    img_center = (img_w // 2, img_h // 2)
                    cv2.circle(img_raw, img_center, 4, (0, 255, 0), -1)
                    cv2.putText(img_raw, "Image Center", (img_center[0]+10, img_center[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)
                    cv2.imshow('Rect True Center Detection', img_raw)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                    continue
                dir_vec = (up_pt - np.array([cx, cy])) / norm
                target_pt = np.array([cx, cy]) + dir_vec * (20/87 * l1)
                if np.isnan(target_pt[0]) or np.isnan(target_pt[1]):
                    # 跳过本帧
                    img_h, img_w = img_raw.shape[:2]
                    img_center = (img_w // 2, img_h // 2)
                    cv2.circle(img_raw, img_center, 4, (0, 255, 0), -1)
                    cv2.putText(img_raw, "Image Center", (img_center[0]+10, img_center[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)
                    cv2.imshow('Rect True Center Detection', img_raw)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                    continue
                tx, ty = int(round(target_pt[0])), int(round(target_pt[1]))
                cv2.circle(img_raw, (tx, ty), 4, (255, 0, 0), -1)
                cv2.putText(img_raw, "Target", (tx, ty-10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,0,0), 2)

    # 始终显示图片中心点
    img_h, img_w = img_raw.shape[:2]
    img_center = (img_w // 2, img_h // 2)
    cv2.circle(img_raw, img_center, 4, (0, 255, 0), -1)
    cv2.putText(img_raw, "Image Center", (img_center[0]+10, img_center[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)

    cv2.imshow('Rect True Center Detection', img_raw)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
