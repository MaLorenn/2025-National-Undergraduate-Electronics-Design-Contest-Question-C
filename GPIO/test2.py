import cv2
import serial

def mode_gray_click(cap):
    # ...灰度点击显示的实现...
    print("== 模式0：灰度点击显示 ==")
    # 可选：放在子循环里
    pass

def mode_roi_contour(cap):
    # ...ROI轮廓检测的实现...
    print("== 模式1：ROI轮廓检测 ==")
    pass

def mode_roi_preview(cap):
    # ...ROI原图预览的实现...
    print("== 模式2：只显示ROI原图 ==")
    pass

if __name__ == "__main__":
    # 打开串口
    ser = serial.Serial('/dev/ttyTHS1', 9600, timeout=0.1)
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    mode = 0  # 默认状态0

    while True:
        # 串口切换判断
        if ser.in_waiting:
            data = ser.read(ser.in_waiting).decode(errors='ignore')
            if '1' in data:
                mode = 1
            elif '2' in data:
                mode = 2
            elif '0' in data:
                mode = 0

        # 根据模式执行不同逻辑
        if mode == 0:
            # 灰度点击显示
            # 你可以把下面的内容换成你的函数
            ret, frame = cap.read()
            if not ret:
                break
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            cv2.imshow("Mode 0: Gray Click", gray)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        elif mode == 1:
            # ROI轮廓检测
            x1, y1, x2, y2 = 496, 284, 717, 440
            xmin, xmax = min(x1, x2), max(x1, x2)
            ymin, ymax = min(y1, y2), max(y1, y2)
            ret, frame = cap.read()
            if not ret:
                break
            roi = frame[ymin:ymax, xmin:xmax]
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            _, binary = cv2.threshold(gray, 190, 255, cv2.THRESH_BINARY_INV)
            contours, hierarchy = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            roi_output = roi.copy()
            if hierarchy is not None:
                for i, cnt in enumerate(contours):
                    area = cv2.contourArea(cnt)
                    if area > 10:
                        cv2.drawContours(roi_output, [cnt], -1, (0, 0, 255), 2)
            cv2.imshow('Mode 1: ROI Contours', roi_output)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        elif mode == 2:
            # 只显示ROI原图
            x1, y1, x2, y2 = 496, 284, 717, 440
            xmin, xmax = min(x1, x2), max(x1, x2)
            ymin, ymax = min(y1, y2), max(y1, y2)
            ret, frame = cap.read()
            if not ret:
                break
            roi = frame[ymin:ymax, xmin:xmax]
            cv2.imshow("Mode 2: ROI Preview", roi)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    cap.release()
    ser.close()
    cv2.destroyAllWindows()

