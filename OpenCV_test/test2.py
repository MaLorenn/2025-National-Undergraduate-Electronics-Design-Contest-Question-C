import cv2
import numpy as np

def nothing(x):
    pass

# 初始化摄像头
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 960)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 540)
cap.set(cv2.CAP_PROP_FPS, 60)

# 创建窗口和滑动条
cv2.namedWindow("Binarization")
cv2.createTrackbar("Thresh", "Binarization", 90, 255, nothing)
cv2.createTrackbar("Kernel", "Binarization", 3, 15, nothing)    # 形态学核，0表示跳过
cv2.createTrackbar("Gauss", "Binarization", 5, 15, nothing)     # 高斯核，0表示跳过

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # 获取滑动条参数
    thresh = cv2.getTrackbarPos("Thresh", "Binarization")
    kernel_size = cv2.getTrackbarPos("Kernel", "Binarization")
    gauss_size = cv2.getTrackbarPos("Gauss", "Binarization")

    # CUDA灰度
    gpu_frame = cv2.cuda_GpuMat()
    gpu_frame.upload(frame)
    gpu_gray = cv2.cuda.cvtColor(gpu_frame, cv2.COLOR_BGR2GRAY)

    # CUDA高斯滤波（0表示跳过）
    if gauss_size < 3:
        blur_cpu = gpu_gray.download()
    else:
        if gauss_size % 2 == 0:
            gauss_size += 1
        gauss_filter = cv2.cuda.createGaussianFilter(cv2.CV_8UC1, cv2.CV_8UC1, (gauss_size, gauss_size), 0)
        gpu_blur = gauss_filter.apply(gpu_gray)
        blur_cpu = gpu_blur.download()

    # 形态学闭运算（0表示跳过）
    if kernel_size < 3:
        blur_close = blur_cpu
    else:
        if kernel_size % 2 == 0:
            kernel_size += 1
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (kernel_size, kernel_size))
        blur_close = cv2.morphologyEx(blur_cpu, cv2.MORPH_CLOSE, kernel)

    # 二值化
    _, binary = cv2.threshold(blur_close, thresh, 255, cv2.THRESH_BINARY_INV)

    # 显示结果
    cv2.imshow("Gray", blur_close)
    cv2.imshow("Binarization", binary)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
