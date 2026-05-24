# 🚗 NUEDC-2025

## 📁 项目说明

- `NUEDC-pid`：小车底盘和云台代码  
- `GPIO/uart.py`：比赛时使用的串口通信及图像采集代码  
- `opencv` 和 `OpenCV_test`：OpenCV 练习代码  
- `tensorrtx-yolov5-v6.0` 和 `yolov5-6.0`：Jetson Nano 上部署含 TensorRT 加速的 YOLOv5（手动编译带 CUDA 加速的 OpenCV），重点关注：

```text
tensorrtx-yolov5-v6.0/yolov5/yolov5.cpp
```

具体细节：

- https://blog.csdn.net/baihei23311/article/details/149579053?spm=1001.2014.3001.5501
- https://blog.csdn.net/baihei23311/article/details/149617428?spm=1001.2014.3001.5501
- https://blog.csdn.net/baihei23311/article/details/149714305?spm=1001.2014.3001.5501
- https://blog.csdn.net/baihei23311/article/details/149785409?spm=1001.2014.3001.5501
