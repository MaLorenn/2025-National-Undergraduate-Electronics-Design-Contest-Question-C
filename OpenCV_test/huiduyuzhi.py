import cv2

def show_gray_value(event, x, y, flags, param):
    global img
    if event == cv2.EVENT_LBUTTONDOWN:
        if len(img.shape) == 3 and img.shape[2] == 3:  # 彩色图
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        else:
            gray = img
        value = gray[y, x]
        print(f"位置({x},{y})的灰度值: {value}")
        temp = img.copy()
        cv2.circle(temp, (x, y), 4, (0, 255, 0), -1)
        cv2.putText(temp, str(value), (x+10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)
        cv2.imshow('image', temp)

cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))  # MJPG usually supports higher fps
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 960)   # 先设置为支持60fps的分辨率
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 540)
cap.set(cv2.CAP_PROP_FPS, 60)
ret, img = cap.read()
if not ret:
    print("无法打开摄像头")
    cap.release()
    exit(0)

cv2.namedWindow('image')
cv2.setMouseCallback('image', show_gray_value)

while True:
    ret, img = cap.read()
    if not ret:
        break
    cv2.imshow('image', img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
