import cv2

# 打开相机
cap = cv2.VideoCapture(6)

# 设置相机的分辨率
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

orb = cv2.ORB_create()

# 定义窗口名字
window_name = "Camera with ORB Keypoints"

# 注册鼠标事件的回调函数
cv2.namedWindow(window_name, 0)
# cv2.resizeWindow(window_name, 1280, 720)

while True:
    # 读取相机图像
    ret, frame = cap.read()

    # 检查图像是否成功读取
    if not ret:
        print("无法读取相机图像")
        break

    # print(type(frame))  # <class 'numpy.ndarray'>

    # 转换图像为灰度图
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # 检测ORB特征点
    keypoints = orb.detect(gray, None)

    # 绘制ORB特征点
    img_with_keypoints = cv2.drawKeypoints(frame, keypoints, None, color=(0, 255, 0), flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    # 显示图像
    cv2.imshow(window_name, img_with_keypoints)

    # 检测按键事件，按下 "q" 键退出程序
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 关闭相机和窗口
cap.release()
cv2.destroyAllWindows()

