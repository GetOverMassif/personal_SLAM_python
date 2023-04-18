import cv2

# 打开相机
cap = cv2.VideoCapture(6)

while True:
    # 读取相机图像
    ret, frame = cap.read()

    # 检查图像是否成功读取
    if not ret:
        print("无法读取相机图像")
        break

    # 显示图像
    cv2.imshow("Camera", frame)

    # 检测按键事件，按下 "q" 键退出程序
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 关闭相机和窗口
cap.release()
cv2.destroyAllWindows()
