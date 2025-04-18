import cv2
import numpy as np

# 初始化参数
minLineLength = 100
maxLineGap = 10
threshold = 200

# 创建窗口
cv2.namedWindow('Trackbar Window')

# 定义Trackbar回调函数（如果有需要）
def nothing(x):
    pass

# 创建Trackbar
cv2.createTrackbar('Threshold', 'Trackbar Window', threshold, 500, nothing)  # 设置阈值范围（0-500）
cv2.createTrackbar('Min Line Length', 'Trackbar Window', minLineLength, 500, nothing)  # 设置最小线长范围（0-500）
cv2.createTrackbar('Max Line Gap', 'Trackbar Window', maxLineGap, 100, nothing)  # 设置最大线间距范围（0-100）

# 加载图片
image = cv2.imread('./test_1.jpg')

# 设置窗口大小
cv2.resizeWindow('Trackbar Window', 600, 400)  # 设置Trackbar窗口大小为600x400

while True:
    # 获取Trackbar值
    threshold = cv2.getTrackbarPos('Threshold', 'Trackbar Window')
    minLineLength = cv2.getTrackbarPos('Min Line Length', 'Trackbar Window')
    maxLineGap = cv2.getTrackbarPos('Max Line Gap', 'Trackbar Window')

    # 转为灰度图像
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # 使用Canny边缘检测
    edges = cv2.Canny(gray, 50, threshold)

    # 直线检测（Hough变换）
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold, minLineLength=minLineLength, maxLineGap=maxLineGap)

    # 创建一个拷贝图像来绘制直线
    img_with_lines = image.copy()

    # 如果检测到直线，绘制出来
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(img_with_lines, (x1, y1), (x2, y2), (0, 255, 0), 2)

    # 显示带有检测直线的图像
    cv2.imshow('Detected Lines', img_with_lines)

    # 按'q'键退出
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 释放资源
cv2.destroyAllWindows()
