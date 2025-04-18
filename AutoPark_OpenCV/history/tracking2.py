import cv2
import numpy as np

# 设置图像尺寸
frame_width , frame_height  =640 ,480 
center_x = frame_width // 2
center_y = frame_height // 2

# 读取测试图片
frame = cv2.imread('test_1.jpg')

# 如果图像加载失败，退出
if frame is None:
    print("Failed to load image")
    exit()

# 调整窗口大小
frame = cv2.resize(frame, (frame_width, frame_height))

# 中心点坐标
frame_height, frame_width = frame.shape[:2]
center_x = frame_width // 2
center_y = frame_height // 2

# 预处理
original = frame.copy()
gray = cv2.cvtColor(original, cv2.COLOR_BGR2GRAY)
blurred = cv2.GaussianBlur(gray, (5, 5), 0)
_, binary = cv2.threshold(blurred, 50, 255, cv2.THRESH_BINARY_INV)
kernel = np.ones((5, 5), np.uint8)
binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)

# Canny边缘检测
edges = cv2.Canny(binary, 50, 150)

# 查找轮廓
contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# 绘制轮廓到新图像上
contour_img = np.zeros_like(binary)
cv2.drawContours(contour_img, contours, -1, 255, 2)  # 只绘制边缘，厚度为1
cv2.imshow("Contours", contour_img)

# 筛选水平轮廓
lane_lines = []  # 水平车道线
lane_centers_y = []  # 水平车道线的纵坐标
avg_lane_y = center_y  # 默认值
avg_angle = 0  # 默认角度

for contour in contours:
    if len(contour) > 5:  # 至少有5个点，适合拟合
        # 计算轮廓的拟合直线
        [vx, vy, x0, y0] = cv2.fitLine(contour, cv2.DIST_L2, 0, 0.01, 0.01)

        # 计算角度（倾斜度），只筛选近似水平的线
        angle = np.arctan2(vy, vx) * 180 / np.pi
        if abs(angle) < 10:  # 如果直线的角度接近水平
            # 获取线段的两端点
            left_x = 0
            left_y = int(y0 - (x0 - left_x) * vy / vx)
            right_x = frame_width
            right_y = int(y0 + (right_x - x0) * vy / vx)

            # 确保坐标在图像范围内
            left_y = max(0, min(left_y, frame_height - 1))
            right_y = max(0, min(right_y, frame_height - 1))

            # 绘制检测到的直线
            cv2.line(original, (left_x, left_y), (right_x, right_y), (0, 255, 0), 2)

            # 保存水平线的中心点的y坐标
            lane_centers_y.append((left_y + right_y) // 2)

# 计算所有水平线中心点的平均y坐标
if lane_centers_y:
    avg_lane_y = sum(lane_centers_y) // len(lane_centers_y)

# 水平参考线
cv2.line(original, (0, center_y), (frame_width, center_y), (255, 0, 0), 2)

# 计算误差并添加文本
error = center_y - avg_lane_y  # 计算中心点偏差
cv2.putText(original, "Error: {}".format(error), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

# 显示图像
cv2.imshow("Lane Detection", original)
cv2.imshow("Binary", binary)

# 打印误差信息
print("error: {}".format(error))

# 按键退出
cv2.waitKey(0)
cv2.destroyAllWindows()
