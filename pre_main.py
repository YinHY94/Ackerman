import cv2
import numpy as np

# 初始化视频源，全局变量
cap = cv2.VideoCapture("video_1.mp4")
mode = "Approaching"
corner_pass_count = 0
prev_corners = []

# 主循环
while True:
    ret, frame = cap.read()
    if not ret:
        print("无法获取视频帧")
        break

    # 调整图像大小
    frame_width = 320
    frame_height = 240
    frame = cv2.resize(frame, (frame_width, frame_height))

    # 定义参考线
    center_y = frame_height // 2
    center_x = frame_width // 2

    # 图像预处理
    original_frame = frame.copy()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    _, binary = cv2.threshold(blurred, 100, 255, cv2.THRESH_BINARY_INV)
    kernel = np.ones((3, 3), np.uint8)
    binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
    binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)

    # 霍夫变换检测直线并过滤竖直线
    lines = cv2.HoughLinesP(binary, 1, np.pi/180, threshold=50, 
                        minLineLength=50, maxLineGap=10)
    
    # 计算像素与厘米的转换比例（假设，需要根据实际情况调整）
    pixels_per_cm = 10  # 假设每厘米对应10个像素
    parallel_distance_cm = 5  # 5厘米
    parallel_distance_pixels = int(parallel_distance_cm * pixels_per_cm)
    
    lane_lines = [] # 水平车道线
    lane_centers_y = [] # 水平车道线的纵坐标
    avg_lane_y = center_y  # 默认值
    avg_angle = 0  # 默认角度
    
    if lines is not None:
        angles = []
        for line in lines:
            x1, y1, x2, y2 = line[0]
            angle = np.arctan2(y2 - y1, x2 - x1) * 180 / np.pi
            if abs(angle) < 60:  # 筛选近似水平线
                lane_lines.append(line)
                angles.append(angle)
                cv2.line(original_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)                
                # 保存水平线的中心点的y坐标
                line_center_y = (y1 + y2) // 2
                lane_centers_y.append(line_center_y)
        
        # 计算平均角度
        if angles:
            avg_angle = sum(angles) / len(angles)
    
    # 计算所有水平线中心点的平均y坐标
    if lane_centers_y:
        avg_lane_y = sum(lane_centers_y) // len(lane_centers_y)
    
    # 计算平行线位置（在检测到的水平线下方5cm）
    parallel_line_y = min(avg_lane_y + parallel_distance_pixels, frame_height - 1)
    
    # 计算平行线质心到画面底部的距离
    distance_to_bottom = frame_height - parallel_line_y
    
    # 绘制与检测到的线平行的参考线
    if lane_lines:
        # 计算斜率 (tan(角度))
        slope = np.tan(np.radians(avg_angle))
        
        # 计算平行线的两个端点
        # 使用中心点和斜率计算参考线的左右端点
        left_x = 0
        left_y = int(avg_lane_y - slope * center_x)
        right_x = frame_width
        right_y = int(avg_lane_y + slope * (frame_width - center_x))
        
        # 确保坐标在图像范围内
        left_y = max(0, min(left_y, frame_height - 1))
        right_y = max(0, min(right_y, frame_height - 1))
        
        # 绘制检测到的水平线的平均线
        cv2.line(original_frame, (left_x, left_y), (right_x, right_y), (0, 255, 0), 2)  # 绿色检测线
        
        # 计算平行线的端点（下移5cm）
        parallel_left_y = min(left_y + parallel_distance_pixels, frame_height - 1)
        parallel_right_y = min(right_y + parallel_distance_pixels, frame_height - 1)
        
        # 绘制平行线
        cv2.line(original_frame, (left_x, parallel_left_y), (right_x, parallel_right_y), (0, 0, 255), 2)  # 红色平行线
    else:
        # 如果没有检测到线，则绘制水平线
        cv2.line(original_frame, (0, avg_lane_y), (frame_width, avg_lane_y), (0, 255, 0), 2)  # 绿色检测线
        cv2.line(original_frame, (0, parallel_line_y), (frame_width, parallel_line_y), (0, 0, 255), 2)  # 红色平行线
    
    # 显示距离信息
    cv2.putText(original_frame, f"Distance to bottom: {distance_to_bottom}px", (10, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    cv2.putText(original_frame, f"Angle: {avg_angle:.2f} deg", (10, 50),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

    # 第二部分: 角点检测和模式切换
    # 使用 Harris 角点检测
    harris_corners = cv2.cornerHarris(gray, blockSize=2, ksize=3, k=0.04)
    harris_corners = cv2.dilate(harris_corners, None)  # 膨胀角点以便更好地显示

    # 设置阈值以筛选角点
    threshold = 0.01 * harris_corners.max()
    corners = np.argwhere(harris_corners > threshold)
    corners = [(int(x), int(y)) for y, x in corners]

    # 绘制角点
    for corner in corners:
        x, y = corner
        cv2.circle(original_frame, (x, y), 5, (0, 255, 0), -1)

    # 检查角点经过中心
    if corners and prev_corners:
        for curr_corner in corners:
            x_curr, _ = curr_corner
            # 找到最接近的上一帧角点
            min_dist = float('inf')
            closest_prev = None
            for prev_corner in prev_corners:
                x_prev, _ = prev_corner
                dist = abs(x_curr - x_prev)
                if dist < min_dist:
                    min_dist = dist
                    closest_prev = prev_corner
            if closest_prev:
                x_prev, _ = closest_prev
                # 从左到右经过中心
                if x_prev < center_x and x_curr >= center_x:
                    corner_pass_count += 1
                    print(f"角点从左到右经过中心，第 {corner_pass_count} 次")
                    break  # 每帧只计数一次

    # 模式判断
    if corner_pass_count == 0:
        mode = "Approaching"
    elif corner_pass_count == 1:
        mode = "Approaching"  # 第一个车库
    elif corner_pass_count == 2:
        mode = "Approaching"  # 第二个车库
    elif corner_pass_count == 3:
        mode = "Backing"  # 中间车库，进入倒车

    # 显示模式信息
    cv2.putText(original_frame, f'Mode: {mode}', (10, 70), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    cv2.putText(original_frame, f'Pass Count: {corner_pass_count}', (10, 90),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    # 显示图像
    cv2.waitKey(1)  # 等待1毫秒 
    cv2.imshow("Combined Tracking", original_frame)
    cv2.imshow("Binary", binary)

    # 输出偏差值和模式信息
    print(f"车道偏差值: {avg_lane_y - center_y}, 模式: {mode}, 经过角点次数: {corner_pass_count}")

    # 更新上一帧角点
    prev_corners = corners.copy()

    # 按'q'退出
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 释放资源
cap.release()
cv2.destroyAllWindows()