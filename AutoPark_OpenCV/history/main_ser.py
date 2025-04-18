#!/usr/bin/python3
import cv2
import numpy as np
import serial
import time

# 初始化串⼝
ser = serial.Serial(
    port="/dev/ttyTHS1",
    baudrate=115200,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
)
time.sleep(2)  # 等待串口稳定

#  GStreamer管道
gs_pipeline = (
    "v4l2src device=/dev/video0 ! "
    "image/jpeg,width=640,height=480,framerate=30/1 ! "
    "jpegparse ! "
    "queue max-size-buffers=3 leaky=downstream ! "  # 添加缓冲队列
    "nvv4l2decoder mjpeg=1 enable-max-performance=1 ! "
    "nvvidconv output-buffers=10 ! "  # 增加输出缓冲区
    "video/x-raw(memory:NVMM),format=RGBA ! "  # 显式指定内存类型
    "nvvidconv ! " 
    "video/x-raw,format=BGRx ! "
    "videoconvert ! "
    "video/x-raw,format=BGR ! "
    "appsink drop=1 sync=0 emit-signals=0"  # 禁用所有信号
)

frame_width , frame_height  =640 ,480 
center_x = frame_width // 2
center_y = frame_height // 2

# 初始化变量
prev_corners = []  # 初始化上一帧角点
corner_pass_count = 0  # 初始化角点经过中心计数
mode = "Approaching"  # 初始化模式

# 摄像头
# cap = cv2.VideoCapture(gs_pipeline, cv2.CAP_GSTREAMER)
cap = cv2.VideoCapture(0)
while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        print("无法获取图像")
        break

    #调整窗口大小
    frame = cv2.resize(frame,(frame_width, frame_height))
       
    # 初始化变量
    original_frame = frame.copy()


    # 第一部分：车道线检测
    # 预处理
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    _, binary = cv2.threshold(blurred, 30, 255, cv2.THRESH_BINARY_INV)
    kernel = np.ones((3, 3), np.uint8)
    binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
    binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)

    # 霍夫变换检测直线
    lines = cv2.HoughLinesP(binary, 1, np.pi / 180, threshold=50, minLineLength=100, maxLineGap=10)

    # 过滤竖直线
    lane_lines = [] # 水平车道线
    lane_centers_y = [] # 水平车道线的纵坐标
    avg_lane_y = center_y  # 默认值
    avg_angle = 0  # 默认角度
    
    # 初始化 quarter_y 和 three_quarter_y 为默认值
    quarter_y = center_y
    three_quarter_y = center_y

    if lines is not None:
        angles = []
        for line in lines:
            x1, y1, x2, y2 = line[0]
            angle = np.arctan2(y2 - y1, x2 - x1) * 180 / np.pi
            if abs(angle) < 60:  # 筛选近似水平线
                lane_lines.append(line)
                angles.append(angle)
#                cv2.line(original_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)                
                # 保存水平线的中心点的y坐标
                line_center_y = (y1 + y2) // 2
                lane_centers_y.append(line_center_y)
        
        # 计算平均角度
        if angles:
            avg_angle = sum(angles) / len(angles)
    
    # 计算所有水平线中心点的平均y坐标
    if lane_centers_y:
        avg_lane_y = sum(lane_centers_y) // len(lane_centers_y)

    # 水平参考线
    cv2.line(original_frame, (0, center_y), (frame_width, center_y), (255, 0, 0), 2)  
    
    # 绘制平均线
    if lane_lines:
        # 计算斜率 (tan)
        slope = np.tan(np.radians(avg_angle))
        
        # 计算平均线的两个端点
        left_x = 0
        left_y = int(avg_lane_y - slope * center_x)
        right_x = frame_width
        right_y = int(avg_lane_y + slope * (frame_width - center_x))
        
        # 确保坐标在图像范围内
        left_y = max(0, min(left_y, frame_height - 1))
        right_y = max(0, min(right_y, frame_height - 1))
        
        # 绘制检测到的平均线
        cv2.line(original_frame, (left_x, left_y), (right_x, right_y), (0, 255, 0), 2)
        
        # 标记平均线上的 1/4, 1/2, 3/4 点
        quarter_x = frame_width // 4
        half_x = frame_width // 2
        three_quarter_x = (frame_width * 3) // 4
        quarter_y = int(left_y + (right_y - left_y) * 0.25)
        half_y = int(left_y + (right_y - left_y) * 0.5)
        three_quarter_y = int(left_y + (right_y - left_y) * 0.75)
        cv2.circle(original_frame, (quarter_x, quarter_y), 5, (0, 165, 255), -1) 
        cv2.circle(original_frame, (half_x, half_y), 5, (0, 165, 255), -1) 
        cv2.circle(original_frame, (three_quarter_x, three_quarter_y), 5, (0, 165, 255), -1)
    
    # 计算error并putText
    error = center_y - avg_lane_y + center_y - quarter_y - (center_y - three_quarter_y)  # 考虑中点、1/4点与3/4点
    cv2.putText(original_frame, f"Error: {error}", (10, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    
    # 第二部分: 角点检测和模式切换
    # Harris 角点检测
    mode = "Approaching"
    corner_pass_count = 0
    prev_corners = []
    dst = cv2.cornerHarris(np.float32(gray), blockSize=2, ksize=3, k=0.04)
    
    # 膨胀角点响应图，使角点更加明显
    dst = cv2.dilate(dst, None)
    
    # 设置阈值，标记角点
    threshold = 0.01 * dst.max()
    
    # 提取角点坐标
    corners = []
    y_indices, x_indices = np.where(dst > threshold)
    for y, x in zip(y_indices, x_indices):
        corners.append((x, y))
    
    # 过滤角点，移除距离过近的点
    filtered_corners = []
    min_distance = 10  
    for corner in corners:
        current_point = np.array(corner)
        should_keep = True
        for existing_corner in filtered_corners:
            existing_point = np.array(existing_corner)
            distance = np.linalg.norm(current_point - existing_point)
            if distance <= min_distance:
                should_keep = False
                break
        if should_keep:
            filtered_corners.append(corner)
    corners = filtered_corners

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
        mode_code = 0  # 数字代码表示模式
    elif corner_pass_count == 1:
        mode = "Approaching"  # 第一个车库
        mode_code = 1
    elif corner_pass_count == 2:
        mode = "Approaching"  # 第二个车库
        mode_code = 2
    elif corner_pass_count == 3:
        mode = "Backing"  # 中间车库，进入倒车
        mode_code = 3

    # 在图像上绘制角点
    for corner in corners:
        x, y = corner
        cv2.circle(original_frame, (x, y), 5, (0, 255, 0), -1)
    
    # 在图像上绘制中心线
    cv2.line(original_frame, (center_x, 0), (center_x, frame_height), (255, 0, 0), 2)

    # 显示模式信息
    cv2.putText(original_frame, f'Mode: {mode}', (10, 50), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    cv2.putText(original_frame, f'Pass Count: {corner_pass_count}', (10, 70),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    # 显示图像
    cv2.imshow("Combined Tracking", original_frame)
#    cv2.imshow("Binary", binary)

    # 输出偏差值和模式信息
    print(f"车道偏差值: {error}, 模式: {mode}, 经过角点次数: {corner_pass_count}")

    # 构造定长数据包发送到串口
    # 格式: MSXXX，其中:
    # M = 模式 (0-9)
    # S = 符号位 (1=正, 0=负)
    # XXX = 偏差值的绝对值 (000-999)
    
    # 限制error范围，避免数据过长
    capped_error = max(min(int(error), 999), -999)
    
    # 确定符号位
    sign_bit = 1 if capped_error >= 0 else 0
    
    # 构造固定格式字符串：模式 + 符号位 + 错误绝对值(3位)
    data_str = "{}{}{:03d}\n".format(mode_code, sign_bit, abs(capped_error))
    
    # 输出发送的数据字符串
    print(f"发送到STM32: {data_str.strip()}")
    
    # 发送到串口
    ser.write(data_str.encode('utf-8'))

    # 更新上一帧角点
    prev_corners = corners.copy()

    # 按'q'退出
    if cv2.waitKey(30) & 0xFF == ord('q'):
        break

# 释放资源
cap.release()
cv2.destroyAllWindows()
