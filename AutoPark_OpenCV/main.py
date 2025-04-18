import cv2
import numpy as np
import math
import serial
import time

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

# 初始化串⼝
try:
    ser = serial.Serial(
        port="/dev/ttyTHS1",
        baudrate=115200,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
    )
    time.sleep(2)  # 等待串口稳定
    print("串口连接成功")
except Exception as e:
    print(f"串口连接失败: {str(e)}")
    ser = None

# 添加检查来自串口的重置命令的函数
def check_for_reset():
    if ser is not None and ser.is_open and ser.in_waiting > 0:
        try:
            data = ser.read(ser.in_waiting)
            for b in data:
                if b == 0xFF:  # 检查是否有重置标志 (0xFF)
                    print("收到重置命令")
                    return True
        except Exception as e:
            print(f"读取串口数据失败: {str(e)}")
    return False

# 打开视频流
cap = cv2.VideoCapture(gs_pipeline, cv2.CAP_GSTREAMER)
# cap = cv2.VideoCapture(0)  # 使用摄像头，或者替换为视频文件路径
if not cap.isOpened():
    print("无法打开视频流")
    exit()

# 定义状态变量
corner_pass_count = 0  # 初始化角点经过中心计数
last_points = []
DISTANCE_THRESHOLD = 20  # 相近点的距离阈值
mode = "Approaching"  # 初始化模式

while True:
    # 在每帧开始时检查是否有重置命令
    if check_for_reset():
        corner_pass_count = 0
        mode = "Approaching"
        mode_num = 0
        print("模式已重置为初始状态")
    
    ret, frame = cap.read()
    if not ret:
        print("无法读取帧，结束视频流")
        break

    # 计算画面中心点
    frame_height, frame_width = frame.shape[:2]
    frame_center_x = frame_width // 2
    frame_center_y = frame_height // 2

    # 预处理
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    _, binary = cv2.threshold(blurred, 80, 255, cv2.THRESH_BINARY_INV)

    kernel = np.ones((5, 5), np.uint8)
    binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
    binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)

    # Canny 边缘检测
    edges = cv2.Canny(binary, 50, 150, apertureSize=3)

    # 查找外部轮廓
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    result = frame.copy()
    bottom_line = None
    max_y = -1
    bottom_angle = None
    
    # 当前帧中的角点
    current_points = []

    # 遍历轮廓
    for contour in contours:
        pts = contour.reshape(-1, 2)

        # 近似轮廓为多边形
        epsilon = 0.01 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)

        # 获取 Y 坐标最大的两个点
        sorted_pts = sorted(approx, key=lambda x: x[0][1], reverse=True)  # 按 Y 坐标降序排列
        top_point = sorted_pts[0][0]  # Y 坐标最大点
        bottom_point = sorted_pts[1][0]  # 第二大点

        # 计算这两个点之间的直线角度
        dx = bottom_point[0] - top_point[0]
        dy = bottom_point[1] - top_point[1]
        if dx == 0:
            continue  # 避免除零错误

        # 计算角度
        angle = abs(math.degrees(math.atan2(dy, dx)))

        # 找到Y坐标最大（最下面）的线段
        if bottom_point[1] > max_y:
            max_y = bottom_point[1]
            bottom_line = (tuple(top_point), tuple(bottom_point))
            bottom_angle = angle

        # 强制闭合轮廓并检测角点
        for pt in approx:
            x, y = pt[0]
            current_points.append((x, y))
            cv2.circle(result, (x, y), 4, (0, 0, 255), -1)  # 红色圆点表示角点

    # 合并相近的角点
    merged_points = []
    for pt in current_points:
        is_close = False
        for idx, mp in enumerate(merged_points):
            dist = np.sqrt((pt[0] - mp[0])**2 + (pt[1] - mp[1])**2)
            if dist < DISTANCE_THRESHOLD:
                # 更新为平均位置
                merged_points[idx] = ((mp[0] + pt[0])//2, (mp[1] + pt[1])//2)
                is_close = True
                break
        if not is_close:
            merged_points.append(pt)

    # 检测角点穿过中心线的情况
    point_crossed = False  # 标记是否有点跨越中心线
    
    if last_points and merged_points:
        # 为每个当前点找到最接近的上一帧点
        for pt in merged_points:
            if point_crossed:  # 如果已经检测到穿越，跳出循环
                break
                
            closest_last_point = None
            min_distance = float('inf')
            
            for last_pt in last_points:
                dist = np.sqrt((pt[0] - last_pt[0])**2 + (pt[1] - last_pt[1])**2)
                if dist < min_distance:
                    min_distance = dist
                    closest_last_point = last_pt
            
            # 只有当距离小于阈值时才认为是同一个点
            if closest_last_point and min_distance < DISTANCE_THRESHOLD * 2:
                # 检查是否从左到右穿过中心线
                if closest_last_point[0] < frame_center_x and pt[0] > frame_center_x:
                    # 点必须在画面的中间部分高度范围内
                    if pt[1] > frame_height * 0.2 and pt[1] < frame_height * 0.8:
                        point_crossed = True
                        corner_pass_count += 1
                        # 可视化这个穿越点
                        cv2.circle(result, pt, 8, (255, 255, 0), -1)  # 黄色大圆表示穿越点
                        print(f"角点从左到右经过中心，第 {corner_pass_count} 次，坐标: {pt}")
    
    # 更新上一帧的点
    last_points = merged_points.copy()

    # 模式判断
    if corner_pass_count == 0:
        mode = "Approaching"
        mode_num = 0
    elif corner_pass_count == 1:
        mode = "Approaching"  # 第一个车库
        mode_num = 1
    elif corner_pass_count == 2:
        mode = "Approaching"  # 第二个车库
        mode_num = 2
    elif corner_pass_count == 3:
        mode = "Backing"  # 中间车库，进入倒车
        mode_num = 3
    elif corner_pass_count > 3:
        mode = f"Mode {corner_pass_count}"  # 其他模式
        mode_num = min(corner_pass_count, 7)  # 限制在0-7范围内

    # 绘制最下面的近水平线段
    if bottom_line:
        cv2.line(result, bottom_line[0], bottom_line[1], (0, 255, 0), 2)
        
        # 计算线段中点
        mid_x = (bottom_line[0][0] + bottom_line[1][0]) // 2
        mid_y = (bottom_line[0][1] + bottom_line[1][1]) // 2
        
        # 计算中点与画面中心的距离
        distance_to_center = abs(mid_y - frame_center_y)
        
        # 在图像上显示相关信息
        cv2.circle(result, (mid_x, mid_y), 5, (255, 0, 0), -1)  # 蓝色圆点表示线段中点
        cv2.putText(result, f"Angle: {bottom_angle:.2f}", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(result, f"Center Dist: {distance_to_center}px", (10, 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(result, f"Mode: {mode}", (10, 90), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(result, f"Pass Count: {corner_pass_count}", (10, 120),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        print(f"Bottom Line: Angle = {bottom_angle:.2f}°, Center Distance = {distance_to_center}px")
        print(f"模式: {mode}, 经过角点次数: {corner_pass_count}")
        
        # 发送数据到串口
        if ser is not None and ser.is_open:
            try:
                # 将模式编码为3位(0-7)，角度编码为5位(0-31)
                angle_int = min(31, int(abs(bottom_angle)))  # 限制在0-31范围
                byte1 = (mode_num << 5) | angle_int  # 模式占高3位，角度占低5位
                
                # 将distance_to_center编码为第二个字节(有符号，-128到127)
                # 计算相对于中心的距离（上方为负，下方为正）
                relative_dist = mid_y - frame_center_y
                # 确保在-128到127范围内
                clamped_dist = max(-128, min(127, relative_dist))
                byte2 = clamped_dist & 0xFF  # 将有符号转换为无符号发送
                
                # 发送两个字节
                data_to_send = bytearray([byte1, byte2])
                ser.write(data_to_send)
                
                # 显示发送的数据
                cv2.putText(result, f"Sent: Mode:{mode_num} Ang:{angle_int} Dist:{clamped_dist}", 
                          (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                
                print(f"发送数据: 模式={mode_num}, 角度={angle_int}, 距离={clamped_dist}")
                
            except Exception as e:
                print(f"发送数据失败: {str(e)}")

    # 画出帧中心位置
    cv2.line(result, (frame_center_x, 0), (frame_center_x, frame_height), (0, 0, 255), 1)
    cv2.line(result, (0, frame_center_y), (frame_width, frame_center_y), (0, 0, 255), 1)

    # 显示结果
    cv2.imshow("Bottom Line & Corners", result)

    # 按 'q' 键退出
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break

# 释放资源
if ser is not None and ser.is_open:
    ser.close()
    print("串口已关闭")
cap.release()
cv2.destroyAllWindows()
