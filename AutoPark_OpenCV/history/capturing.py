import cv2
import numpy as np

mode = "Approaching"
corner_pass_count = 0
prev_corners = []

frame_width , frame_height  =640 ,480 
center_x = frame_width // 2
center_y = frame_height // 2

# GStreamer管道
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


# 摄像头
cap = cv2.VideoCapture(gs_pipeline, cv2.CAP_GSTREAMER)
while cap.isOpened():
    ret, frame = cap.read()  # 读取每一帧
    if not ret:
        print("无法获取图像")
        break
    
    # 初始化变量
    original_frame = frame.copy()

    #调整窗口大小
    frame = cv2.resize(frame,(frame_width, frame_height))

    
    # 第二部分: 角点检测和模式切换
    # 转换为灰度图
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # 高斯模糊平滑图像
    gray_blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    # 形态学操作
    kernel = np.ones((4, 4), np.uint8)
    gray_processed = cv2.morphologyEx(gray_blurred, cv2.MORPH_OPEN, kernel)
    gray_processed = cv2.morphologyEx(gray_processed, cv2.MORPH_CLOSE, kernel)
    
    # Harris 角点检测
    dst = cv2.cornerHarris(np.float32(gray_processed), blockSize=2, ksize=3, k=0.04)

    # 膨胀角点响应图，使角点更加明显
    dst = cv2.dilate(dst, None)

    # 设置阈值，标记角点
    threshold = 0.01 * dst.max()
    frame_copy = frame.copy()
    frame_copy[dst > threshold] = [0, 0, 255]  # 红色
    
    # 提取角点坐标
    corners = []
    y_indices, x_indices = np.where(dst > threshold)
    for y, x in zip(y_indices, x_indices):
        corners.append((x, y))
    
    # 创建二值图像用于显示
    _, binary = cv2.threshold(gray_processed, 127, 255, cv2.THRESH_BINARY)
    
    
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
    elif corner_pass_count == 1:
        mode = "Approaching"  # 第一个车库
    elif corner_pass_count == 2:
        mode = "Approaching"  # 第二个车库
    elif corner_pass_count == 3:
        mode = "Backing"  # 中间车库，进入倒车

    # 在图像上绘制角点
    for corner in corners:
        x, y = corner
        cv2.circle(original_frame, (x, y), 5, (0, 255, 0), -1)
    
    # 在图像上绘制中心线
    cv2.line(original_frame, (center_x, 0), (center_x, frame_height), (255, 0, 0), 2)

    # 显示模式信息 
    cv2.putText(original_frame, f'Mode: {mode}', (10, 60), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    cv2.putText(original_frame, f'Pass Count: {corner_pass_count}', (10, 90),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    # 显示图像
    cv2.imshow("Combined Tracking", original_frame)
    cv2.imshow("Binary", binary)


    # 更新上一帧角点
    prev_corners = corners.copy()

    # 按'q'退出
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 释放资源
cap.release()
cv2.destroyAllWindows()
