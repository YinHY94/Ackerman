import cv2
import numpy as np

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
    ret, frame = cap.read()
    if not ret:
        print("Failed to capture image")
        break

    #调整窗口大小
    frame = cv2.resize(frame,(frame_width, frame_height))
    
    #中心点
    frame_height, frame_width = frame.shape[:2]
    center_x = frame_width // 2
    center_y = frame_height // 2

    #预处理
    original = frame.copy()
    gray = cv2.cvtColor(original, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    _, binary = cv2.threshold(blurred,30,255,cv2.THRESH_BINARY_INV)
    kernel = np.ones((3, 3), np.uint8)
    binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
    binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)

    #霍夫变换检测直线
    lines = cv2.HoughLinesP(binary, 1, np.pi / 180, threshold=50, minLineLength=100, maxLineGap=10)

    #过滤竖直线
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
                cv2.line(original, (x1, y1), (x2, y2), (0, 255, 0), 2)                
                # 保存水平线的中心点的y坐标
                line_center_y = (y1 + y2) // 2
                lane_centers_y.append(line_center_y)
        
        # 计算平均角度
        if angles:
            avg_angle = sum(angles) / len(angles)
    
    # 计算所有水平线中心点的平均y坐标
    if lane_centers_y:
        avg_lane_y = sum(lane_centers_y) // len(lane_centers_y)

    #水平参考线
    cv2.line(original, (0, center_y), (frame_width, center_y), (255, 0, 0), 2)  
    
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
        cv2.line(original, (left_x, left_y), (right_x, right_y), (0, 255, 0), 2)
        
        # 标记平均线上的 1/4, 1/2, 3/4 点
        quarter_x = frame_width // 4
        half_x = frame_width // 2
        three_quarter_x = (frame_width * 3) // 4
        quarter_y = int(left_y + (right_y - left_y) * 0.25)
        half_y = int(left_y + (right_y - left_y) * 0.5)
        three_quarter_y = int(left_y + (right_y - left_y) * 0.75)
        cv2.circle(original, (quarter_x, quarter_y), 5, (0, 165, 255), -1) 
        cv2.circle(original, (half_x, half_y), 5, (0, 165, 255), -1) 
        cv2.circle(original, (three_quarter_x, three_quarter_y), 5, (0, 165, 255), -1)
        
    #计算error并putText
    error=center_y-avg_lane_y + center_y-quarter_y -(center_y-three_quarter_y)#考虑中点、/4点与3/4点
    cv2.putText(original, "Error: {}".format(error), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    #显示图像
    cv2.waitKey(1)
    cv2.imshow("Lane Detection", original)
    cv2.imshow("Binary", binary)

    #print信息
    print("error: {}".format(error))

    #按q键退出
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()
