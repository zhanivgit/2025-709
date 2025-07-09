import sensor, image, time, pyb
import display
import ml

# --- 初始化摄像头 ---
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA) # 恢复为QVGA分辨率
sensor.skip_frames(time=2000)
sensor.set_vflip(True) # 摄像头模块的设置
sensor.set_hmirror(True) # 摄像头模块的设置

# --- 初始化UART和LCD ---
uart = pyb.UART(3, 9600)
lcd = display.SPIDisplay()

# --- 加载数字识别模型 ---
model = ml.Model("trained.tflite", load_to_fb=True)
norm = ml.Normalization(scale=(0, 1.0))
print("成功加载TFLite模型。")

# --- 阈值定义 ---
red_threshold = (0, 100, 20, 127, 0, 127) # 红色循迹线和红色十字检测的阈值
cross_area_threshold = 15000 # 用于判断红色十字的面积阈值，需要根据实际情况调整

clock = time.clock()
last_print_time = 0

# --- 状态机 ---
STATE_LEARNING, STATE_TRACKING, STATE_DETECTING, STATE_CROSSING, STATE_ARRIVED = 0, 1, 2, 3, 4
current_state = STATE_LEARNING
target_number = None
crossing_timer = 0
line_lost_timer = 0
LINE_LOST_THRESHOLD = 1000 # 丢线1秒后停止

# --- 学习确认机制 ---
LEARNING_CONFIRMATION_FRAMES = 20 # 需要连续识别20次相同才确认
learning_buffer = []

print("请将要学习的数字放在摄像头中央...")

while True:
    clock.tick()
    img = sensor.snapshot()

    # --- 状态机逻辑 ---
    if current_state == STATE_LEARNING:              # 学习状态
        # 步骤 1: 在屏幕中央寻找数字进行学习
        img_w, img_h = img.width(), img.height()
        learn_roi_rect = (img_w // 2 - 50, img_h // 2 - 30, 100, 100)
        img.draw_rectangle(learn_roi_rect, color=(255, 255, 0)) # 黄色学习框

        # 从ROI中提取图像并进行处理
        roi_img = img.copy(roi=learn_roi_rect)
        processed_img = roi_img.binary([(0, 60)]).dilate(2)
        # 运行数字识别
        input_data = [norm(processed_img)]
        result = model.predict(input_data)[0].flatten().tolist()
        confidence = max(result)
        predicted_number = result.index(confidence)
        # 步骤 2: 检查识别置信度并进行学习确认
        if confidence > 0.8:
            learning_buffer.append(predicted_number)
            if len(learning_buffer) > LEARNING_CONFIRMATION_FRAMES:
                learning_buffer.pop(0)

            if len(learning_buffer) == LEARNING_CONFIRMATION_FRAMES and all(n == learning_buffer[0] for n in learning_buffer):
                target_number = learning_buffer[0]
                uart.write("0\n") # 发送启动循迹指令
                current_state = STATE_TRACKING # 学习完成后切换到循迹状态
                print("已稳定识别并学习数字: %d. 发送指令 0, 现在开始循迹." % target_number)
        else:
            learning_buffer.clear()

    elif current_state == STATE_TRACKING:                                # 循迹状态
        # 红色十字检测 (通过面积区分)
        blobs = img.find_blobs([red_threshold], pixels_threshold=200, area_threshold=200, merge=True)
        found_cross = False
        for blob in blobs:
            if blob.area() > cross_area_threshold:
                # 如果检测到足够大的红色色块，认为是十字，切换到检测状态
                current_state = STATE_DETECTING
                print("检测到红色十字 (面积: %d)，切换到数字检测状态。" % blob.area())
                found_cross = True
                break
        if found_cross:
            continue # 跳过当前帧的循迹逻辑，直接进入下一帧的检测状态

        # 红色循迹逻辑
        img_binary = img.binary([red_threshold])
        line = img_binary.get_regression([(100,100)], robust = True)

        if (line):
            rho_err = abs(line.rho())-img.width()/2
            if line.theta()>90:
                theta_err = line.theta()-180
            else:
                theta_err = line.theta()
            img_binary.draw_line(line.line(), color = 127)
            # lcd.write(img_binary,x_scale=-0.5,y_scale=-0.5) # 循迹时不再显示LCD，避免卡顿
            uart.write("@%.1f,%.1f,%d\r\n" % (
                rho_err,
                theta_err,
                line.magnitude()
            ))
            print("正常循迹中... 偏差:%.1fpx 角度:%.1f°" % (rho_err, theta_err))
            # 只要找到线，就重置计时器
            line_lost_timer = time.ticks_ms()
        else:
            # lcd.write(img_binary,x=160,y=90,x_scale=0.5,y_scale=0.5)
            uart.write("@null\r\n")
            print("正常循迹中... 未检测到线")
            # 检查丢线时间是否超过阈值
            if time.ticks_diff(time.ticks_ms(), line_lost_timer) > LINE_LOST_THRESHOLD:
                print("丢线超过1秒，判断为到达终点！发送停止指令。")
                uart.write("H\n") # 发送最终停止指令
                current_state = STATE_ARRIVED # 切换到完成状态

    elif current_state == STATE_DETECTING:              # 数字检测状态
        # 步骤 1: 寻找红色十字
        print("进入数字检测状态...") # 添加这行
        blobs = img.find_blobs([red_threshold], pixels_threshold=200, area_threshold=cross_area_threshold, merge=True)
        if not blobs:
            # 如果红色十字消失，则返回循迹状态
            current_state = STATE_TRACKING
            print("红色十字消失，返回循迹状态。")
            continue

        cross_blob = max(blobs, key=lambda b: b.area())
        img.draw_cross(cross_blob.cx(), cross_blob.cy(), color=(0, 0, 0), size=10)

        # --- 边界检查 ---
        img_w, img_h = img.width(), img.height()
        if (cross_blob.cx() < 80 or
            cross_blob.cx() + 80 > img_w or
            cross_blob.cy() + 80 > img_h):
            print("警告: 红色十字太靠近图像边缘，跳过数字识别。")
            current_state = STATE_TRACKING
            continue

        if target_number is None:
            current_state = STATE_TRACKING
            print("没有目标数字，返回循迹状态。")
            continue

        # 步骤 2: 在十字周围识别目标数字
        roi_definitions = {
            "left_num": (cross_blob.cx() - 45, cross_blob.cy() + 5, 40, 40), # 左侧数字的ROI
            "right_num": (cross_blob.cx() + 5, cross_blob.cy() + 5, 40, 40) # 右侧数字的ROI
        }

        found_target_number = False
        for r_name, r_rect in roi_definitions.items():
            img.draw_rectangle(r_rect, color=(0, 0, 255))

            roi_img = img.copy(roi=r_rect)
            processed_img = roi_img.binary([(0, 60)])

            input_data = [norm(processed_img)]
            result = model.predict(input_data)[0].flatten().tolist()
            confidence = max(result)
            predicted_number = result.index(confidence)

            # 添加这行，打印每次识别到的数字和置信度
            print("ROI %s 识别: 数字 %d, 置信度 %.2f" % (r_name, predicted_number, confidence))

            if predicted_number == target_number and confidence > 0.7:
                img.draw_rectangle(r_rect, color=(255, 255, 0)) # 将边框颜色改为黄色
                img.draw_string(r_rect[0], r_rect[1] - 15, str(predicted_number), color=(0, 255, 0), scale=2)
                print("在 %s 区域找到目标数字: %d (置信度: %.2f)" % (r_name, predicted_number, confidence))

                if r_name == "left_num":
                    uart.write("1\n")
                    print("发送指令: 1")
                elif r_name == "right_num":
                    uart.write("2\n")
                    print("发送指令: 2")

                found_target_number = True
                time.sleep_ms(500) # 发送指令后延时，确保STM32有时间处理
                break # 找到后立即跳出循环
            else:
                img.draw_rectangle(r_rect, color=(255, 0, 0))

        # --- 决策点 ---
        if not found_target_number:
            # 没找到目标，切换到“穿越路口”状态，并启动计时器
            print("当前路口非目标，进入穿越状态...")
            current_state = STATE_CROSSING
            crossing_timer = time.ticks_ms() # 记录当前时间
        else:
            # 找到了目标，切换回正常循迹状态，准备进入病房
            print("找到目标，恢复正常循迹以进入病房...")
            current_state = STATE_TRACKING
            # 重置丢线计时器，准备进入最后的循迹
            line_lost_timer = time.ticks_ms()

    elif current_state == STATE_CROSSING:
        # 在这个状态下，我们强制执行循迹，完全不检测十字路口

        # 1. 执行循迹
        img_binary = img.binary([red_threshold])
        line = img_binary.get_regression([(100,100)], robust = True)

        if (line):
            rho_err = abs(line.rho())-img.width()/2
            if line.theta()>90:
                theta_err = line.theta()-180
            else:
                theta_err = line.theta()
            img_binary.draw_line(line.line(), color = 127)
            uart.write("@%.1f,%.1f,%d\r\n" % (
                rho_err,
                theta_err,
                line.magnitude()
            ))
            print("穿越中... 偏差:%.1fpx 角度:%.1f°" % (rho_err, theta_err))
        else:
            uart.write("@null\r\n")
            print("穿越中... 未检测到线")

        # 2. 检查是否应该结束穿越状态 (强制循迹500毫秒)
        if time.ticks_diff(time.ticks_ms(), crossing_timer) > 500:
            print("穿越完成，恢复正常循迹。")
            current_state = STATE_TRACKING # 切换回正常的循迹状态

    elif current_state == STATE_ARRIVED:
        # 到达目的地，彻底停止
        # 可以在这里加点亮LED等操作
        while(True):
            pass
