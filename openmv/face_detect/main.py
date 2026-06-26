"""
OpenMV H7 Plus - 人脸检测
========================

硬件: OpenMV H7 Plus (STM32H743VI + OV5640)
方式: Haar Cascade 正面人脸,每帧检测,选最大人脸 + EMA 平滑 + 短时记忆
输出: 画框 / 十字准星 / FPS,并可通过 UART 输出坐标(供 STM32 接收)

传感器配置说明:
  - GRAYSCALE 灰度: Haar Cascade 基于积分图,只支持灰度
  - HQVGA (240x160): 官方推荐分辨率,帧率与精度的平衡点;VGA 更精但慢

调参建议(见文末常数):
  - DETECT_THRESHOLD: 越高越严(少误报),越低越松(多检出)
  - SCALE_FACTOR:     越小能检更小人脸但更慢
  - SMOOTH_ALPHA:     平滑系数,越大跟得越快但越抖
  - MEMORY_FRAMES:    丢失后保留多少帧的目标位置(惯性跟踪)
"""

import sensor
import time
import image
import machine

# ---------------- 传感器 ----------------
sensor.reset()
sensor.set_contrast(3)
sensor.set_gainceiling(16)
sensor.set_framesize(sensor.HQVGA)        # 240x160
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False, gain_db=6)
sensor.set_auto_exposure(False, exposure_us=8000)

# ---------------- Haar Cascade ----------------
face_cascade = image.HaarCascade("/rom/haarcascade_frontalface.cascade", stages=25)

# ---------------- 调参常数 ----------------
DETECT_THRESHOLD = 0.75     # 检测阈值 0~1
SCALE_FACTOR     = 1.25     # 多尺度缩放步长
MIN_FACE_W       = 16       # 过滤过小误报
SMOOTH_ALPHA     = 0.4      # EMA 平滑 0~1
MEMORY_FRAMES    = 8        # 丢失后记忆帧数
PAN_DEADBAND     = 5        # 死区像素(中心判定)

# ---------------- 状态 ----------------
track_box = None            # (x, y, w, h) 平滑后的框
miss_cnt  = 0               # 连续丢失帧数
led_red   = machine.LED("LED_RED")
led_green = machine.LED("LED_GREEN")

clock = time.clock()


def pick_largest(objects):
    """从检测结果里挑面积最大的人脸,过滤过小框。"""
    best = None
    best_area = 0
    for r in objects:
        x, y, w, h = r
        if w < MIN_FACE_W or h < MIN_FACE_W:
            continue
        area = w * h
        if area > best_area:
            best_area = area
            best = r
    return best


def ema(old, new, a=SMOOTH_ALPHA):
    """指数移动平均,old 为 None 时直接用 new。"""
    if old is None:
        return new
    return tuple(int(o * (1 - a) + n * a) for o, n in zip(old, new))


while True:
    clock.tick()
    img = sensor.snapshot()

    objects = img.find_features(
        face_cascade,
        threshold=DETECT_THRESHOLD,
        scale_factor=SCALE_FACTOR,
    )

    det = pick_largest(objects)

    if det is not None:
        # 命中:平滑并画框
        track_box = ema(track_box, det)
        miss_cnt = 0
        led_green.on(); led_red.off()
    else:
        # 丢失:短时记忆,保持上一次框;超过阈值则清空
        miss_cnt += 1
        if miss_cnt > MEMORY_FRAMES:
            track_box = None
            led_green.off(); led_red.on()

    # ---------------- 绘制 ----------------
    if track_box is not None:
        x, y, w, h = track_box
        img.draw_rectangle(x, y, w, h, thickness=2)
        cx = x + w // 2
        cy = y + h // 2
        img.draw_cross(cx, cy, size=14, thickness=1)
        # 与画面中心的偏差(供云台 PID 用)
        err_x = cx - img.width() // 2
        err_y = cy - img.height() // 2
        img.draw_string(2, 2, "dx=%d dy=%d" % (err_x, err_y), color=255)
    else:
        err_x = err_y = 0
        img.draw_string(2, 2, "No face", color=255)

    img.draw_string(2, img.height() - 12, "FPS:%.1f n=%d" % (clock.fps(), len(objects)),
                    color=255)
