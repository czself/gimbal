"""
OpenMV H7 Plus - 人脸检测
========================

硬件: OpenMV H7 Plus (STM32H743VI + OV5640)
方式: Haar Cascade 正面人脸,选最大框 + EMA 平滑
输出: 画框 / 十字准星 / FPS / 中心偏差
"""

import sensor
import time
import image
import machine

# ---------------- 传感器 (和 main1.py 一样) ----------------
sensor.reset()
sensor.set_contrast(1)
sensor.set_brightness(1)
sensor.set_gainceiling(16)
sensor.set_framesize(sensor.HQVGA)        # 240x160
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False, gain_db=32)            # 和以前一样
sensor.set_auto_exposure(False, exposure_us=30000)  # 和以前一样

# ---------------- Haar Cascade (放宽参数) ----------------
face_cascade = image.HaarCascade("/rom/haarcascade_frontalface.cascade", stages=20)

# ---------------- 调参常数 ----------------
DETECT_THRESHOLD = 0.6      # 从0.75降到0.6,更容易检出
SCALE_FACTOR     = 1.2      # 从1.25降到1.2,更多尺度搜索
MIN_FACE_W       = 10       # 过滤过小误报
SMOOTH_ALPHA     = 0.4      # EMA 平滑

# ---------------- 状态 ----------------
track_box = None
miss_cnt  = 0
led_red   = machine.LED("LED_RED")
led_green = machine.LED("LED_GREEN")

clock = time.clock()


def pick_largest(objects):
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
        track_box = ema(track_box, det)
        miss_cnt = 0
        led_green.on(); led_red.off()
    else:
        miss_cnt += 1
        if miss_cnt > 20:          # 从8→20,断触后保持2秒不丢
            track_box = None
            led_green.off(); led_red.on()

    if track_box is not None:
        x, y, w, h = track_box
        img.draw_rectangle(x, y, w, h, thickness=2)
        cx = x + w // 2
        cy = y + h // 2
        img.draw_cross(cx, cy, size=14, thickness=1)
        err_x = cx - img.width() // 2
        err_y = cy - img.height() // 2
        img.draw_string(2, 2, "dx=%d dy=%d" % (err_x, err_y), color=255)
    else:
        err_x = err_y = 0
        img.draw_string(2, 2, "No face", color=255)

    img.draw_string(2, img.height() - 12, "FPS:%.1f n=%d" % (clock.fps(), len(objects)),
                    color=255)