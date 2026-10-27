#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
vision_tracker.py — 双目 USB 摄像头 + STM32 云台视觉跟踪上位机

数据流：
    摄像头 --USB--> 本脚本(OpenCV 检测目标) --串口 921600--> STM32 --I2C--> 舵机

两种模式：
    --mode color   用颜色阈值找目标（默认绿色），可拖动滑条实时调 HSV
    --mode sim     不接摄像头，发送一个虚拟移动目标（用来验证串口链路 + PID）

用法（在你的 vision_servo 环境里）：
    python vision_tracker.py --port COM7 --cam 0 --mode color
    python vision_tracker.py --port COM7 --mode sim
    python vision_tracker.py --list            # 列出可用摄像头序号

快捷键：q 退出，s 保存当前帧，r 重置滑条
"""

import argparse
import math
import struct
import sys
import time

import serial

# Windows 控制台默认 GBK，中文输出会乱码
if hasattr(sys.stdout, "reconfigure"):
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:
        pass

# ---------------------------------------------------------------- 协议
FRAME_HEAD = b"\xAA\x55"
CMD_TARGET = 0x10
CMD_PING = 0x03


def crc16_ccitt(data: bytes, crc: int = 0xFFFF) -> int:
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
    return crc


def build_frame(cmd: int, payload: bytes) -> bytes:
    body = bytes([len(payload) + 1, cmd]) + payload
    c = crc16_ccitt(body)
    return FRAME_HEAD + body + bytes([c & 0xFF, (c >> 8) & 0xFF])


def build_target(dx: int, dy: int, valid: bool) -> bytes:
    return build_frame(CMD_TARGET, struct.pack("<hhB", int(dx), int(dy), 1 if valid else 0))


# ---------------------------------------------------------------- 摄像头枚举
def list_cameras(max_idx: int = 6) -> None:
    try:
        import cv2
    except ImportError:
        print("需要 opencv：pip install opencv-python")
        return
    print("探测摄像头序号（每个会尝试打开并读一帧）...")
    for i in range(max_idx):
        cap = cv2.VideoCapture(i, cv2.CAP_DSHOW)
        if not cap.isOpened():
            cap.release()
            continue
        ok, frame = cap.read()
        cap.release()
        if ok and frame is not None:
            h, w = frame.shape[:2]
            print(f"  序号 {i}: {w}x{h}  可用")
        else:
            print(f"  序号 {i}: 能打开但读不到画面")


# ---------------------------------------------------------------- 颜色检测
class ColorTracker:
    def __init__(self, h_min=35, h_max=85, s_min=80, v_min=60, s_max=255, v_max=255):
        self.h_min, self.h_max = h_min, h_max
        self.s_min, self.v_min = s_min, v_min
        self.s_max, self.v_max = s_max, v_max

    def make_trackbars(self, win: str) -> None:
        import cv2
        cv2.createTrackbar("Hmin", win, self.h_min, 179, lambda v: None)
        cv2.createTrackbar("Hmax", win, self.h_max, 179, lambda v: None)
        cv2.createTrackbar("Smin", win, self.s_min, 255, lambda v: None)
        cv2.createTrackbar("Vmin", win, self.v_min, 255, lambda v: None)

    def read_trackbars(self, win: str) -> None:
        import cv2
        self.h_min = cv2.getTrackbarPos("Hmin", win)
        self.h_max = cv2.getTrackbarPos("Hmax", win)
        self.s_min = cv2.getTrackbarPos("Smin", win)
        self.v_min = cv2.getTrackbarPos("Vmin", win)

    def detect(self, frame):
        """返回目标质心 (cx, cy) 或 None"""
        import cv2
        import numpy as np

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(
            hsv,
            np.array([self.h_min, self.s_min, self.v_min], np.uint8),
            np.array([self.h_max, self.s_max, self.v_max], np.uint8),
        )
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None, mask

        c = max(contours, key=cv2.contourArea)
        if cv2.contourArea(c) < 300:          # 太小的噪点丢弃
            return None, mask

        (x, y), radius = cv2.minEnclosingCircle(c)
        if radius < 8:
            return None, mask
        return (int(x), int(y)), mask


# ---------------------------------------------------------------- 主流程
def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM7", help="STM32 串口，如 COM7")
    ap.add_argument("--baud", default=921600, type=int)
    ap.add_argument("--cam", default=0, type=int, help="摄像头序号（双目可能有 0/1 两个）")
    ap.add_argument("--mode", default="color", choices=["color", "sim"])
    ap.add_argument("--width", default=640, type=int)
    ap.add_argument("--height", default=480, type=int)
    ap.add_argument("--fps", default=30, type=int, help="发送频率")
    ap.add_argument("--duration", default=0, type=float, help="运行秒数，0=一直跑")
    ap.add_argument("--list", action="store_true", help="只列出摄像头")
    args = ap.parse_args()

    if args.list:
        list_cameras()
        return 0

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.2)
    except Exception as e:
        print(f"[错误] 打不开串口 {args.port}: {e}")
        print("      检查：是否被串口助手占用？端口号对不对？")
        return 1

    print(f"串口 {args.port} @ {args.baud} 已打开")

    cap = None
    tracker = None
    win = "vision_tracker"

    if args.mode == "color":
        import cv2
        cap = cv2.VideoCapture(args.cam, cv2.CAP_DSHOW)
        if not cap.isOpened():
            print(f"[错误] 打不开摄像头 {args.cam}，用 --list 看看有哪些序号")
            return 1
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
        w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print(f"摄像头 {args.cam} 分辨率 {w}x{h}")
        tracker = ColorTracker()
        cv2.namedWindow(win)
        tracker.make_trackbars(win)
        cx_ref, cy_ref = w // 2, h // 2
    else:
        w = h = None
        cx_ref = cy_ref = 0.0

    period = 1.0 / max(1, args.fps)
    next_t = time.perf_counter()
    t0 = time.perf_counter()
    sent = 0
    lost = 0
    last_report = t0

    try:
        while True:
            now = time.perf_counter()
            if args.duration and (now - t0) >= args.duration:
                break

            if args.mode == "color":
                import cv2
                ok, frame = cap.read()
                if not ok:
                    continue

                tracker.read_trackbars(win)
                target, mask = tracker.detect(frame)

                if target is None:
                    lost += 1
                    ser.write(build_target(0, 0, False))
                    cv2.putText(frame, "TARGET LOST", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                else:
                    cx, cy = target
                    dx, dy = cx - cx_ref, cy - cy_ref
                    ser.write(build_target(dx, dy, True))
                    sent += 1

                    cv2.circle(frame, (cx, cy), 10, (0, 255, 0), 2)
                    cv2.line(frame, (cx_ref, cy_ref), (cx, cy), (0, 255, 255), 1)
                    cv2.putText(frame, f"dx={dx:+4d} dy={dy:+4d}", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                cv2.line(frame, (cx_ref - 15, cy_ref), (cx_ref + 15, cy_ref), (255, 255, 255), 1)
                cv2.line(frame, (cx_ref, cy_ref - 15), (cx_ref, cy_ref + 15), (255, 255, 255), 1)
                cv2.imshow(win, frame)
                cv2.imshow("mask", mask)

                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
                if key == ord("s"):
                    cv2.imwrite("snapshot.png", frame)
                    print("已保存 snapshot.png")
            else:
                # 虚拟目标：在画面里画一个椭圆轨迹
                tt = now - t0
                dx = 200.0 * math.sin(2 * math.pi * 0.20 * tt)
                dy = 120.0 * math.sin(2 * math.pi * 0.13 * tt)
                ser.write(build_target(int(dx), int(dy), True))
                sent += 1

            # 每秒打印一次统计
            if now - last_report >= 1.0:
                last_report = now
                print(f"  发送 {sent:5d} 帧, 丢失 {lost:4d} 帧, "
                      f"目标丢失率 {100.0 * lost / max(1, sent + lost):5.1f}%")

            next_t += period
            sleep = next_t - time.perf_counter()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_t = time.perf_counter()

    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        ser.close()
        if cap is not None:
            cap.release()
            import cv2
            cv2.destroyAllWindows()
        print(f"结束：共发送 {sent} 帧有效目标，{lost} 帧丢失")

    return 0


if __name__ == "__main__":
    sys.exit(main())
