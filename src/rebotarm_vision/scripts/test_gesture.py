#!/usr/bin/env python3
"""
手势识别独立测试脚本（不依赖 ROS）

实时显示摄像头画面 + 手部骨架 + 每根手指的弯曲比值 + 判定结果，
用于在封装 ROS 节点之前调节 gesture_classifier.py 中的阈值。

用法：
  python3 src/rebotarm_vision/scripts/test_gesture.py [--camera 0]
按键：
  q / ESC  退出
  r        打印当前各手指比值（标定阈值用）
"""

import argparse
import sys
import time
from collections import deque
from pathlib import Path

import cv2
import mediapipe as mp

# 不依赖 colcon 安装，直接从源码树导入分类器
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from rebotarm_vision.gesture_classifier import (  # noqa: E402
    DEFAULT_PARAMS, classify, finger_ratios, heart_debug,
)

DEBOUNCE = 5   # 连续 5 帧一致才切换输出（与 ROS 节点一致）


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", type=int, default=0)
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    args = ap.parse_args()

    cap = cv2.VideoCapture(args.camera)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    if not cap.isOpened():
        print(f"无法打开摄像头 /dev/video{args.camera}")
        sys.exit(1)

    hands_det = mp.solutions.hands.Hands(
        static_image_mode=False, max_num_hands=2,
        min_detection_confidence=0.7, min_tracking_confidence=0.5)
    draw = mp.solutions.drawing_utils

    history = deque(maxlen=DEBOUNCE)
    stable = "none"
    fps_t = time.time()
    fps = 0.0

    print("阈值参数:", DEFAULT_PARAMS)
    print("开始识别，按 q 退出，按 r 打印当前手指比值")

    while True:
        ok, frame = cap.read()
        if not ok:
            print("读取摄像头失败")
            break
        frame = cv2.flip(frame, 1)   # 镜像，符合自拍直觉
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        res = hands_det.process(rgb)

        h, w = frame.shape[:2]
        hand_lms = []
        if res.multi_hand_landmarks:
            for hl in res.multi_hand_landmarks:
                draw.draw_landmarks(frame, hl, mp.solutions.hands.HAND_CONNECTIONS)
                hand_lms.append([(lm.x * w, lm.y * h) for lm in hl.landmark])

        raw = classify(hand_lms)
        history.append(raw)
        if len(history) == DEBOUNCE and len(set(history)) == 1 and raw != stable:
            stable = raw
            print(f"[{time.strftime('%H:%M:%S')}] 稳定输出 -> {stable}")

        now = time.time()
        fps = 0.9 * fps + 0.1 / max(now - fps_t, 1e-6)
        fps_t = now

        color = {"rock": (0, 0, 255), "paper": (0, 255, 0),
                 "heart": (255, 0, 255)}.get(stable, (128, 128, 128))
        cv2.putText(frame, f"raw: {raw}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(frame, f"stable: {stable}", (10, 65),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 3)
        cv2.putText(frame, f"fps: {fps:.0f}", (10, h - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)

        # 双手入镜：实时显示比心逐项诊断（哪个条件不满足一眼可见）
        if len(hand_lms) >= 2:
            for i, (name, val, thr, ok) in enumerate(
                    heart_debug(hand_lms[0], hand_lms[1])):
                txt = f"{name}: {val:.2f} (thr {thr:.2f})"
                cv2.putText(frame, txt, (10, 100 + i * 22),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (0, 255, 0) if ok else (0, 0, 255), 1)
        cv2.imshow("gesture test", frame)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            break
        if key == ord("r") and hand_lms:
            for i, lm in enumerate(hand_lms):
                r = finger_ratios(lm)
                print(f"  hand{i}: " + "  ".join(
                    f"{k}={v:.2f}" for k, v in r.items()))
            if len(hand_lms) >= 2:
                print("  比心诊断:")
                for name, val, thr, ok in heart_debug(hand_lms[0], hand_lms[1]):
                    print(f"    {'PASS' if ok else 'FAIL'} {name}: "
                          f"{val:.2f} (thr {thr:.2f})")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
