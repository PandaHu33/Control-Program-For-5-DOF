# robot_control/utils.py
import cv2
import numpy as np
import math
import mediapipe as mp


def get_hand_openness_distance(lm):
    # 通过计算大拇指指尖到小拇指指尖的欧式距离来估算手掌的“张开度”
    p1 = lm.landmark[mp.solutions.hands.HandLandmark.THUMB_TIP]
    p2 = lm.landmark[mp.solutions.hands.HandLandmark.PINKY_TIP]
    return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)

def get_pinch_distance(lm):
    """
    计算拇指指尖和食指指尖的距离（捏合手势）。
    使用相对于手指长度的归一化距离，避免手旋转和前后移动的影响。

    返回: 归一化距离 (0~1 范围内的浮点数)
    """
    thumb_tip = lm.landmark[mp.solutions.hands.HandLandmark.THUMB_TIP]
    index_tip = lm.landmark[mp.solutions.hands.HandLandmark.INDEX_FINGER_TIP]
    thumb_mcp = lm.landmark[mp.solutions.hands.HandLandmark.THUMB_CMC]
    index_mcp = lm.landmark[mp.solutions.hands.HandLandmark.INDEX_FINGER_MCP]

    # 计算拇指-食指指尖的3D距离
    pinch_dist = math.sqrt((thumb_tip.x - index_tip.x)**2 +
                           (thumb_tip.y - index_tip.y)**2 +
                           (thumb_tip.z - index_tip.z)**2)

    # 计算拇指长度（拇指根部到指尖）
    thumb_length = math.sqrt((thumb_tip.x - thumb_mcp.x)**2 +
                             (thumb_tip.y - thumb_mcp.y)**2 +
                             (thumb_tip.z - thumb_mcp.z)**2)

    # 计算食指长度（食指根部到指尖）
    index_length = math.sqrt((index_tip.x - index_mcp.x)**2 +
                             (index_tip.y - index_mcp.y)**2 +
                             (index_tip.z - index_mcp.z)**2)

    # 使用两个手指的平均长度作为归一化参考
    avg_finger_length = (thumb_length + index_length) / 2.0

    # 归一化：距离除以平均手指长度
    if avg_finger_length > 0.01:  # 避免除零
        normalized_dist = pinch_dist / avg_finger_length
    else:
        normalized_dist = pinch_dist

    return normalized_dist

def apply_servo_mapping(angles_deg, k_vals, b_vals):
    # 机械臂逆运动学计算出来的角度 x 是理想值，但实际电机往往存在偏差，要进行物理校准
    if len(angles_deg) != len(k_vals) or len(angles_deg) != len(b_vals):
        print("Warning: Angle/K/B list lengths differ. Mapping skipped.")
        return angles_deg
    return [(k * x + b) for k, x, b in zip(k_vals, angles_deg, b_vals)]

def draw_info_on_frame(frame, angles, ik_msg, ik_ok, target, paused, show_text=True):
    if show_text:
        y, font = 30, cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(frame, f"IK Status: {ik_msg}", (10,y), font, 0.7, (0,255,0) if ik_ok else (0,0,255), 2)
        y+=40; cv2.putText(frame, f"Target XYZ: [{target[0]:.2f},{target[1]:.2f},{target[2]:.2f}] m",(10,y),font,0.7,(255,255,0),2)
        y+=40; cv2.putText(frame, "Final Angles (J1-J6, Deg):",(10,y),font,0.7,(0,255,0),2)
        y+=10
        for i, angle in enumerate(angles):
            cv2.putText(frame,f"Joint {i+1}: {angle}",(15,y+(i+1)*30),font,0.6,(0,255,0),2)
        cv2.putText(frame,"P:Pause | R:Reset | Q:Quit",(10,frame.shape[0]-10),font,0.6,(255,255,0),2)
    if paused:
        font = cv2.FONT_HERSHEY_SIMPLEX
        pause_text = "[PAUSED]"
        sz = cv2.getTextSize(pause_text, font, 1.5, 3)[0]
        cv2.putText(frame,pause_text,(frame.shape[1]-sz[0]-15,frame.shape[0]-15),font,1.5,(0,165,255),3)
    return frame


# 滑动窗口：跨帧保存最近 N 帧的骨骼段中位数，用于时间平滑
# 放在模块级别，函数每次调用时自动更新
_depth_window: list[float] = []
_DEPTH_WINDOW_SIZE = 8  # 保留最近 8 帧，约 130ms@60fps / 270ms@30fps


def estimate_depth_from_hand(lm):
    """
    基于多骨骼段中位数 + 滑动窗口的深度估计，对手掌旋转和横向位置鲁棒。

    修复：改用 3D 欧氏距离（x, y, z 三分量）代替原来的 2D 投影距离。
    原 2D 方案在手移到画面边缘时，透视投影会压缩骨骼段的像素长度，
    导致深度被低估（Z 值下降）。MediaPipe 的 z 分量是相对手腕的归一化
    深度，与手在画面中的横向位置无关，用 3D 距离可消除该耦合。
    """
    global _depth_window

    lm = lm.landmark  # 展开为列表，方便按索引访问

    def d(i, j):
        # 三维欧氏距离：x/y 为归一化图像坐标，z 为 MediaPipe 相对深度
        # 三者量纲不完全一致，但对同一只手的骨骼段比较是自洽的
        return math.sqrt(
            (lm[i].x - lm[j].x) ** 2 +
            (lm[i].y - lm[j].y) ** 2 +
            (lm[i].z - lm[j].z) ** 2
        )

    # 15 段固定骨骼段，覆盖手掌和全部手指，方向分散
    segs = [
        d(0, 5),   # wrist → 食指MCP
        d(0, 9),   # wrist → 中指MCP  ← 手掌中轴，最稳定
        d(0, 13),  # wrist → 无名指MCP
        d(0, 17),  # wrist → 小指MCP
        d(5, 6),   # 食指 MCP→PIP
        d(9, 10),  # 中指 MCP→PIP
        d(13, 14), # 无名指 MCP→PIP
        d(17, 18), # 小指 MCP→PIP
        d(6, 7),   # 食指 PIP→DIP
        d(10, 11), # 中指 PIP→DIP
        d(14, 15), # 无名指 PIP→DIP
        d(18, 19), # 小指 PIP→DIP
        d(1, 2),   # 拇指 CMC→MCP
        d(2, 3),   # 拇指 MCP→IP
        d(3, 4),   # 拇指 IP→TIP
    ]

    # 中位数：自动选中受旋转压缩最少的那一半骨骼段
    frame_median = float(np.median(segs))

    # 滑动窗口时间平滑
    _depth_window.append(frame_median)
    if len(_depth_window) > _DEPTH_WINDOW_SIZE:
        _depth_window.pop(0)
    smoothed_size = float(np.mean(_depth_window))

    smoothed_size = max(smoothed_size, 0.005)

    # 映射到机械臂 Z 轴
    # smoothed_size 典型值（归一化坐标，各手指骨骼段中位数）：
    #   手靠近摄像头 (~30cm): ≈ 0.07~0.10
    #   手正常距离  (~50cm): ≈ 0.04~0.07
    #   手远离摄像头 (~80cm): ≈ 0.02~0.04
    PALM_SIZE_NEAR = 0.085
    PALM_SIZE_FAR  = 0.030

    Z_OUT_MIN = 0.05
    Z_OUT_MAX = 1.0

    return float(np.clip(
        np.interp(smoothed_size, [PALM_SIZE_FAR, PALM_SIZE_NEAR], [Z_OUT_MIN, Z_OUT_MAX]),
        Z_OUT_MIN, Z_OUT_MAX
    ))


def landmark_distance(lm, idx_a, idx_b):
    """Return normalized 3D distance between two MediaPipe hand landmarks."""
    points = lm.landmark
    a = points[int(idx_a)]
    b = points[int(idx_b)]
    return math.sqrt(
        (a.x - b.x) ** 2 +
        (a.y - b.y) ** 2 +
        (a.z - b.z) ** 2
    )


def is_finger_extended(lm, tip_idx, pip_idx, mcp_idx, wrist_idx=0):
    """Heuristic finger extension test that is less sensitive to hand rotation."""
    tip_to_wrist = landmark_distance(lm, wrist_idx, tip_idx)
    pip_to_wrist = landmark_distance(lm, wrist_idx, pip_idx)
    mcp_to_wrist = landmark_distance(lm, wrist_idx, mcp_idx)
    return tip_to_wrist > pip_to_wrist * 1.12 and tip_to_wrist > mcp_to_wrist * 1.35


def get_extended_fingers(lm):
    """Return extension state for index/middle/ring/pinky fingers."""
    hand = mp.solutions.hands.HandLandmark
    return {
        "index": is_finger_extended(lm, hand.INDEX_FINGER_TIP, hand.INDEX_FINGER_PIP, hand.INDEX_FINGER_MCP),
        "middle": is_finger_extended(lm, hand.MIDDLE_FINGER_TIP, hand.MIDDLE_FINGER_PIP, hand.MIDDLE_FINGER_MCP),
        "ring": is_finger_extended(lm, hand.RING_FINGER_TIP, hand.RING_FINGER_PIP, hand.RING_FINGER_MCP),
        "pinky": is_finger_extended(lm, hand.PINKY_TIP, hand.PINKY_PIP, hand.PINKY_MCP),
    }


def classify_hand_gesture(lm, pinch_threshold=0.45):
    """Classify simple control gestures used by the arm control interface."""
    fingers = get_extended_fingers(lm)
    extended_count = sum(1 for value in fingers.values() if value)
    pinch = get_pinch_distance(lm) <= pinch_threshold

    if fingers["index"] and fingers["middle"] and not fingers["ring"] and not fingers["pinky"]:
        return "victory", fingers
    if extended_count >= 4:
        return "open_palm", fingers
    if extended_count == 0:
        return "fist", fingers
    if pinch:
        return "pinch", fingers
    return "unknown", fingers


def get_wrist_control_point(lm):
    """Return normalized wrist x/y plus estimated hand depth."""
    wrist = lm.landmark[mp.solutions.hands.HandLandmark.WRIST]
    return np.array([float(wrist.x), float(wrist.y), estimate_depth_from_hand(lm)], dtype=float)
