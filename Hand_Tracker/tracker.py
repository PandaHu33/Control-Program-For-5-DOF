import cv2
import mediapipe as mp
import numpy as np
import time


def find_available_camera(max_index: int = 5) -> int:
    """
    从索引 0 开始依次尝试打开摄像头，返回第一个可用的索引。
    max_index: 最多尝试到该索引（不含）。
    找不到时抛出 RuntimeError。
    """
    for i in range(max_index):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            cap.release()
            print(f"[Tracker] 自动检测到可用摄像头，索引={i}")
            return i
        cap.release()
    raise RuntimeError("未找到任何可用摄像头（索引 0~{} 均不可用）".format(max_index - 1))


class DepthCameraTracker:

    def __init__(self, camera_index=0, min_detection_confidence=0.7, min_tracking_confidence=0.7,
                 show_timing=True):
        """
        camera_index: 摄像头索引。
          - 整数（如 0、1）：直接使用该索引。
          - None：自动检测第一个可用摄像头。
        show_timing: 是否打印各阶段启动耗时（对应 SHOW_DEBUG_TEXT）。
        """
        t_total = time.perf_counter()

        self.mp_hands = mp.solutions.hands
        self.mp_drawing = mp.solutions.drawing_utils

        # ── 阶段1：打开摄像头硬件 ──────────────────────────────────────
        if camera_index is None:
            camera_index = find_available_camera()

        print(f"[Tracker] 正在打开摄像头，索引={camera_index}")
        t0 = time.perf_counter()
        # 用 CAP_DSHOW 后端直接在构造时传入分辨率和帧率，
        # 避免后续 cap.set() 每次触发驱动重协商（实测每次约 6 秒）
        self.cap = cv2.VideoCapture(camera_index, cv2.CAP_DSHOW)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        t_open = (time.perf_counter() - t0) * 1000

        if not self.cap.isOpened():
            raise RuntimeError(f"Cannot open camera (index={camera_index})")

        # ── 阶段2：属性设置（已在构造时完成，此处仅占位计时）──
        t_props = 0.0

        # ── 阶段3：读取第一帧（硬件真正就绪的时刻）──────────────────────
        t0 = time.perf_counter()
        for _ in range(5):  # 最多重试 5 次，兼容启动慢的摄像头
            ret, _ = self.cap.read()
            if ret:
                break
        t_first_frame = (time.perf_counter() - t0) * 1000
        t_from_open_to_first = (time.perf_counter() - t_total) * 1000

        if not ret:
            raise RuntimeError(f"Camera opened but failed to read first frame (index={camera_index})")

        # 第一帧成功后再设置缓冲区大小，避免部分驱动的重协商延迟
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)

        # ── 阶段4：MediaPipe 模型加载 ──────────────────────────────────
        # 放在摄像头就绪之后：模型加载期间摄像头已在后台预热，
        # 加载完成后可立即开始推理，不再有额外的硬件等待
        t0 = time.perf_counter()
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            model_complexity=0,
            min_detection_confidence=min_detection_confidence,
            min_tracking_confidence=min_tracking_confidence
        )
        t_mediapipe = (time.perf_counter() - t0) * 1000

        t_total_ms = (time.perf_counter() - t_total) * 1000

        # ── 启动耗时报告 ───────────────────────────────────────────────
        print(f"[Tracker] 摄像头已就绪：{actual_w}×{actual_h} @ {actual_fps:.0f}fps")
        if show_timing:
            print(f"[Tracker] 启动耗时分解:")
            print(f"  1. VideoCapture 打开+属性设置  {t_open:6.0f} ms")
            print(f"  2. 第一帧读取（从打开起计）     {t_from_open_to_first:6.0f} ms  (read本身 {t_first_frame:.0f}ms)")
            print(f"  3. MediaPipe 模型加载          {t_mediapipe:6.0f} ms")
            print(f"  ─────────────────────────────────────────")
            print(f"     总初始化耗时                {t_total_ms:6.0f} ms")

    def get_frames_and_process_hands(self):

        ret, frame = self.cap.read()
        if not ret:
            return None, None, None, None

        # 镜像翻转：在送入 MediaPipe 之前完成
        frame = cv2.flip(frame, 1)

        # 缩小到 320×240 再送入 MediaPipe，推理速度约快 4x（面积缩小 1/4）
        # 显示仍用原始 640×480 的 frame，只有推理用小图
        small = cv2.resize(frame, (320, 240), interpolation=cv2.INTER_LINEAR)
        image_rgb = cv2.cvtColor(small, cv2.COLOR_BGR2RGB)

        image_rgb.flags.writeable = False
        hand_results = self.hands.process(image_rgb)
        image_rgb.flags.writeable = True

        return hand_results, None, None, frame

    def draw_landmarks(self, frame, hand_landmarks):

        self.mp_drawing.draw_landmarks(
            frame,
            hand_landmarks,
            self.mp_hands.HAND_CONNECTIONS
        )

    def close(self):

        print("Closing webcam and MediaPipe resources.")

        self.cap.release()
        self.hands.close()