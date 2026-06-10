# main.py
import cv2
import numpy as np
import time
import threading
import collections

from robot_control.config import Config
from robot_control.tracker import DepthCameraTracker
from robot_control.kinematics import RobotKinematics
import robot_control.utils as utils

try:
    from simulation.pybullet_sim import PyBulletSim
    _PYBULLET_AVAILABLE = True
except ImportError:
    _PYBULLET_AVAILABLE = False
    print("[警告] pybullet 未安装，PyBullet 仿真已禁用。可运行: pip install pybullet")


class PerfStats:
    """滑动窗口统计各环节耗时，窗口大小 = 60 帧。"""
    _WINDOW = 60

    def __init__(self, keys):
        self._data = {k: collections.deque(maxlen=self._WINDOW) for k in keys}

    def record(self, key, dt_ms):
        self._data[key].append(dt_ms)

    def report(self):
        lines = ["[Perf] 各环节耗时（最近60帧）:"]
        for k, dq in self._data.items():
            if not dq:
                continue
            lines.append(f"  {k:<20s}  avg={np.mean(dq):5.1f}ms  max={np.max(dq):5.1f}ms")
        print("\n".join(lines))


class ComputeWorker:
    """后台线程：摄像头采集 + 手势追踪 + IK 计算"""

    def __init__(self, cfg, robot_kinematics, shutdown_event):
        self.cfg = cfg
        self.robot_kinematics = robot_kinematics
        self.shutdown_event = shutdown_event
        self.tracker = None

        # 共享输出（主线程读取）
        self.lock = threading.Lock()
        self.frame = None
        self.current_angles = None
        self.smoothed_target = None
        self.link_nodes = None       # 预算好的关节节点坐标，供 update_plot 直接使用
        self.final_angles_deg = []
        self.ik_msg = "Initializing"
        self.ik_ok = True
        self.paused = False

        # 主线程写入的控制指令
        self.cmd_pause = False
        self.cmd_reset = False

    def run(self):
        try:
            self.tracker = DepthCameraTracker(camera_index=self.cfg.CAMERA_INDEX,
                                               show_timing=self.cfg.SHOW_DEBUG_TEXT)
        except Exception as e:
            print(f"FATAL: Could not initialize DepthCameraTracker. {e}")
            self.shutdown_event.set()
            return

        home_pos_rad = [np.deg2rad(a) for a in self.cfg.HOME_POSITION_DEG]
        current_angles = [0.0] * len(self.robot_kinematics.my_chain.links)
        current_angles[1:5] = home_pos_rad
        current_angles[5] = np.deg2rad(self.cfg.J11_ANGLE_MAX_DEG)
        current_angles[6] = -current_angles[5]
        smoothed_target = self.robot_kinematics.my_chain.forward_kinematics(current_angles)[:3, 3]
        smoothed_j11_rad = current_angles[5]

        ik_msg = "Initializing"
        ik_ok = True
        paused = False
        final_angles_deg = list(self.cfg.HOME_POSITION_DEG) + [0, 0]

        perf = PerfStats(["cap.read+flip", "mediapipe", "features", "IK", "total"])
        _perf_frame_count = 0
        _PERF_REPORT_INTERVAL = 60  # 每 60 帧打印一次耗时报告

        try:
            while not self.shutdown_event.is_set():
                t_frame_start = time.perf_counter()

                # ── 控制指令 ──
                if self.cmd_reset:
                    self.cmd_reset = False
                    current_angles[1:5] = home_pos_rad
                    current_angles[5] = np.deg2rad(self.cfg.J11_ANGLE_MAX_DEG)
                    current_angles[6] = -current_angles[5]
                    smoothed_target = self.robot_kinematics.my_chain.forward_kinematics(current_angles)[:3, 3]
                    smoothed_j11_rad = current_angles[5]
                    print("--- Position Reset ---")
                paused = self.cmd_pause

                # ── 1. 摄像头读取（含 flip + BGR2RGB，在 tracker 内完成）──
                t0 = time.perf_counter()
                hand_results, _, _, frame = self.tracker.get_frames_and_process_hands()
                t_cap = (time.perf_counter() - t0) * 1000  # tracker 内含 mediapipe，拆分见下

                if frame is None:
                    time.sleep(0.01)
                    continue

                # tracker.get_frames_and_process_hands 把 cap.read+flip 和 mediapipe 合并了，
                # 此处将总耗时记录到 mediapipe 桶（已是最大瓶颈，无需再拆）
                perf.record("cap.read+flip", 0)       # 占位，实际在 mediapipe 桶
                perf.record("mediapipe", t_cap)

                # ── 2. 特征提取 + 目标点计算 ──
                t0 = time.perf_counter()
                if hand_results and hand_results.multi_hand_landmarks:
                    lm = hand_results.multi_hand_landmarks[0]
                    self.tracker.draw_landmarks(frame, lm)

                    wrist = lm.landmark[self.tracker.mp_hands.HandLandmark.WRIST]
                    h, w, _ = frame.shape
                    wrist_px = int(wrist.x * w)
                    wrist_py = int(wrist.y * h)

                    x = np.interp(wrist.x, [0, 1], [self.cfg.ROBOT_X_MIN, self.cfg.ROBOT_X_MAX])
                    y = np.interp(wrist.y, [0, 1], [self.cfg.ROBOT_Y_MIN, self.cfg.ROBOT_Y_MAX])
                    z = utils.estimate_depth_from_hand(lm)

                    raw_target = np.array([x, y, z])
                    xy_a = self.cfg.SMOOTHING_FACTOR
                    z_a  = self.cfg.Z_SMOOTHING_FACTOR
                    smoothed_target = np.array([
                        (1 - xy_a) * smoothed_target[0] + xy_a * raw_target[0],
                        (1 - xy_a) * smoothed_target[1] + xy_a * raw_target[1],
                        (1 - z_a)  * smoothed_target[2] + z_a  * raw_target[2],
                    ])

                    if self.cfg.SHOW_DEBUG_TEXT:
                        cv2.putText(frame, f"Robot: [{x:.2f}, {y:.2f}, {z:.2f}]",
                                    (wrist_px + 10, wrist_py),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

                    # ── 夹爪控制 ──
                    if self.cfg.USE_PINCH_GESTURE:
                        pinch_dist = utils.get_pinch_distance(lm)
                        j11_deg = np.interp(pinch_dist,
                                            [self.cfg.PINCH_CLOSE_DIST, self.cfg.PINCH_OPEN_DIST],
                                            [self.cfg.J11_ANGLE_MIN_DEG, self.cfg.J11_ANGLE_MAX_DEG])
                    else:
                        open_dist = utils.get_hand_openness_distance(lm)
                        j11_deg = np.interp(open_dist,
                                            [self.cfg.HAND_OPEN_MIN_DIST, self.cfg.HAND_OPEN_MAX_DIST],
                                            [self.cfg.J11_ANGLE_MAX_DEG, self.cfg.J11_ANGLE_MIN_DEG])

                    j11_rad = np.deg2rad(np.clip(j11_deg, -30, 15))
                    smoothed_j11_rad = ((1 - self.cfg.J11_SMOOTHING_FACTOR) * smoothed_j11_rad
                                        + self.cfg.J11_SMOOTHING_FACTOR * j11_rad)

                perf.record("features", (time.perf_counter() - t0) * 1000)

                # ── 3. IK 求解 ──
                t0 = time.perf_counter()
                target_pos = np.clip(
                    smoothed_target,
                    [self.cfg.ROBOT_X_MIN, self.cfg.ROBOT_Y_MIN, self.cfg.ROBOT_Z_MIN],
                    [self.cfg.ROBOT_X_MAX, self.cfg.ROBOT_Y_MAX, self.cfg.ROBOT_Z_MAX],
                )
                # 限制目标帧间最大位移：防止手部跳变导致 IK 初始猜测偏差过大，
                # 迭代次数飙升（max_iter=50 时仍可能耗时 50ms+）
                MAX_STEP = 0.03  # 每帧最多移动 3cm
                delta = target_pos - smoothed_target
                dist = np.linalg.norm(delta)
                if dist > MAX_STEP:
                    target_pos = smoothed_target + delta / dist * MAX_STEP

                ik_angles, ik_ok = self.robot_kinematics.solve_ik(target_pos, current_angles)
                ik_msg = "OK" if ik_ok else "Failed"
                if ik_ok:
                    current_angles = ik_angles
                    smoothed_target = target_pos
                perf.record("IK", (time.perf_counter() - t0) * 1000)

                # ── 4. 夹爪写入 + 伺服映射 + 预算节点坐标 ──
                j21_rad = -smoothed_j11_rad
                current_angles[5] = smoothed_j11_rad
                current_angles[6] = j21_rad

                calc_deg = [np.degrees(a) for a in current_angles[1:5]]
                calc_deg += [np.degrees(smoothed_j11_rad), np.degrees(j21_rad)]
                mapped = utils.apply_servo_mapping(calc_deg, self.cfg.SERVO_K, self.cfg.SERVO_B)
                final_angles_deg = [int(round(a)) for a in mapped]

                # 在后台线程预算节点坐标，GUI 主线程的 update_plot 直接使用，
                # 避免在主线程重复调用 forward_kinematics(full_kinematics=True)
                link_nodes = self.robot_kinematics.get_link_nodes(current_angles)

                frame = utils.draw_info_on_frame(frame, final_angles_deg, ik_msg, ik_ok,
                                                 smoothed_target, paused, self.cfg.SHOW_DEBUG_TEXT)

                perf.record("total", (time.perf_counter() - t_frame_start) * 1000)

                # 每 60 帧输出一次性能报告（仅 SHOW_DEBUG_TEXT 模式）
                _perf_frame_count += 1
                if self.cfg.SHOW_DEBUG_TEXT and _perf_frame_count % _PERF_REPORT_INTERVAL == 0:
                    perf.report()

                # ── 写入共享状态 ──
                with self.lock:
                    self.frame = frame
                    self.current_angles = current_angles.copy()
                    self.smoothed_target = smoothed_target.copy()
                    self.link_nodes = link_nodes
                    self.final_angles_deg = final_angles_deg
                    self.ik_msg = ik_msg
                    self.ik_ok = ik_ok
                    self.paused = paused

        finally:
            if self.tracker:
                self.tracker.close()


def main():
    cfg = Config()
    shutdown_event = threading.Event()

    workspace = {
        'x': (cfg.ROBOT_X_MIN, cfg.ROBOT_X_MAX),
        'y': (cfg.ROBOT_Y_MIN, cfg.ROBOT_Y_MAX),
        'z': (cfg.ROBOT_Z_MIN, cfg.ROBOT_Z_MAX),
    }

    robot_kinematics = RobotKinematics(
        urdf_file_path="urdf/5f_manipulator1.urdf",
        workspace_limits=workspace,
    )

    pybullet_sim = None
    if _PYBULLET_AVAILABLE and cfg.ENABLE_PYBULLET:
        try:
            pybullet_sim = PyBulletSim(urdf_path="urdf/5f_manipulator1.urdf")
            print("[PyBullet] 仿真窗口已启动。")
        except Exception as e:
            print(f"[PyBullet] 初始化失败，已禁用: {e}")

    worker = ComputeWorker(cfg, robot_kinematics, shutdown_event)
    worker_thread = threading.Thread(target=worker.run, daemon=True)
    worker_thread.start()

    print("\n" + "=" * 40)
    print("   Camera → Hand Tracking → Target → IK → Simulation")
    print("   P: Pause | R: Reset | Q: Quit")
    print("=" * 40 + "\n")

    # Matplotlib 每 3 帧更新一次：视觉上仍流畅（~10fps），
    # 但不再每帧阻塞主循环，cv2.imshow 可以全速运行
    _mpl_update_every = 3
    _mpl_frame_counter = 0

    # 主线程：cv2 显示 + matplotlib 更新（两者都必须在主线程）
    try:
        while not shutdown_event.is_set():
            with worker.lock:
                frame  = worker.frame
                nodes  = worker.link_nodes
                angles = worker.current_angles
                target = worker.smoothed_target
                paused = worker.paused

            # ── cv2 窗口（最高优先级，每帧都刷新）──
            if frame is not None:
                cv2.imshow('Robot Arm Simulation', frame)

            # ── Matplotlib 仿真（每 3 帧更新一次，降低阻塞频率）──
            _mpl_frame_counter += 1
            if not paused and nodes is not None and target is not None:
                if _mpl_frame_counter % _mpl_update_every == 0:
                    robot_kinematics.update_plot(nodes, target)

            # ── PyBullet 仿真 ──
            if not paused and pybullet_sim is not None and angles is not None and target is not None:
                try:
                    pybullet_sim.update(angles, target)
                except Exception as e:
                    print(f"[PyBullet] 更新出错: {e}")

            # 键盘输入
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                shutdown_event.set()
            elif key == ord('p'):
                worker.cmd_pause = not worker.cmd_pause
                print(f"Simulation {'PAUSED' if worker.cmd_pause else 'RESUMED'}.")
            elif key == ord('r'):
                worker.cmd_reset = True

    except KeyboardInterrupt:
        print("\nProgram interrupted by user.")
    finally:
        shutdown_event.set()
        cv2.destroyAllWindows()
        worker_thread.join(timeout=2.0)
        if pybullet_sim is not None:
            pybullet_sim.close()
        robot_kinematics.close()
        print("All resources cleaned up.")


if __name__ == "__main__":
    main()
