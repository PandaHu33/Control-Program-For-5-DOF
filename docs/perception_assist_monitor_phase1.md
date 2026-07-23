# 第一阶段形态—负载一致性抓持判别（monitor-only）

## 真实链路与边界

- `Hand_Tracker/unity_hand_udp_bridge.py` 继续只用左手握拳作为 deadman。新增的右手四指平均弯曲度名为 `right_operator_intent_closure_prior`，明确只是操作者期望先验，不参与 WA100 实际闭合度或抓持状态判别。
- `wa100-sdk-publish/examples/udp_receiver_unity.cpp` 在既有 50 Hz 柔顺循环中读取 WA100 实际位置和电流。它继续保留硬件限流、软卸力和拉索单向柔顺逻辑，仅向 UDP 25003 增加只读遥测字段。电流语义为负载/接触代理，不是真实力或触觉。
- `control_ui/bridge.py` 在 UDP 25003 旁路运行 `PerceptionAssistMonitor`，通过现有 H5 状态 WebSocket、REST `/api/perception-assist/status` 和 Unity 提示 UDP 25004 发布统一消息。状态机不持有机械手/机械臂命令 socket，不改变命令。
- 默认 `perception_assist_config.json` 故意不包含实测开闭边界或无接触电流基线；在完成标定之前输出 `valid=0`。

## 输入和计算

每个 WA100 通道需要：实际位置 `q_i`、滤波电机电流 `I_i`、逐通道位置有效性和源时间戳。速度 `dq_i` 由相邻有效实际位置及后端单调接收时间差计算。

位置与电流有效性彼此独立。单帧位置出现 `0xFFFF` 等无效值时，WA100 接收器继续发布同一状态帧中有效的电流，并标记独立的 `position_valid[]` 与 `current_valid[]`。后端在 `position_fallback_grace_sec` 内使用上一有效位置、令该通道速度为零，使瞬时位置毛刺不会清空负载判别；超时后停止形态—负载联合推断，但仍通过 `filtered_current_ma` 和 `signal_validity.current` 保留电流可见性。

对于零点附近特有的无符号回绕，底层仅在原始位置为 `65000..65535` 且上一有效实测位置不大于 20 时归一化为 0，并发布 `position_zero_wrap_corrected[]`。判据不依赖期望命令或柔顺偏移；远离零点历史的大数仍视为异常，避免掩盖坏包或断线。

实际闭合度只由实测 WA100 边界计算：

`c_i = clip((q_i - q_open_i) / (q_close_i - q_open_i), 0, 1)`，`C = sum(w_i*c_i)`。

无接触基线采用配置化的 `affine_q_dq_direction` 模型：

`I0_i = intercept + a_q*q_i + a_v*abs(dq_i) + offset[opening|stationary|closing]`

`r_i = I_i - I0_i`。每个模型必须带 `valid=true`，且三个方向偏置齐全。任何必需通道缺少标定、基线、速度历史、有效反馈或新鲜时间戳时，整次判别降级为 `FREE, valid=0`，不输出抓持推断。

## 状态转移

所有数值均来自 `control_ui/perception_assist_config.json`，文件中的数值只是待实机整定的工程占位值，不是论文结论或实测阈值。

| 当前证据 | 目标状态 | 迟滞/时间条件 | 提示含义 |
|---|---|---|---|
| 无有效受载，或闭合/负载不一致 | `FREE` | 负载用 `load_on/load_off` 锁存；抓持退出闭合阈值低于进入阈值 | 空载或无联合证据 |
| `C < Cg_enter` 且至少一个有效通道残差达到负载门槛 | `EARLY_CONTACT` | 负载关闭阈值低于开启阈值 | 疑似边缘、障碍或错误接近，建议停止继续闭合并调整 |
| `C >= Cg` 且有效受载通道数 `>= Ng` | `GRASP_CANDIDATE` | 联合证据连续保持 `candidate_hold_sec` | 疑似包覆抓持；不是“已成功抓住” |
| 候选期间闭合度范围、各受载通道残差范围及受载通道数稳定 | `GRASP_SUPPORTED` | 覆盖 `supported_hold_sec` 的短窗，受 `supported_stability_window_sec` 限制 | 提示操作者确认保持；仍不是成功结论 |
| 任一有效电流残差达到风险阈值 | `OVERLOAD` | 进入/退出阈值分离，满足 `overload_hold_sec`；退出还需 `overload_release_sec` | 建议松开或调整；不替代 WA100 本地保护 |
| 数据过期、反馈无效、标定/基线缺失 | `FREE, valid=0` | 清除所有候选、稳定和负载锁存状态 | 不作抓持推断，原控制与本地保护继续独立运行 |

`GRASP_SUPPORTED` 只表示短时“形态与多指负载代理一致”。要陈述已成功抓住，仍需要人工确认、视觉确认，或经过安全评审的保持测试。

## 统一消息与提示接口

消息 `type=perception_assist_state, schema_version=1` 至少包含：

- `timestamp/timestamp_ns/timestamp_source`：优先 WA100 `source_time_ns`，缺失时用 bridge UTC 接收时间；
- `decision_seq/frame_id`：状态机决策序号和 WA100 遥测 `seq`；
- `state/closure_score/current_residual/loaded_fingers`；
- `valid/invalid_reason/reason`；
- `events/prompt_event/prompt_message`；
- 永远固定的 `monitor_only=true`、`grasp_success_confirmed=false`；
- 对照诊断 `ablation.current_only/closure_only/joint`。

H5 从状态 WebSocket 中的 `perception_assist` 字段显示提示。Unity 可只读监听 `127.0.0.1:25004`：`EARLY_CONTACT` 为边缘早碰提示，`GRASP_CANDIDATE/GRASP_SUPPORTED` 为疑似抓持及确认保持提示，`OVERLOAD` 为松开/调整提示。UDP 消息没有命令、目标位置或增力字段。

## EpisodeRecord

开始多模态记录后，`RecordingManager` 生成 `episode_records.csv`。每个决策记录源时间戳及其来源、bridge UTC/单调接收时间、决策/帧序号、状态、闭合度、残差 JSON、受载通道 JSON、有效性/原因、提示事件、三种对照结果，以及固定的 `monitor_only=1`、`grasp_success_confirmed=0`。原始 WA100 数据仍独立写入 `hand.csv`，避免用决策结果替代原始证据。

## 对照与假阳性报告

- 仅电流：`loaded_finger_count >= Ng`；同一拇指的两个电机只计一个手指。低闭合度下同时碰到物体边缘的多个手指可能被误报为抓持。
- 仅闭合度：`C >= Cg`。空载闭合会被误报为抓持。
- 联合：同时要求闭合构型、多通道有效残差和持续/稳定时间，可把上述两类情况保留为 `EARLY_CONTACT` 或 `FREE`。

合成测试只验证逻辑，不报告成功率、假阳性率或端到端时延。真实假阳性必须在标注“边缘碰撞”的实机 episode 上统计：以 `GRASP_CANDIDATE/GRASP_SUPPORTED` 作为预测阳性，边缘碰撞作为真实阴性，单独报告被误判的 episode 数和比例。

## 未验证硬件假设与最小实机验证

尚未验证：六个电机与物理手指/自由度的最终映射；各通道实际开闭端点及方向；WA100 电流单位、滤波相位和温漂；无接触基线对位置、速度、运动方向的拟合充分性；不同物体/姿态下 `Ng` 与稳定窗；UDP 25003/25004 在目标 Windows 环境的丢包和调度抖动。

最小验证分四步，期间状态机保持 monitor-only：

1. 空手、低速条件下逐通道往返，采集实际 `q/dq/I`，确认通道映射和实际开闭端点。
2. 空手在至少三档速度下重复开/闭，分 opening/stationary/closing 拟合基线；留出独立往返验证残差分布和温漂，不用训练数据自证阈值。
3. 用软物体执行“空载闭合、单指/边缘早碰、正常包覆、多指稳定、接近现有软卸力但不越过安全边界”五类人工标注 episode；不抬升物体。
4. 只比较三种对照的混淆矩阵，重点报告边缘碰撞被判成候选/支撑的假阳性；确认本地过流保护和单向卸力触发行为完全未变后，再讨论视觉确认或经安全评审的保持测试。
