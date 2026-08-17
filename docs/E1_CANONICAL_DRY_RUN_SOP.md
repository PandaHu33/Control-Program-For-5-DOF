# E1 Canonical no-send dry-run SOP

本 SOP 仅用于 P0/E1 的四种主端条件。MATLAB 主手不属于本轮 E1；P1/P2 与 E2（导纳、shadow mode、闭环修正）继续保持 `BLOCKED`。首次正式试次前，四种条件都必须完成一次 no-send dry-run。dry-run 期间禁止向机械臂和 WA100 下发新目标。

Canonical 层以固定 50 Hz 发布最新有效语义目标，而键盘、游戏手柄、PICO、VR 手柄和手套保持各自原生采样频率。这样可将异步、异频主端统一成同频、同结构的中间语义流，便于公平比较、同步记录和确定性回放。

## 1. 上电前与配置冻结

1. 清空机械臂和灵巧手工作空间，确认急停可触达、限位与 WA100 过流保护有效；操作者在设备旁监护。
2. 检查机械臂、WA100、PICO/VR、手柄或键盘、Hi5 手套连接。检查双相机画面、时间连续性和磁盘剩余空间（至少 2 GiB）。
3. 确认 `control_ui/config.yaml` 中 `canonical.control_path_enabled: true`。E1 正式取数时还应为 `canonical.log_enabled: true`；日志开关不得用于改变控制结果。
4. 冻结并记录以下文件/编号：
   - 当前配置所指向的 Hi5 模型（现为 `control_ui/hi5_hand_model_v2.json`）的 `calibration_id` 和程序显示的 SHA-256；旧记录继续使用其记录在案的 v1 标定；
   - 手套拇指标定 `calibration_id`；
   - wrist mapping、PICO 外参和相机配置版本。
5. 任一标定文件缺失、哈希变化、编号不匹配或数值非有限时，停止本次检查，恢复正确冻结文件后重新启动程序，不得沿用旧输入。

## 2. 条件选择

| 条件 | 机械臂显控选择 | 灵巧手选择 | Canonical wrist / hand 来源 |
|---|---|---|---|
| `M1-PICO` | PICO 手部追踪 | VR 手部追踪 | `pico_wrist` / `pico_hand` |
| `M2-VR-GLOVE` | VR 手柄增量 | 手套操控 | `vr_controller` / `data_glove` |
| `M3-GAMEPAD-GLOVE` | 游戏手柄、位置控制 | 手套操控 | `gamepad` / `data_glove` |
| `M4-KEYBOARD-GLOVE` | 键盘、位置控制 | 手套操控 | `keyboard` / `data_glove` |

`M2-VR-GLOVE` 中 wrist authority 只能来自 VR 手柄。`master_fusion` 仅作诊断遥测，手套姿态不得注入 J4；`Hand_Tracker/hand_control_config.yaml` 中 `master_fusion.apply_to_controller_delta` 必须为 `false`。

## 3. no-send dry-run

1. 启动显控和所需传感器程序，但先不要开始正式实验。WA100 原生端应显示 `idle (holding current position)`。
2. 打开 Debug 页，在“UDP/WS 传输”中勾选 **E1 no-send dry-run**。该开关会继续生成、校验、显示和记录 canonical 流，但浏览器不发送机械臂 H5 帧，bridge 也不发送 WA100 canonical 目标。
3. 选择本条条件的机械臂和灵巧手来源。切换 authority、重连、复位或重新居中后，保持主端静止一帧：首帧只建立锚点，不应出现目标跳变。
4. 缓慢操作各有效方向和手指，至少持续 10 秒。确认：
   - 状态页 `Canonical样本` 持续增长，来源与上表一致；
   - wrist mask 为 5-DoF 当前支持位（XYZ 与实际支持的腕部旋转），hand 为 `21/21`；
   - `condition_id` 对应当前条件，标定编号和哈希正确；
   - `Canonical最近错误` 为空；机械臂与 WA100 不运动。
   - 各主端保持原生采样频率；`canonical_goal` 目标与实测频率均约为 50 Hz，周期 P95 接近 20 ms。
5. 依次验证释放按键后停止积分、手柄死区、PICO/VR 重连重新锚定，以及 quaternion 符号翻转不跳变。`stale`、重复/倒退序号或重定心后不得继续累计旧增量。

## 4. 记录与停录检查

1. 保持 **E1 no-send dry-run** 已勾选，点击“开始记录”；操作 10–30 秒后点击“停止记录”。不要直接关闭程序。
2. 在 `recordings/<日期>/<会话>/` 逐项检查：
   - `manifest.json` 存在，且 `complete: true`；
   - `raw_input.jsonl` 存在、非空、各要求来源均有计数；
   - `canonical_goal.jsonl` 存在、非空、每行可解析，`valid_samples > 0`；
   - `canonical_goal` 摘要包含来源分布、校准编号、无效原因和序号丢失数；
   - `left.mp4`、`right.mp4` 及时间戳/对齐文件存在且非空。
3. E1 在 canonical 日志启用时，文件缺失、为空、不可解析或没有有效记录，必须由 `manifest.json` 标记为 `incomplete`。非 E1 记录不因关闭 canonical 日志而自动失败。

## 5. 离线回放

在程序根目录执行（工具不打开控制 socket）：

```powershell
python control_ui\replay_canonical.py "recordings\<日期>\<会话>" --tolerance 1e-6 --output "recordings\<日期>\<会话>\canonical_replay_report.json"
```

返回码为 0 且报告中 `ok: true`、`no_send: true` 才通过。相同 `raw_input.jsonl` 重放两次必须得到一致结论；腕部积分误差不得超过 `1e-6`，重建的 21 节点手骨架必须与记录一致。

## 6. 判定与异常恢复

- **PASS**：no-send 确认无执行器运动；来源、掩码、条件和标定正确；日志完整；回放通过；无重连跳变。
- **FAIL**：目标误差超限、通道不兼容、重连跳变、释放后继续移动、日志不可解析或回放不一致。停止记录并保留失败会话，不得进入正式试次。
- **INVALID**：stale、标定/哈希不匹配、设备掉线、磁盘/相机失败、急停或人为中断。停止记录并在 manifest/实验记录中注明原因，排除问题后从步骤 1 重做。

发生 stale 时保持输出并恢复数据源；标定不匹配时关闭相关输入、恢复冻结文件并重启；VR/PICO 重连时必须重新锚定且首帧零增量；`canonical_goal.jsonl` 缺失时不得补写或手工改 manifest，应新建一次记录。完成四条 dry-run 后仍将 E1 标记为 `NOT_RUN`，只有人工审核通过后才允许开始正式 E1 试次。
