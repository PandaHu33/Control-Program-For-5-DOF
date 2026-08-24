# WA100 本地运动学模型

## 文件

- `wa100_kinematics.py`：URDF 编译器和运行时正运动学。
- `wa100_kinematics_calibration.json`：六路电机到 11 个关节的可替换标定表，以及五个指尖坐标。
- `wa100_kinematics_model.json`：由 URDF 几何和标定表编译得到的单文件运行时模型。

运行时不依赖 ROS 或原始 STL。网格显示仍应使用原 URDF/STL；骨架、关节和指尖正运动学只需编译后的 JSON。

## URDF 数据如何使用

每个 URDF 关节被保存为：

- 父/子 link；
- `origin xyz/rpy`；
- 局部旋转轴 `axis`；
- 角度上下限。

正运动学按 URDF 树递推：

```text
world_T_child = world_T_parent
              * parent_T_joint_origin
              * rotation(joint_axis, joint_angle)
```

指尖是标定文件中附加在末端 link 上的固定坐标。URDF 本身没有 `*_tip` link，所以当前指尖偏移来自 STL 尺寸估计。

## 运行时用法

```python
from control_ui.wa100_kinematics import WA100Kinematics, transform

hand = WA100Kinematics.load("control_ui/wa100_kinematics_model.json")

# WA100 固定顺序：thumb_pitch, thumb_yaw, index, middle, ring, pinky
actual_units = [1200, 1800, 900, 1000, 1100, 1300]

joint_angles = hand.motor_units_to_joint_angles(actual_units)
local_skeleton = hand.skeleton_from_motor_units(actual_units)
local_tips = hand.fingertip_positions(actual_units)
canonical_21 = hand.canonical_skeleton_21(actual_units)

# 可选：直接输出世界坐标
world_from_hand = transform(translation=[0.8, 0.1, -0.2])
world_skeleton = hand.skeleton_from_motor_units(actual_units, world_from_hand)
```

`canonical_skeleton_21()` 是显控和 Canonical v4 的唯一手部骨架入口。它按
`wrist + thumb(4) + index/middle/ring/pinky(4 each)` 输出 `21×3` 点；四指的
第 3 点是末端关节中心和指尖的固定中点，不引入额外自由度。

Canonical C 坐标系固定为右手掌心向下的默认姿态：骨架主要展开在 XY 平面，
手指由腕部指向 +Y，右手拇指位于 −X 一侧，掌面法向（食指根到小指根的叉积）
指向 −Z。WA100 URDF 的原生 XZ 骨架会先经过这一固定刚体旋转，再应用调用方
提供的 `base_transform`。

每根手指的骨架数组均包含手基座、各 URDF 关节特征点和最后的指尖点。

## 从 URDF 重新编译

```powershell
python control_ui\wa100_kinematics.py compile `
  --urdf "D:\桌面\黑漫灵巧手\WA100_R_URDF\urdf\WA100_R_URDF.urdf" `
  --calibration control_ui\wa100_kinematics_calibration.json `
  --output control_ui\wa100_kinematics_model.json
```

检查模型：

```powershell
python control_ui\wa100_kinematics.py inspect `
  --model control_ui\wa100_kinematics_model.json
```

## 替换为实测标定

当前 `samples_units_rad` 只有开、闭两个点，是按 URDF 限位建立的临时 1:1 耦合。取得真实标定后，为每个关节增加任意数量的测量点：

```json
"Index Finger_J1": {
  "channel": "index",
  "samples_units_rad": [
    [0, 1.42],
    [500, 1.10],
    [1000, 0.72],
    [1500, 0.31],
    [2000, 0.0]
  ]
}
```

运行时使用分段线性插值，并按照 URDF 限位裁剪。修改标定 JSON 后重新编译即可，绘图和分析程序无需更改。

当前模型状态为 `provisional`。在完成真实电机—关节角标定和指尖坐标验证前，适合可视化与流程验证，不应作为定量指尖精度结论。

## 拇指四特征点运动模型

拇指对外提供四个物理特征点：基点关节、第二关节、第三关节和指尖，三段连杆依次连接这四点。

- `Thumb_J1` 使用 `thumb_yaw` 通道。横摇发生在拇指基点，带动三根连杆整体运动。
- `Thumb_J2` 与 `Thumb_J3` 使用同一个 `thumb_pitch` 通道，两级关节耦合弯曲。
- 第一根连杆位于弯曲关节上游，因此弯曲时其两个端点和方向保持不变，只随 `Thumb_J1` 横摇。
- `Thumb_J1` 横摇轴与 `Thumb_J2`、`Thumb_J3` 弯曲轴正交；J2/J3 的几何轴反向平行，映射角符号也相反，因此产生同向的连续弯曲。
- 当弯曲编码器为 2000（弯曲角为零）时，拇指三根连杆严格共线；横摇编码器从 2000 到 0 时，共线方向从局部 +X 转到局部 −Y。
- 横摇编码器为 0 时，三根连杆沿局部 −Y，与局部 XZ 手背平面垂直。
- 腕部原点到拇指基点单独作为掌面连接，不计入拇指的三根运动连杆。
- URDF 限位使用近似值 1.57 rad；骨架层使用 `π/2 ÷ 1.57` 校正系数，使端点姿态严格达到 90°。

可直接获取四特征点：

```python
thumb_points = hand.thumb_feature_points_from_motor_units(actual_units)
assert thumb_points.shape == (4, 3)
```

## 手套模式死区

本地运动学默认复刻 `udp_receiver_unity.cpp` 和当前 `thumb_glove_calibration.json` 的端点死区与 smoothstep：

- 拇指弯曲、横摇：张开端 4%，闭合端 6%；编码器值 `>= 1920` 时关节保持 0°。
- 食指、中指、无名指、小指：张开端 6%，闭合端 12%；编码器值 `>= 1880` 时关节保持 0°。
- 中间区间先去除两端死区、重新归一化，再使用 `t²(3−2t)` 平滑映射。

如需查看未加死区的原始线性角度，可调用：

```python
raw_angles = hand.motor_units_to_joint_angles(actual_units, apply_deadzones=False)
```
