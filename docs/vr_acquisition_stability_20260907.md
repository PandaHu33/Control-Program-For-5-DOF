# VR 主端采集稳定优化与验证（2026-09-07）

## 生效位置

所有新增处理均在 canonical 发布前完成：

- 手腕：现有 canonical One Euro 规划器仅对 `pico_wrist` 加入静止／运动滞回、软死区和静止时禁止预测。40 ms 运动证据避免单个异常帧解除稳定状态；持续缓慢位移仍能释放。
- 手指：Unity 桥接器保留原始关节点，分离手腕平移和相对关节点；对局部尖峰限幅，再按局部速度与残差自适应调整 One Euro 带宽。右手拇指使用更低静止截止频率；左手握拳使能保持原有滤波参数。
- VR 拇指映射：在指尖距离形成锚点融合目标前稳定融合比例／权重，并增加进入、退出和锚点切换确认。数据手套沿用原融合分支。
- canonical 手势：从同一最终目标的 FK 骨架计算，VR 增加时间确认和活动锚点保持；源切换、标定变化、追踪代次变化及中断时重置。
- canonical 到从端没有新增滤波，也没有新增电机目标低通。

## 参数

主端配置文件 `control_ui/config.yaml` 中：

| 参数 | 默认值 |
| --- | --- |
| planner_vr_stability_enabled | true |
| planner_vr_still_cutoff_hz | 0.4 Hz |
| planner_vr_translation_deadband_m | 0.0015 m |
| planner_vr_roll_deadband_rad | 0.004 rad |
| vr_gesture_enter_sec / vr_gesture_release_sec | 0.08 / 0.04 s |

Unity 桥接器可用命令行参数 `--thumb-cutoff-hz 0.25 --finger-cutoff-hz 0.35 --local-beta 20` 调整。移动时带宽自动增大，这些截止频率是低速下限。

现有手部标定 JSON 可选增加 `vr_anchor_stability` 对象；未增加时自动使用默认值：`enter_sec=0.08`、`release_sec=0.04`、`switch_sec=0.10`、`release_scale=1.3`、`still_cutoff_hz=0.65`、`anchor_radius_scale=0.25`。进入半径为现有 `thumb_position_fusion.support_radius` 的 0.25 倍，退出半径再乘 1.3；原有锚点目标和标定文件无需重做。

可在 canonical 的 `planning.vr_stability` 查看静止状态，在 `hand_processing.vr_anchor_stability` 查看活动锚点（-1 无、0 食指、1 中指）、融合比例及权重。原始关节点仍在 `rightPositionsRaw` 中，滤波参数与追踪代次在 `xrSkeletonFilter` 中。

## 离线验证

固定随机种子、相同输入下相对旧算法的标准差降幅：

| 信号 | 降幅 |
| --- | --- |
| 手腕位移 | 71.3% |
| 腕部转动 | 61.3% |
| 拇指弯曲 | 55.2% |
| 拇指横摇 | 51.0% |

手腕测试注入 1 mm 位移噪声和 0.004 rad 转动噪声；手指测试注入 0.35 mm 关节点噪声，通过现有 VR 拇指几何映射计算两个通道。以上为合成数据结果，不代表真实设备的测量降幅。

验证包括：30/50/90 Hz 手指动作与尖峰、整手平移不改变相对形状、慢速位移脱离稳定区、10 秒锚点保持、主动切换与松手、重复帧不推进状态、追踪中断／代次变化、其他臂源行为保持一致。

- Python：44 passed，3 skipped。跳过的三项是缺少 OpenCV 的相机镜像测试。
- C++：`vr_anchor_stability_test` 和 `wa100_channel_layout_test` 均通过。
- Release 构建成功：`wa100-sdk-publish/build/bin/udp_receiver_unity.exe`。编译器仍报告既有 `getenv`／`inet_addr` 弃用警告。
- 扩展运行 `test_h5_source_helpers.py` 时，两项未改动的 ROS/H5 链路测试失败：`test_fresh_versioned_state_blocks_competing_generic_joint_state` 和 `test_generic_joint_state_without_named_j4_is_rejected`。本次未修改该 ROS 桥接实现或该测试文件。

## 实物测试交接

重启主端后端、Unity 桥接器及手部接收进程，并刷新控制页面，以加载新 Python 代码、配置、接收程序及手腕参考会话字段。本次没有启动实物控制程序或发送测试动作。

建议佩戴后依次观察：静止手腕、慢移／快移后停住、拇指独立横摇／弯曲、食指捏合保持、切换中指、主动松手、短暂遮挡后恢复。重点比较 canonical 显示与手势标签，以及松手时是否有明显拖延；实物测试由用户执行。
