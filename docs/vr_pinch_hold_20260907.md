# VR 捏合保持：移动机械臂时的拇指稳定改动

## 后续修正：vr-native-contact-hold-v3

核对实际加载的 thumb_glove_calibration.json 后发现，原实现把 Hi5 原生 FK 距离用于 XR 米制坐标，未转换尺度。因此下方旧版基于 support_radius／标定接触距离的 VR 范围规则已废止。VR 现在只复用标定目标，使用自身指尖距离判定：默认 25 mm 内确认捏合、45 mm 外持续 180 ms 退出、75 mm 外持续 60 ms 快速退出。距离为两指指尖距离，不再是相对手套标定距离的增量。可配置 xr_contact_enter_m（默认 0.025）；release_scale、fast_release_scale 与确认时间继续生效。旧 contact_margin_m／anchor_radius_scale 不参与新的 VR 判定。

启动控制台打印 build=vr-native-contact-hold-v3 对应版本行及 Loaded executable 绝对路径；canonical 的 hand_processing.vr_anchor_stability.build 也携带版本。锚点状态变化时打印 anchor=-1（未保持）、0（食指）、1（中指）和实际指尖距离。

一键显控从 control_ui/config.yaml 的 programs.hand_exe 读取路径；相对路径基于项目根目录解析，指向 wa100-sdk-publish/build/bin/udp_receiver_unity.exe。本次核对时未发现运行中的接收进程，只确认启动链路和已生成文件，不声称核实了此前某一次运行的进程版本。此修正仅编译，未执行测试。

## 之前的修改记录（范围规则以上述 v3 为准）

这次修改针对已经确认捏合后，平移手部导致拇指变形／松开的现象。所有处理仍位于 VR 采集映射到 canonical 之前，没有在 canonical 到从端增加滤波。

原逻辑中，锚点有效时融合权重仍随距离下降，抖动的拇指估计会重新影响目标。同时，两个指尖距离组成的特征可因非捏合手指移动而触发释放。

新逻辑：

- 确认锚点后，融合权重连续收敛到 1，保持标定的拇指弯曲／横摇姿态；退出确认前不随瞬时融合权重下降。
- 进入锚点的特征范围从 support_radius 的 0.25 倍扩大至 0.40 倍，同时要求对应两指足够靠近，避免只扩大范围造成误吸附。
- 保持食指捏合时，使用拇指—食指分离量判断退出；中指捏合对应拇指—中指。另一根手指移动不直接解除当前捏合。
- 默认在标定接触距离以外，额外 12 mm 内可确认进入；分离超过 21.6 mm 持续 180 ms 后退出。明显张开超过 36 mm 时，持续 60 ms 可快速退出。这里的距离都是相对标定接触距离的增量。
- 切换锚点需要原捏合开始分开、新锚点明显更近且持续 160 ms。快速退出独立计时，避免普通退出计时累积后被单个尖峰触发。
- 追踪丢失、超时、模式切换和标定重载仍清除锚点历史。禁用融合或将 influence 设为 0 时不启用保持。

可在现有标定 JSON 的 vr_anchor_stability 中覆盖默认值：release_sec=0.18、switch_sec=0.16、release_scale=1.8、anchor_radius_scale=0.40、contact_margin_m=0.012、fast_release_scale=3.0、fast_release_sec=0.06、hold_weight=1.0。已显式配置的值优先于默认值。

按用户要求，本次不执行自动测试、回放或实物测试。只编译 udp_receiver_unity 的 Release 程序；编译通过不代表行为验证通过。实际松手响应还包含输入滤波与采样延迟，上述毫秒数仅为退出确认时间。

加载方式：重启 wa100-sdk-publish/build/bin/udp_receiver_unity.exe 对应的手部接收进程。
