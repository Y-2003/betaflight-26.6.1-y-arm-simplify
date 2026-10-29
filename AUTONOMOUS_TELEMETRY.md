# 保持控制器的 OSD 与黑匣子数据

2026-10-27：运动数据改为读取实际控制循环的快照，不再读取已经停用的自主轨迹接口。快照独立于 `debug_mode`，不改变控制目标、PID、模式切换或电机混控算法。旧日志不会因此改变。

## OSD

自主状态行的 `T` 是最终总油门指令。自主运动项占三行，需要为它预留三行空间：

```text
TH 目标高度    H 当前高度
TVV 目标垂速   VV 当前垂速
HE 水平误差    THV 目标水平速率    HV 当前水平速率
```

高度、误差单位为 m，速度为 m/s，显示一位小数。高度为估计器相对参考点的高度，垂直方向向上为正。当前速度是控制器使用的估计速度，不是单独读取的 GPS 原始速度。无有效控制快照时显示 `--`，油门显示 `----`。

## 黑匣子

自主组保留 13 个数值字段，运动字段读取实际控制数据：

| 字段 | 含义与单位 |
| --- | --- |
| `autonomousState` | 自主状态机阶段编号 |
| `autonomousValidity` | Z、XY、加速度计、接收机有效性位标志 |
| `autonomousArmResult` | 最近一次自主解锁结果 |
| `autonomousArmBlockingFlags` | 最近一次自主解锁阻止原因位标志 |
| `autonomousTargetAltitude` | 本次高度控制目标，cm；与高度 debug[2] 同源 |
| `autonomousAltitude` | 本次控制使用的高度，cm；与高度 debug[3] 同源 |
| `autonomousTargetVerticalVelocity` | 限幅后的目标垂直速度，cm/s，向上为正 |
| `autonomousVerticalVelocity` | 本次控制使用的垂直速度，cm/s，向上为正 |
| `autonomousHorizontalDistance` | 本次控制实际使用的水平距离误差向量的模，cm |
| `autonomousTargetHorizontalVelocity` | 目标水平速度向量的模，cm/s |
| `autonomousHorizontalVelocity` | 当前水平速度向量的模，cm/s |
| `autonomousThrottlePwm` | 最终总油门，按 `1000 + 1000 × 最终归一化总油门` 表示；1000 为零总油门，2000 为满总油门 |
| `autonomousControlStatus` | 下表定义的状态位 |

最终总油门在 ALTHOLD 等模式覆盖和姿态混控去饱和调整之后采集，因此包含 POSHOLD 通过姿态修正对混控的影响。它不是高度控制的原始 PWM 数值，也不是电机实测推力；各路最终电机命令仍看 `motor[]`。未解锁、motor-stop 或显式停止电机时总油门为 1000。翻龟和 3D 模式没有这个普通单向总油门定义，标记无效。

控制器快照有效期为 100 ms，失效时运动数值置零，必须结合状态位判断；真实目标也可以为零。采样时间仅在内部用于过期判断，不作为黑匣子字段输出。OSD 与黑匣子都读取相同来源的最近有效快照。

| `autonomousControlStatus` 位 | 含义 |
| --- | --- |
| 0 / 1 | ALTHOLD / POSHOLD 内部模式标志已置位 |
| 2 / 3 | 高度 / 位置控制器已接管 |
| 4 / 5 | 当前记录的高度 / 水平控制快照可用，相关估计数据有效 |
| 6 | 水平误差来自真实位置锚点；为零且 bit 5 有效时，是速度误差积分产生的虚拟距离误差 |
| 7 | 最终总油门记录有效 |

`autonomousValidity` 的既有位定义不变。按用户要求，不再记录总状态位、解锁尝试次数、三个采样时间和目标/当前水平速度的方向分量；水平误差也只记录大小，不记录东/北分量。字段头、I 帧和 P 帧同步调整，依赖固定列位置的自定义解析脚本需要按新字段头读取。

## 验证

```sh
make -C src/test test_mixer_telemetry_unittest test_althold_unittest test_poshold_unittest test_osd_unittest test_blackbox_unittest -j4
make CONFIG=GEPRCF722_BT_HD_V3 -j4
```

新增控制器测试覆盖限幅后的目标速度、估计反馈、位置/导航目标切换、误差分量及过期/失败复位。新增混控测试直接运行真实 mixer，验证输入 0.24、定高要求 0.7、姿态去饱和后总油门 0.5 的区别，以及后续更新、未解锁、电机停止、过期和 3D 无效标记。
