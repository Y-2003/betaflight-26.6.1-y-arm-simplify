# Betaflight 自主飞行代码说明

本文说明 simplify 简化版的自主飞行实现，阶段定义更新于 2026-10-26。旧记录中的 FLIP/BRAKING/READY 编号不适用于本版；其他集成点行号仅作定位参考。

2026-10-27 的 OSD/黑匣子控制器数据来源、字段单位和有效性定义见 [AUTONOMOUS_TELEMETRY.md](AUTONOMOUS_TELEMETRY.md)。

代码来源标识：

- `[自定义新增]`：项目自行添加的文件或独立模块，Betaflight 官方基础代码中不存在。
- `[官方代码修改]`：在 Betaflight 官方文件中加入或调整的集成逻辑。

## 涉及文件

与其他 Betaflight 代码融合时，自定义新增文件可以直接加入；官方代码修改文件需要逐个合并差异。

### [自定义新增] 代码与工具

- `src/main/flight/autonomous_mode.c`
- `src/main/flight/autonomous_mode.h`
- `gazebo_pose.py`
- `sitl_rc.py`
- `tx16s_sitl_rc.py`

### [官方代码修改] 需要合并差异

- `mk/source.mk`
- `src/main/fc/core.c`
- `src/main/fc/core.h`
- `src/main/fc/rc.c`
- `src/main/fc/rc_controls.c`
- `src/main/fc/tasks.c`
- `src/main/flight/failsafe.c`
- `src/main/flight/failsafe.h`
- `src/main/flight/mixer.c`
- `src/main/flight/pid.c`
- `src/main/msp/msp_box.c`
- `src/platform/SIMULATOR/sitl.c`
- `src/platform/SIMULATOR/target/SITL/target.h`
- `src/test/unit/flight_failsafe_unittest.cc`

### 说明文档

- `CUSTOM_CHANGES.md`

## 功能概览

自主流程使用 ARM、USER1、USER2、USER3 和 USER4 开关输入：

- ARM 是飞手允许自主解锁的许可，同时也是 USER3 投掷流程的触发边沿输入。
- USER2 上升沿作为一次性触发，要求 ARM 已打开且 USER1 能读到有效位置；触发后请求 synthetic failsafe 接收链路锁定。failsafe 完成处置、飞机解除 ARM 并恢复有效 RX 后，再等待代码头部定义的 200 ms 延时，进入自主解锁和自由落体检测。当前 USER2 高电平可以继续完成本次流程；下一次触发前必须重新检测到低电平。
- USER3 是投掷机专用的备用触发路径：USER2 为低、USER3 为高且 ARM 出现上升沿时触发自主流程，同样要求 USER1 初始位置有效。USER3 触发边沿后释放 synthetic RX 锁，等待自由落体；USER3 高电平期间禁止普通 ARM 解锁。
- USER1 用于读取三段拨杆初始位置并检测位置变化。自主流程开始时记录初始位置，运行期间位置持续变化 30 ms 会中止自主流程。

USER1 需要在地面站的 Modes 页面配置，以便固件从 `modeActivationCondition` 取得对应的 AUX 通道索引。配置的激活范围只用于让该映射有效；代码随后直接读取该通道的原始 PWM：

- 小于 1300：`AUTONOMOUS_USER1_POSITION_LOW`
- 1300 至 1700：`AUTONOMOUS_USER1_POSITION_MIDDLE`
- 大于等于 1800：`AUTONOMOUS_USER1_POSITION_HIGH`
- 1701 至 1799：`AUTONOMOUS_USER1_POSITION_INVALID`

三个 USER1 位置不会直接启用保持模式。USER4 只在阶段 5 的新上升沿申请 AUTOPILOT；提前打开并保持不会排队，也不再绕过自由落体自主解锁条件。

## 自主状态机（当前简化版）

| state | 枚举 | 行为及进入下一阶段的条件 |
| ---: | --- | --- |
| 0 | IDLE | 等待原有 USER2 或 USER3+ARM 触发 |
| 1 | WAITING_FOR_ARM | 原有 failsafe/RX 恢复；恢复后等待 200 ms |
| 2 | ACCEL_WAITING | 加速度小于 0.80 G 持续 30 ms，再尝试自主解锁；成功后进入 3 |
| 3 | REQUESTING_HOLD | 只申请 ANGLE、ALTHOLD；两个内部模式标志与定高控制器生效，且 roll、pitch 绝对值都严格小于 20°，连续保持 0.5 s 后进入 4 |
| 4 | REQUESTING_POSITION_HOLD | 维持 ANGLE、ALTHOLD，再申请 POSHOLD；三个内部模式标志与定高、定点控制器全部生效后进入 5 |
| 5 | HOLD | 三模式确认后保持等待，不自动推进；USER4 新上升沿且 RX/保持控制有效后进入 6 |
| 6 | REQUESTING_AUTOPILOT | USER4 已开启，维持三模式保持并提交 AUTOPILOT 请求；模式处理接收后进入 7 |
| 7 | WAITING_FOR_AUTOPILOT | 请求已提交，继续保持并等待内部 AUTOPILOT_MODE 标志置位；确认后进入 8 |
| 8 | AUTOPILOT | 航线接管；释放自主姿态输入及油门覆盖，保持模式由 AUTOPILOT 驱动 |

状态编号显式写在 `src/main/flight/autonomous_mode.h`；黑盒 `autonomousState` 和 `DEBUG_FAILSAFE[4]` 使用这些编号。**本版 state3 确认 ANGLE/ALTHOLD，state4 再确认 POSHOLD，state5 等待 USER4，state6 提交 AUTOPILOT 请求，state7 等待内部标志确认，state8 才表示 AUTOPILOT 已启用。**

阶段 3 要求 ANGLE/ALTHOLD 已生效，且横滚、俯仰绝对值都严格小于 20°，连续保持 0.5 秒后，下一阶段才申请 POSHOLD。任一轴达到或超过 20°，或者模式/定高控制器失效，都清除本次计时；中止流程或回退到阶段 3 后也需要重新确认完整的 0.5 秒。主动翻转、自定义 XY/Z 刹车、旧 READY、姿态重入分支通过 `#if 0` 暂时停用，源码保留；翻转、刹车、定制目标兼容接口返回 false/零，使用标准 ALTHOLD/POSHOLD 控制器。阶段 2 至 7 保持自主输入屏蔽和悬停油门基准。

临时关闭阶段 3 的姿态和计时判定：将 `src/main/flight/autonomous_mode.c` 头部的 `AUTONOMOUS_STATE3_LEVEL_CHECK_ENABLED` 从 `1` 改成 `0`，重新编译并刷写固件。关闭后仍确认 ANGLE/ALTHOLD 及定高控制器已生效，然后直接进入阶段 4；恢复为 `1` 即重新启用双轴小于 20° 连续 0.5 秒的条件。

阶段 4 若 ANGLE/ALTHOLD 或定高控制器失效，回到 3；仅 POSHOLD 未生效时留在 4 等待。阶段 5/6/7 若三个模式或保持控制器失效，回到 3 重新确认。阶段 6/7 关闭 USER4 回到 5（保持失效则回 3）。阶段 8 关闭 USER4 或丢失 AUTOPILOT_MODE 回到 3，再经 4 确认后进入 5；再次申请航线需要 USER4 新上升沿。ARM 关闭、USER1 变化持续 30 ms、failsafe 的原有退出处理保留。

`src/main/fc/core.c` 接收自主 AUTOPILOT 请求时回调，使 6→7；实际启用 AUTOPILOT_MODE 后再回调，使 7→8。提交请求不会直接记为 8，阶段 5 也不会自动变为 6。自主流程中原始 BOXAUTOPILOT 不得提前接管；failsafe/rescue 逻辑保留优先级。

自主解锁专用检查允许屏蔽 ANGLE、THROTTLE、ARM_SWITCH、ALTHOLD、POSHOLD、AUTOPILOT 禁止标志，普通解锁检查不变。`throttleRaised` 在自主解锁后置位，后续航线模式仍须满足 ACC、GPS/FIX、高度可用等条件。

测试入口：

- 离线状态机回归：`make -C src/test test_autonomous_mode_unittest`
- SITL 集成：`python3 src/test/sitl/autonomous_user4.py --binary obj/main/betaflight_SITL.elf`
- 飞控构建：`make -j4 CONFIG=GEPRCF722_BT_HD_V3`

## 飞控集成点

### [官方代码修改] 构建与调度

- `mk/source.mk:169`：将 `flight/autonomous_mode.c` 编入固件。
- `src/main/fc/tasks.c:128`：`taskMain()` 周期调用 `autonomousModeUpdate()`；原先在任务中单独处理 USER1/USER2/USER3 边沿的逻辑已移入自主状态机。

### [官方代码修改] 解锁与飞行模式

- `src/main/fc/core.c:567`：`tryArmInternal()` 合并普通和自主解锁检查；自主解锁额外忽略 ANGLE、THROTTLE、ARM_SWITCH、ALTHOLD、POSHOLD、AUTOPILOT 阻止标志。
- `src/main/fc/core.c:708`：`tryArm()` 保留普通飞手解锁入口。
- `src/main/fc/core.c:715`：`tryArmAutonomous()` 提供自主解锁入口并返回结果。
- `src/main/fc/core.h:80`：声明 `tryArmAutonomous()`。
- `src/main/fc/core.c:850`：`processRx()` 在自主翻转期间保持 PID 稳定化。
- `src/main/fc/core.c:992`：`processRxModes()` 应用飞手输入屏蔽及 ANGLE、高度和位置控制请求；USER1 不再直接启用高度或位置保持。
- `src/main/fc/core.c:1345`：`subTaskPidController()` 在自主翻转期间抑制失控起飞保护误触发。

### [官方代码修改] RC 输入与油门

- `src/main/fc/rc.c:108`：`getFeedforward()` 在自主翻转期间关闭飞手前馈。
- `src/main/fc/rc.c:623`：`processRcCommand()` 在翻转期间输出自主角速度，其他自主阶段屏蔽飞手姿态输入。
- `src/main/fc/rc.c:721`：`updateRcCommands()` 在翻转期间输出闭环翻转油门，其他自主阶段输出悬停或入口油门。
- `src/main/fc/rc_controls.c:148`：`processRcStickPositions()` 在自主流程接管或 USER3 高电平期间禁止普通 ARM 解锁，并保留 failsafe 解锁阻止状态。

### [官方代码修改] Failsafe

- `src/main/flight/failsafe.c:106`：`failsafeReset()` 复位自主链路锁定请求和触发锁存。
- `src/main/flight/failsafe.c:153`：`failsafeIsActive()` 报告真实、BOXFAILSAFE 或 USER2 自主锁定触发的 failsafe 状态。
- `src/main/flight/failsafe.c:158`：`failsafeIsAutonomousLockActive()` 查询自主 synthetic RX 锁。
- `src/main/flight/failsafe.c:164`：`failsafeRequestAutonomousLock()` 设置或解除自主链路锁定；USER2 触发的锁在 failsafe 完成解锁后、飞机已解除 ARM 时自动释放。
- `src/main/flight/failsafe.c:217`：`failsafeIsReceivingRxData()` 在自主锁定期间报告链路不可用。
- `src/main/flight/failsafe.c:228`、`:234`：RX 挂起/恢复回调在自主锁定期间保持链路 down。
- `src/main/flight/failsafe.c:242`：有效 RX 数据在自主锁定期间不立即恢复正常链路。
- `src/main/flight/failsafe.c:344`：`failsafeUpdateState()` 将自主锁定纳入 failsafe 触发、解锁和 RX 恢复流程。
- `src/main/flight/failsafe.c:632`：在 `DEBUG_FAILSAFE` 中输出自主锁定状态和解锁阻止标志。
- `src/main/flight/failsafe.h:110`：声明自主锁定查询和请求接口。

### [官方代码修改] 历史翻转保护接口（当前简化版不触发）

- `src/main/flight/mixer.c:678`：`mixTable()` 在自主翻转期间强制启用空中模式补偿。
- `src/main/flight/pid.c:651`：`handleCrashRecovery()` 在自主翻转期间跳过失控恢复干预。
- `src/main/flight/pid.c:699`：`detectAndSetCrashRecovery()` 在自主翻转期间禁止碰撞恢复触发。
- `src/main/flight/pid.c:1059`：`pidController()` 接入自主翻转保护。
- `src/main/flight/pid.c:1154-1195`：修正 chirp 轴枚举的循环、类型转换和格式化判断。

### [官方代码修改] 地面站开关映射

- `src/main/msp/msp_box.c:202`：`initActiveBoxIds()` 初始化 MSP 可见开关，并固定暴露 BOXUSER1、BOXUSER2 和 BOXUSER3。
- `src/main/msp/msp_box.c:336-338`：暴露 USER1/USER2/USER3 的 AUX 通道映射入口；USER1 的配置仅用于选择待读取的原始通道，不直接启用高度或位置保持模式。

## SITL 与辅助工具

### [官方代码修改] SITL 固件

- `src/platform/SIMULATOR/sitl.c:276`：`updateState()` 接收仿真状态，并周期输出姿态/加速度诊断信息。
- `src/platform/SIMULATOR/target/SITL/target.h:123-125`：为 SITL 启用 `USE_CHIRP`、`USE_RX_MSP` 和 `USE_RX_MASP_OVERRIDE`。
- `src/platform/SIMULATOR/target/SITL/target.h:238-242`：明确 Gazebo UDP IMU 数据使用 FRD 传感器坐标系，姿态四元数使用桥接包坐标系。

### [自定义新增] `gazebo_pose.py`

Gazebo 模型位姿交互工具，手动运行，不由固件调用。

- 第 31 行：`quaternion_from_euler()`，将欧拉角转换为 Gazebo 使用的四元数。
- 第 52 行：`current_quaternion()`，根据当前姿态生成四元数。
- 第 61 行：`print_status()`，显示位置、姿态、速度和四元数。
- 第 81 行：`apply_pose()`，通过 Gazebo service 设置模型位置和姿态。
- 第 138 行：`apply_velocity()`，通过 Gazebo service 设置线速度。
- 第 180 行：`reset_pose()`，恢复默认位置、水平姿态和零速度。
- 第 190 行：`print_help()`，显示位置、姿态、速度设置、应用和复位命令。
- 第 211 行：`main()`，运行交互命令循环；`p` 支持同时设置位置、姿态和速度，`y` 应用当前值，`re` 复位模型。

### [自定义新增] `sitl_rc.py`

基于 UDP 的 SITL RC 控制工具，手动运行，不由固件调用。

- 第 11-28 行：初始化 16 路 RC 通道，默认设置姿态、油门和开关通道。
- 第 35 行：`print_status()`，显示全部 16 路 RC 通道值。
- 第 45 行：`command_thread()`，处理 `ch1` 至 `ch16`、`ch <number> <value>` 和 `ud<number>` 命令；`ud<number>` 将通道置高 0.5 秒后恢复低位。
- 第 167-181 行：组装 16 路 RC 数据包并以 50 Hz 发送到 SITL UDP 端口。

### [自定义新增] `tx16s_sitl_rc.py`

TX16S USB 摇杆到 SITL RC UDP 的桥接工具。

- 第 54 行：`print_edgetx_setup()`，显示 EdgeTX Extended 模式配置。
- 第 93 行：`axis_to_pwm()`，将摇杆轴值转换为 1000 至 2000 的 PWM。
- 第 101 行：`switch_to_pwm()`，将按钮组转换为二段或三段开关 PWM。
- 第 116 行：`read_channels()`，生成 16 路 RC 通道并报告无效开关编码。
- 第 136 行：`find_joystick()`，查找并初始化 USB 摇杆。
- 第 152 行：`validate_layout()`，检查 8 轴/22 按钮的 Extended HID 布局，拒绝会丢失三段开关中位的 Classic 模式。
- 第 177 行：`format_channels()`，格式化 RC 通道监视输出。
- 第 182 行：`diagnose()`，显示原始 HID 输入和解码结果。
- 第 213 行：`parse_args()`，解析主机、端口、设备、诊断、监视和配置打印参数。
- 第 225 行：`main()`，以 50 Hz 向 SITL 发送 16 路 RC 数据。

### [官方代码修改] Failsafe 单元测试

- `src/test/unit/flight_failsafe_unittest.cc:111`：增加 USER2 开关激活测试辅助函数。
- `src/test/unit/flight_failsafe_unittest.cc:727`：增加 `TestAutonomousLockAutoReleasesAfterDisarmWithUser2High`，验证 USER2 保持高电平时，failsafe 解锁并解除 ARM 后 synthetic RX 锁、RX_FAILSAFE 阻止标志和 failsafe 状态能够恢复。
- `src/test/unit/flight_failsafe_unittest.cc:953`、`:995`：增加有效飞行通道测试桩及 `rxAreFlightChannelsValid()` 实现。
- `src/test/unit/flight_failsafe_unittest.cc:256`：调整 failsafe landing 测试的时间推进，使其覆盖完整 RX 恢复周期。
