# simplify 自主流程排障（2026-10-22）

本文件对应 `betaflight-26.6.1-y-arm-simplify` 当前代码。旧日志须按生成固件时的枚举解释；本次将 USER4 开启和 AUTOPILOT 请求提交拆开，后续编号顺延。

## 当前阶段

| state | 含义 | 进入下一阶段 |
| ---: | --- | --- |
| 0 | 空闲 | 原有 USER2 上升沿，或 USER2 低、USER3 高时 ARM 上升沿；ARM 许可及 USER1 位置须有效 |
| 1 | 等待 failsafe/RX 恢复和延时 | RX 可用、failsafe 恢复后等待 200 ms |
| 2 | 自由落体检测/自主解锁 | 加速度小于 0.30 G 持续 100 ms，且自主解锁成功 |
| 3 | 同时申请 ANGLE、ALTHOLD、POSHOLD | 三个内部标志均置位，且定高、定点控制器实际生效 |
| 4 | 三模式已确认、标准保持并等待 USER4 | 不自动推进；此时新开启 USER4，RX/保持控制有效后进入 5 |
| 5 | USER4 已开启、提交 AUTOPILOT 请求 | core.c 模式处理接收请求后进入 6 |
| 6 | 请求已提交、等待内部确认 | 内部 AUTOPILOT_MODE 标志实际置位后进入 7 |
| 7 | AUTOPILOT 已启用 | 航线接管控制 |

阶段 3 不再要求 20°/1 s 姿态计时。旧翻转、自定义 XY/Z 刹车、READY 及姿态重入已用 `#if 0` 停用。保持由标准 ALTHOLD/POSHOLD 完成；阶段 2 至 6 使用自主输入屏蔽及悬停油门基准，阶段 7 释放给航线控制。

## USER4

配置器 Modes 中为 USER4 分配 AUX 开关（永久模式编号 43）。USER4 不再提供阶段 1/2 提前解锁入口。

必须在阶段 4 新开启 USER4。提前打开后一直保持，不会在到达 4 时自动申请；需关闭再开启。阶段 4 确认三模式后会持续等待；有效 USER4 上升沿使状态进入 5，开始提交 AUTOPILOT 请求；模式处理接收请求后进入 6，实际置位 AUTOPILOT_MODE 后才进入 7。阶段 5/6/7 分别表示提交、等待内部确认、实际启用。

阶段 5/6 关闭 USER4 回到 4；保持控制失效回到 3。阶段 7 关闭 USER4 或丢失 AUTOPILOT 标志回到 3，重新确认后回到 4。再次申请须新的 USER4 上升沿。

自主流程中原始 BOXAUTOPILOT 不得提前接管；正常飞手流程与 failsafe/rescue 的控制保留。

## OSD 接管确认

自主状态第二行 `A H P T` 中，`A` 表示 ANGLE_MODE 标志；`H` 仅在 ALT_HOLD_MODE 已置位且 `isAltHoldActive()` 确认定高控制器已激活后显示 1；`P` 仅在 POS_HOLD_MODE 已置位且 `isAutopilotInControl()` 确认定点控制器已启用、传感器有效、控制状态正常后显示 1。H/P 实时显示当前接管状态，模式退出或对应接管确认失效后回到 0。`P1` 不表示航线 AUTOPILOT_MODE 已启用，航线启用仍看阶段 7。

## 等待原因

- 卡 1：检查 RX/failsafe 是否恢复、ARM 是否许可；200 ms 从 RX 可用且恢复完成后开始。
- 卡 2：检查加速度数据已更新，是否连续满足小于 0.30 G/100 ms，以及自主解锁阻止原因。USER4 不绕过检测。
- 卡 3：分别检查 ANGLE_MODE、ALT_HOLD_MODE、POS_HOLD_MODE。仅申请开关不够，还须 `isAltHoldActive()` 与 `isAutopilotInControl()` 返回 true；后者是定点控制器状态，不是航线 AUTOPILOT 状态。检查高度、XY 估计、航向及定点控制器健康状态。
- 卡 4：检查 USER4 是否已提前保持高电平，是否需要先关后开，以及 RX/保持状态是否有效。
- 卡 5：检查 USER4 是否仍有效、RX 是否有效，以及 `core.c` 模式处理是否接收自主请求。请求接收后进入 6，此时尚不能断言实际模式已启用。
- 卡 6：检查 ACC、GPS 传感器、GPS_FIX、高度可用、已解锁及 throttleRaised；这些条件通过后 `core.c` 才会实际启用 AUTOPILOT_MODE。本版不以航点移动或任务完成作为阶段 7 条件。

自主解锁路径屏蔽 ANGLE、THROTTLE、ARM_SWITCH、ALTHOLD、POSHOLD、AUTOPILOT 禁止标志；其他原有解锁保护及 DShot 检查保留。普通解锁路径不变。

## 退出及记录

ARM 许可关闭、USER1 位置持续变化 30 ms、正常解除 ARM 或 failsafe 按原逻辑中止自主流程。阶段 1 保留等待 failsafe 恢复的原有例外。

黑盒 `autonomousState` 和 `DEBUG_FAILSAFE` 的 `debug[4]` 记录当前阶段。`autonomousStatus` bit8 表示 USER4 激活；`autonomousArmAttempts`、`autonomousArmResult`、`autonomousArmBlockingFlags` 保留解锁诊断，ARM_RESULT=3 表示 DShot 等待保护。

自由落体阈值 0.30 G、确认 100 ms、RX 恢复后延时 200 ms、USER1 防抖 30 ms 均保留修改前当前值。

## 验证入口

```sh
make -C src/test test_autonomous_mode_unittest
make TARGET=SITL -j4
python3 src/test/sitl/autonomous_user4.py --binary obj/main/betaflight_SITL.elf
make -j4 CONFIG=GEPRCF722_BT_HD_V3
```

离线测试直接驱动实际 C 状态机，覆盖模式标志/控制器逐项确认、USER4 提前开启、阶段 4 等待 USER4、阶段 5 提交请求与阶段 6 等待实际标志、模式丢失及取消、旧控制接口停用、ARM/USER1/failsafe 退出与 USER2 恢复延时。SITL 集成测试另外检查航线移动和 USER4 取消重试，运行需要本地 UDP/TCP socket 权限。构建和软件测试不代表实机验证完成。
