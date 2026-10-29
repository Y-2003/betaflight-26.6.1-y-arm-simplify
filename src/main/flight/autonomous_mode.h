#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "common/vector.h"
#include "common/time.h"

typedef enum {
    AUTONOMOUS_USER1_POSITION_LOW = 0,
    AUTONOMOUS_USER1_POSITION_MIDDLE,
    AUTONOMOUS_USER1_POSITION_HIGH,
    AUTONOMOUS_USER1_POSITION_INVALID,
} autonomousUser1Position_e;

typedef enum {
    AUTONOMOUS_MODE_IDLE = 0,
    AUTONOMOUS_MODE_WAITING_FOR_ARM = 1,
    AUTONOMOUS_MODE_ACCEL_WAITING = 2,
    AUTONOMOUS_MODE_REQUESTING_HOLD = 3, // 申请 ANGLE/ALTHOLD，确认 roll/pitch 绝对值均小于20°连续0.5s
    AUTONOMOUS_MODE_REQUESTING_POSITION_HOLD = 4, // 保持 ANGLE/ALTHOLD，再申请 POSHOLD
    AUTONOMOUS_MODE_HOLD = 5, // 三个模式及保持控制器已确认启用
    AUTONOMOUS_MODE_REQUESTING_AUTOPILOT = 6, // USER4 已开启，提交 AUTOPILOT 请求
    AUTONOMOUS_MODE_WAITING_FOR_AUTOPILOT = 7, // 模式处理已接收请求，等待内部标志
    AUTONOMOUS_MODE_AUTOPILOT = 8, // AUTOPILOT_MODE 已确认启用
} autonomousModeState_e;

// Result of the most recent autonomous arming attempt. These values are
// logged alongside autonomousState so a failed attempt can be diagnosed from
// the blackbox without relying on a beeper or live CLI output.
typedef enum {
    AUTONOMOUS_ARM_RESULT_NONE = 0,
    AUTONOMOUS_ARM_RESULT_SUCCESS,
    AUTONOMOUS_ARM_RESULT_BLOCKED,
    AUTONOMOUS_ARM_RESULT_DSHOT_GUARD,
} autonomousArmResult_e;

// [新增] 功能：进入自由落体检测阶段。
void autonomousModeStartAccelerationDetection(void);
// [新增] 功能：中止当前自主流程并恢复空闲状态。
void autonomousModeAbort(void);
// [新增] 功能：按周期推进自主飞行状态机。
void autonomousModeUpdate(timeUs_t currentTimeUs);

// [新增] 功能：查询自主流程对飞手输入和控制模式的接管状态。
bool autonomousModeSuppressesPilotInput(void);
bool autonomousModeOwnsArming(void);
// 自主流程已接管本次飞行，解锁后不再依赖飞手油门触发 throttleRaised。
bool autonomousModeRequestsThrottleRaised(void);
bool autonomousModeOverridesThrottle(void);
uint16_t autonomousModeGetThrottlePwm(void);
bool autonomousModeIsFlipActive(void);
// [新增] 功能：读取自主翻转阶段的角速度和油门输出。
float autonomousModeGetFlipRate(int axis);
uint16_t autonomousModeGetFlipThrottlePwm(void);
// [新增] 功能：查询自主流程请求的姿态、位置和高度控制模式。
bool autonomousModeRequestsAngle(void);
bool autonomousModeRequestsAltitudeHold(void);
bool autonomousModeRequestsPositionHold(void);
// USER4 在阶段 5 的有效上升沿进入 6；请求提交后进入 7，内部标志确认后进入 8。
bool autonomousModeRequestsAutopilot(void);
void autonomousModeOnAutopilotRequested(void);
void autonomousModeOnAutopilotActivated(void);
bool autonomousModeSuppressesPilotHoldModes(void);
// [新增] 功能：读取当前自主状态和 USER1 原始拨杆位置。
autonomousModeState_e autonomousModeGetState(void);
void autonomousModeRecordArmAttempt(bool success, uint32_t blockingFlags, autonomousArmResult_e result);
uint32_t autonomousModeGetArmBlockingFlags(void);
autonomousArmResult_e autonomousModeGetArmResult(void);
uint16_t autonomousModeGetArmAttemptCount(void);
// 兼容接口：简化流程停用自定义刹车，始终返回 false。
bool autonomousModeIsBraking(void);
autonomousUser1Position_e autonomousModeGetUser1Position(void);
// [兼容] USER1 不直接请求高度/位置保持；这两个接口始终返回 false。
bool autonomousModeUserRequestsAltitudeHold(void);
bool autonomousModeUserRequestsPositionHold(void);

// Compatibility hooks: custom braking is disabled; both return false.
bool autonomousModeGetAltitudeSetpoint(float *targetAltitudeCm, float *targetVelocityCmS);
bool autonomousModeGetPositionSetpoint(vector2_t *targetPosition, vector2_t *targetVelocity, bool *positionLocked, float *angleLimitScale);
