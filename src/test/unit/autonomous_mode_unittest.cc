#include <cstring>

extern "C" {
#include "platform.h"
#include "build/debug.h"
#include "fc/core.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"
#include "flight/autonomous_mode.h"
#include "flight/failsafe.h"
#include "flight/imu.h"
#include "rx/rx.h"
#include "sensors/acceleration.h"
#include "pg/pg_ids.h"

PG_REGISTER_ARRAY(modeActivationCondition_t, MAX_MODE_ACTIVATION_CONDITION_COUNT,
    modeActivationConditions, PG_MODE_ACTIVATION_PROFILE, 5);

uint8_t armingFlags;
uint16_t flightModeFlags;
float rcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];
acc_t acc;
attitudeEulerAngles_t attitude;
int16_t debug[DEBUG16_VALUE_COUNT];
uint8_t debugMode;

static bool modes[CHECKBOX_ITEM_COUNT];
static bool failsafeActive, rxValid, rxReceiving, altActive, posActive, armAllowed;
static armingDisableFlags_e disableFlags;
static unsigned armAttempts;

bool IS_RC_MODE_ACTIVE(boxId_e id) { return modes[id]; }
bool failsafeIsActive(void) { return failsafeActive; }
bool failsafeIsReceivingRxData(void) { return rxReceiving; }
void failsafeRequestAutonomousLock(bool) {}
bool rxAreFlightChannelsValid(void) { return rxValid; }
bool isAltHoldActive(void) { return altActive; }
bool isAutopilotInControl(void) { return posActive; }
uint16_t autopilotGetEffectiveHoverThrottlePwm(void) { return 1500; }
armingDisableFlags_e getArmingDisableFlags(void) { return disableFlags; }
void unsetArmingDisabled(armingDisableFlags_e flag) {
    disableFlags = (armingDisableFlags_e)(disableFlags & ~flag);
}
bool tryArmAutonomous(void) {
    ++armAttempts;
    if (armAllowed) {
        ENABLE_ARMING_FLAG(ARMED);
    }
    return armAllowed;
}
}

#include "gtest/gtest.h"

class AutonomousModeTest : public ::testing::Test {
protected:
    timeUs_t now;
    void tick(unsigned dt = 1000) { now += dt; autonomousModeUpdate(now); }
    void SetUp() override {
        memset(modes, 0, sizeof(modes));
        memset(rcData, 0, sizeof(rcData));
        memset(&acc, 0, sizeof(acc));
        memset(&attitude, 0, sizeof(attitude));
        memset(modeActivationConditionsMutable(0), 0,
            sizeof(modeActivationCondition_t) * MAX_MODE_ACTIVATION_CONDITION_COUNT);
        auto *user1 = modeActivationConditionsMutable(0);
        user1->modeId = BOXUSER1;
        user1->auxChannelIndex = 0;
        user1->range.startStep = 0;
        user1->range.endStep = 48;
        rcData[NON_AUX_CHANNEL_COUNT] = 1500;
        armingFlags = 0;
        flightModeFlags = 0;
        failsafeActive = false;
        rxValid = rxReceiving = armAllowed = true;
        altActive = posActive = false;
        disableFlags = (armingDisableFlags_e)0;
        armAttempts = 0;
        now = 100000;
        autonomousModeAbort();
        tick(); // Synchronize all switch edge tracking to low.
        acc.isAccelUpdatedAtLeastOnce = true;
        acc.accMagnitude = 1.0f;
    }
    void enterFreefallWait() {
        modes[BOXUSER3] = modes[BOXARM] = true;
        tick();
        ASSERT_EQ(AUTONOMOUS_MODE_WAITING_FOR_ARM, autonomousModeGetState());
        tick(199999);
        ASSERT_EQ(AUTONOMOUS_MODE_WAITING_FOR_ARM, autonomousModeGetState());
        tick(1);
        ASSERT_EQ(AUTONOMOUS_MODE_ACCEL_WAITING, autonomousModeGetState());
    }
    void enterStage3() {
        enterFreefallWait();
        acc.accMagnitude = 0.29f;
        tick();
        tick(29999);
        ASSERT_FALSE(ARMING_FLAG(ARMED));
        tick(1);
        ASSERT_TRUE(ARMING_FLAG(ARMED));
        ASSERT_EQ(AUTONOMOUS_MODE_REQUESTING_HOLD, autonomousModeGetState());
    }
    void confirmLevelAttitude() {
        tick();
#if !defined(AUTONOMOUS_STATE3_LEVEL_CHECK_ENABLED) || AUTONOMOUS_STATE3_LEVEL_CHECK_ENABLED
        tick(499999);
        ASSERT_EQ(AUTONOMOUS_MODE_REQUESTING_HOLD, autonomousModeGetState());
        ASSERT_FALSE(autonomousModeRequestsPositionHold());
        tick(1);
#endif
        ASSERT_EQ(AUTONOMOUS_MODE_REQUESTING_POSITION_HOLD, autonomousModeGetState());
    }
    void confirmHold() {
        flightModeFlags = ANGLE_MODE | ALT_HOLD_MODE | POS_HOLD_MODE;
        altActive = posActive = true;
        confirmLevelAttitude();
        tick();
        ASSERT_EQ(AUTONOMOUS_MODE_HOLD, autonomousModeGetState());
    }
    void requestMission() {
        modes[BOXUSER4] = true;
        tick();
        ASSERT_EQ(AUTONOMOUS_MODE_REQUESTING_AUTOPILOT, autonomousModeGetState());
        autonomousModeOnAutopilotRequested();
        ASSERT_EQ(AUTONOMOUS_MODE_WAITING_FOR_AUTOPILOT, autonomousModeGetState());
    }
};

TEST_F(AutonomousModeTest, StageNumbersAreStable) {
    EXPECT_EQ(3, AUTONOMOUS_MODE_REQUESTING_HOLD);
    EXPECT_EQ(4, AUTONOMOUS_MODE_REQUESTING_POSITION_HOLD);
    EXPECT_EQ(5, AUTONOMOUS_MODE_HOLD);
    EXPECT_EQ(6, AUTONOMOUS_MODE_REQUESTING_AUTOPILOT);
    EXPECT_EQ(7, AUTONOMOUS_MODE_WAITING_FOR_AUTOPILOT);
    EXPECT_EQ(8, AUTONOMOUS_MODE_AUTOPILOT);
}

TEST_F(AutonomousModeTest, HoldWaitsForUser4ThenRequestSubmissionIsSeparate) {
    enterStage3();
    confirmHold();
    autonomousModeOnAutopilotRequested();
    tick(5000000);
    EXPECT_EQ(AUTONOMOUS_MODE_HOLD, autonomousModeGetState());
    EXPECT_FALSE(autonomousModeRequestsAutopilot());
    modes[BOXUSER4] = true;
    tick();
    EXPECT_EQ(AUTONOMOUS_MODE_REQUESTING_AUTOPILOT, autonomousModeGetState());
    EXPECT_TRUE(autonomousModeRequestsAutopilot());
    autonomousModeOnAutopilotActivated();
    tick();
    EXPECT_EQ(AUTONOMOUS_MODE_REQUESTING_AUTOPILOT, autonomousModeGetState());
    autonomousModeOnAutopilotRequested();
    EXPECT_EQ(AUTONOMOUS_MODE_WAITING_FOR_AUTOPILOT, autonomousModeGetState());
    tick(2000000);
    EXPECT_EQ(AUTONOMOUS_MODE_WAITING_FOR_AUTOPILOT, autonomousModeGetState());
    flightModeFlags |= AUTOPILOT_MODE;
    autonomousModeOnAutopilotActivated();
    EXPECT_EQ(AUTONOMOUS_MODE_AUTOPILOT, autonomousModeGetState());
}

TEST_F(AutonomousModeTest, User4OffCancelsBeforeRequestSubmission) {
    enterStage3();
    confirmHold();
    modes[BOXUSER4] = true;
    tick();
    EXPECT_EQ(AUTONOMOUS_MODE_REQUESTING_AUTOPILOT, autonomousModeGetState());
    modes[BOXUSER4] = false;
    autonomousModeOnAutopilotRequested();
    EXPECT_EQ(AUTONOMOUS_MODE_REQUESTING_AUTOPILOT, autonomousModeGetState());
    tick();
    EXPECT_EQ(AUTONOMOUS_MODE_HOLD, autonomousModeGetState());
    EXPECT_FALSE(autonomousModeRequestsAutopilot());
}

TEST_F(AutonomousModeTest, AngleAndAltitudeConfirmBeforePositionIsRequested) {
    enterStage3();
    EXPECT_TRUE(autonomousModeRequestsAngle());
    EXPECT_TRUE(autonomousModeRequestsAltitudeHold());
    EXPECT_FALSE(autonomousModeRequestsPositionHold());
    EXPECT_TRUE(autonomousModeSuppressesPilotInput());
    EXPECT_TRUE(autonomousModeOverridesThrottle());
    const uint16_t flags = ANGLE_MODE | ALT_HOLD_MODE;
    altActive = true;
    for (uint16_t missing : {ANGLE_MODE, ALT_HOLD_MODE}) {
        flightModeFlags = flags & ~missing;
        tick();
        EXPECT_EQ(AUTONOMOUS_MODE_REQUESTING_HOLD, autonomousModeGetState());
        EXPECT_FALSE(autonomousModeRequestsPositionHold());
    }
    flightModeFlags = flags;
    altActive = false;
    tick();
    EXPECT_EQ(AUTONOMOUS_MODE_REQUESTING_HOLD, autonomousModeGetState());
    altActive = true;
    confirmLevelAttitude();
    EXPECT_EQ(AUTONOMOUS_MODE_REQUESTING_POSITION_HOLD, autonomousModeGetState());
    EXPECT_TRUE(autonomousModeRequestsAngle());
    EXPECT_TRUE(autonomousModeRequestsAltitudeHold());
    EXPECT_TRUE(autonomousModeRequestsPositionHold());
    EXPECT_TRUE(autonomousModeSuppressesPilotInput());
    EXPECT_TRUE(autonomousModeOverridesThrottle());
    tick(2000000);
    EXPECT_EQ(AUTONOMOUS_MODE_REQUESTING_POSITION_HOLD, autonomousModeGetState());
    posActive = true;
    tick(); // Controller alone cannot confirm without POS_HOLD_MODE.
    EXPECT_EQ(AUTONOMOUS_MODE_REQUESTING_POSITION_HOLD, autonomousModeGetState());
    flightModeFlags |= POS_HOLD_MODE;
    posActive = false;
    tick(); // Mode flag alone cannot confirm without its controller.
    EXPECT_EQ(AUTONOMOUS_MODE_REQUESTING_POSITION_HOLD, autonomousModeGetState());
    posActive = true;
    tick();
    EXPECT_EQ(AUTONOMOUS_MODE_HOLD, autonomousModeGetState());
}

TEST_F(AutonomousModeTest, PositionStageReturnsToAngleAndAltitudeOnLoss) {
    enterStage3();
    const uint16_t flags = ANGLE_MODE | ALT_HOLD_MODE | POS_HOLD_MODE;
    flightModeFlags = flags;
    altActive = posActive = true;
    for (uint16_t missing : {ANGLE_MODE, ALT_HOLD_MODE}) {
        confirmLevelAttitude();
        ASSERT_EQ(AUTONOMOUS_MODE_REQUESTING_POSITION_HOLD, autonomousModeGetState());
        flightModeFlags = flags & ~missing;
        tick();
        EXPECT_EQ(AUTONOMOUS_MODE_REQUESTING_HOLD, autonomousModeGetState());
        EXPECT_FALSE(autonomousModeRequestsPositionHold());
        flightModeFlags = flags;
    }
    confirmLevelAttitude();
    ASSERT_EQ(AUTONOMOUS_MODE_REQUESTING_POSITION_HOLD, autonomousModeGetState());
    altActive = false;
    tick();
    EXPECT_EQ(AUTONOMOUS_MODE_REQUESTING_HOLD, autonomousModeGetState());
    EXPECT_FALSE(autonomousModeRequestsPositionHold());
    confirmHold();
}

#if !defined(AUTONOMOUS_STATE3_LEVEL_CHECK_ENABLED) || AUTONOMOUS_STATE3_LEVEL_CHECK_ENABLED
TEST_F(AutonomousModeTest, Stage3RequiresBothAnglesStrictlyBelow20Degrees) {
    enterStage3();
    flightModeFlags = ANGLE_MODE | ALT_HOLD_MODE;
    altActive = true;
    const int16_t invalidAngles[][2] = {
        {200, 0}, {-200, 0}, {201, 0}, {-201, 0},
        {0, 200}, {0, -200}, {0, 201}, {0, -201},
    };
    for (const auto &angles : invalidAngles) {
        attitude.values.roll = angles[0];
        attitude.values.pitch = angles[1];
        tick();
        tick(1000000);
        EXPECT_EQ(AUTONOMOUS_MODE_REQUESTING_HOLD, autonomousModeGetState());
        EXPECT_FALSE(autonomousModeRequestsPositionHold());
    }
    attitude.values.roll = 199;
    attitude.values.pitch = -199;
    confirmLevelAttitude();
}

TEST_F(AutonomousModeTest, EitherAxisExcursionRestartsLevelConfirmation) {
    enterStage3();
    flightModeFlags = ANGLE_MODE | ALT_HOLD_MODE;
    altActive = true;
    for (int axis : {FD_ROLL, FD_PITCH}) {
        tick();
        tick(400000);
        ASSERT_EQ(AUTONOMOUS_MODE_REQUESTING_HOLD, autonomousModeGetState());
        attitude.raw[axis] = -200;
        tick();
        attitude.raw[axis] = 0;
        tick();
        tick(499999);
        EXPECT_EQ(AUTONOMOUS_MODE_REQUESTING_HOLD, autonomousModeGetState());
        EXPECT_FALSE(autonomousModeRequestsPositionHold());
        // Interrupt again before trying the other axis or the final full interval.
        attitude.raw[axis] = 200;
        tick();
        attitude.raw[axis] = 0;
    }
    confirmLevelAttitude();
}

TEST_F(AutonomousModeTest, HoldLossRestartsLevelConfirmation) {
    enterStage3();
    const uint16_t flags = ANGLE_MODE | ALT_HOLD_MODE;
    flightModeFlags = flags;
    altActive = true;
    for (int loss = 0; loss < 3; loss++) {
        tick();
        tick(400000);
        if (loss == 0) {
            flightModeFlags &= ~ANGLE_MODE;
        } else if (loss == 1) {
            flightModeFlags &= ~ALT_HOLD_MODE;
        } else {
            altActive = false;
        }
        tick();
        tick(1000000);
        EXPECT_EQ(AUTONOMOUS_MODE_REQUESTING_HOLD, autonomousModeGetState());
        flightModeFlags = flags;
        altActive = true;
    }
    confirmLevelAttitude();
}

TEST_F(AutonomousModeTest, ReturningToStage3RequiresFreshHalfSecond) {
    enterStage3();
    confirmHold();
    tick(1000000);
    posActive = false;
    tick();
    ASSERT_EQ(AUTONOMOUS_MODE_REQUESTING_HOLD, autonomousModeGetState());
    confirmHold();
}

TEST_F(AutonomousModeTest, AbortClearsPartialLevelConfirmation) {
    enterStage3();
    flightModeFlags = ANGLE_MODE | ALT_HOLD_MODE;
    altActive = true;
    tick();
    tick(400000);
    autonomousModeAbort();
    armingFlags = 0;
    modes[BOXARM] = false;
    tick();
    enterStage3();
    confirmLevelAttitude();
}

TEST_F(AutonomousModeTest, LevelConfirmationHandlesClockWrap) {
    enterStage3();
    flightModeFlags = ANGLE_MODE | ALT_HOLD_MODE;
    altActive = true;
    now = TIMEUS_MAX - 250000;
    confirmLevelAttitude();
}
#else
TEST_F(AutonomousModeTest, DisabledLevelCheckSkipsAnglesAndDelayButRequiresHold) {
    enterStage3();
    attitude.values.roll = 900;
    attitude.values.pitch = -900;
    const uint16_t flags = ANGLE_MODE | ALT_HOLD_MODE;
    altActive = true;
    for (uint16_t missing : {ANGLE_MODE, ALT_HOLD_MODE}) {
        flightModeFlags = flags & ~missing;
        tick(1000000);
        EXPECT_EQ(AUTONOMOUS_MODE_REQUESTING_HOLD, autonomousModeGetState());
        EXPECT_FALSE(autonomousModeRequestsPositionHold());
    }
    flightModeFlags = flags;
    altActive = false;
    tick(1000000);
    EXPECT_EQ(AUTONOMOUS_MODE_REQUESTING_HOLD, autonomousModeGetState());
    altActive = true;
    tick(1);
    EXPECT_EQ(AUTONOMOUS_MODE_REQUESTING_POSITION_HOLD, autonomousModeGetState());
    EXPECT_TRUE(autonomousModeRequestsPositionHold());
}
#endif

TEST_F(AutonomousModeTest, User4DuringPositionConfirmationNeedsNewEdge) {
    enterStage3();
    flightModeFlags = ANGLE_MODE | ALT_HOLD_MODE;
    altActive = true;
    confirmLevelAttitude();
    ASSERT_EQ(AUTONOMOUS_MODE_REQUESTING_POSITION_HOLD, autonomousModeGetState());
    modes[BOXUSER4] = true;
    tick();
    autonomousModeOnAutopilotRequested();
    flightModeFlags |= AUTOPILOT_MODE;
    autonomousModeOnAutopilotActivated();
    EXPECT_EQ(AUTONOMOUS_MODE_REQUESTING_POSITION_HOLD, autonomousModeGetState());
    EXPECT_FALSE(autonomousModeRequestsAutopilot());
    flightModeFlags = ANGLE_MODE | ALT_HOLD_MODE | POS_HOLD_MODE;
    posActive = true;
    tick();
    tick();
    EXPECT_EQ(AUTONOMOUS_MODE_HOLD, autonomousModeGetState());
    EXPECT_FALSE(autonomousModeRequestsAutopilot());
    modes[BOXUSER4] = false;
    tick();
    requestMission();
}

TEST_F(AutonomousModeTest, EarlyUser4CannotArmOrQueueAutopilot) {
    modes[BOXUSER4] = true;
    tick();
    EXPECT_EQ(AUTONOMOUS_MODE_IDLE, autonomousModeGetState());
    enterFreefallWait();
    tick(2000000);
    EXPECT_EQ(0u, armAttempts);
    acc.accMagnitude = 0.80f;
    tick(200000);
    EXPECT_EQ(0u, armAttempts);
    acc.accMagnitude = 0.1f;
    tick();
    tick(100000);
    confirmHold();
    tick();
    EXPECT_EQ(AUTONOMOUS_MODE_HOLD, autonomousModeGetState());
    EXPECT_FALSE(autonomousModeRequestsAutopilot());
    modes[BOXUSER4] = false;
    tick();
    requestMission();
    EXPECT_TRUE(autonomousModeRequestsAutopilot());
}

TEST_F(AutonomousModeTest, RequestWaitsForActualAutopilotFlag) {
    enterStage3();
    confirmHold();
    requestMission();
    autonomousModeOnAutopilotActivated();
    tick(2000000);
    EXPECT_EQ(AUTONOMOUS_MODE_WAITING_FOR_AUTOPILOT, autonomousModeGetState());
    EXPECT_TRUE(autonomousModeRequestsAltitudeHold());
    EXPECT_TRUE(autonomousModeRequestsPositionHold());
    EXPECT_TRUE(autonomousModeOverridesThrottle());
    flightModeFlags |= AUTOPILOT_MODE;
    autonomousModeOnAutopilotActivated();
    EXPECT_EQ(AUTONOMOUS_MODE_AUTOPILOT, autonomousModeGetState());
    EXPECT_TRUE(autonomousModeRequestsAutopilot());
    EXPECT_FALSE(autonomousModeSuppressesPilotInput());
    EXPECT_FALSE(autonomousModeOverridesThrottle());
    EXPECT_FALSE(autonomousModeRequestsAltitudeHold());
    EXPECT_FALSE(autonomousModeRequestsPositionHold());
    EXPECT_TRUE(autonomousModeOwnsArming());
}

TEST_F(AutonomousModeTest, LegacyControlHooksAreDisabled) {
    enterStage3();
    confirmHold();
    requestMission();
    EXPECT_FALSE(autonomousModeIsFlipActive());
    EXPECT_FALSE(autonomousModeIsBraking());
    EXPECT_FLOAT_EQ(0.0f, autonomousModeGetFlipRate(0));
    EXPECT_FALSE(autonomousModeGetAltitudeSetpoint(nullptr, nullptr));
    EXPECT_FALSE(autonomousModeGetPositionSetpoint(nullptr, nullptr, nullptr, nullptr));
    EXPECT_EQ(1500, autonomousModeGetThrottlePwm());
}

TEST_F(AutonomousModeTest, CancelOrControllerLossRequiresNewUser4Edge) {
    enterStage3();
    confirmHold();
    requestMission();
    modes[BOXUSER4] = false;
    tick();
    EXPECT_EQ(AUTONOMOUS_MODE_HOLD, autonomousModeGetState());
    requestMission();
    posActive = false;
    tick();
    EXPECT_EQ(AUTONOMOUS_MODE_REQUESTING_HOLD, autonomousModeGetState());
    confirmHold();
    tick();
    EXPECT_FALSE(autonomousModeRequestsAutopilot());
    modes[BOXUSER4] = false;
    tick();
    requestMission();
    flightModeFlags |= AUTOPILOT_MODE;
    tick();
    EXPECT_EQ(AUTONOMOUS_MODE_AUTOPILOT, autonomousModeGetState());
    flightModeFlags &= ~AUTOPILOT_MODE;
    tick();
    EXPECT_EQ(AUTONOMOUS_MODE_REQUESTING_HOLD, autonomousModeGetState());
    confirmHold();
    tick();
    EXPECT_FALSE(autonomousModeRequestsAutopilot());
}

TEST_F(AutonomousModeTest, ArmPermissionAndUser1ExitRemainEffective) {
    enterStage3();
    confirmHold();
    modes[BOXARM] = false;
    tick();
    EXPECT_EQ(AUTONOMOUS_MODE_IDLE, autonomousModeGetState());
    EXPECT_FALSE(autonomousModeOwnsArming());
    armingFlags = 0;
    flightModeFlags = 0;
    altActive = posActive = false;
    modes[BOXUSER3] = false;
    tick();
    enterStage3();
    rcData[NON_AUX_CHANNEL_COUNT] = 1900;
    tick();
    tick(29999);
    EXPECT_EQ(AUTONOMOUS_MODE_REQUESTING_HOLD, autonomousModeGetState());
    tick(1);
    EXPECT_EQ(AUTONOMOUS_MODE_IDLE, autonomousModeGetState());
}

TEST_F(AutonomousModeTest, FailsafeAbortsHold) {
    enterStage3();
    confirmHold();
    failsafeActive = true;
    tick();
    EXPECT_EQ(AUTONOMOUS_MODE_IDLE, autonomousModeGetState());
    EXPECT_FALSE(autonomousModeRequestsAutopilot());
}

TEST_F(AutonomousModeTest, User2PreservesRecoveryDelayAndFreefall) {
    modes[BOXARM] = true;
    tick();
    modes[BOXUSER2] = true;
    failsafeActive = true;
    tick();
    EXPECT_EQ(AUTONOMOUS_MODE_WAITING_FOR_ARM, autonomousModeGetState());
    tick(1000000);
    EXPECT_EQ(0u, armAttempts);
    failsafeActive = false;
    rxReceiving = false;
    tick(1000000);
    EXPECT_EQ(AUTONOMOUS_MODE_WAITING_FOR_ARM, autonomousModeGetState());
    rxReceiving = true;
    tick();
    tick(199999);
    EXPECT_EQ(AUTONOMOUS_MODE_WAITING_FOR_ARM, autonomousModeGetState());
    tick(1);
    EXPECT_EQ(AUTONOMOUS_MODE_ACCEL_WAITING, autonomousModeGetState());
    acc.accMagnitude = 0.1f;
    tick();
    tick(100000);
    EXPECT_EQ(AUTONOMOUS_MODE_REQUESTING_HOLD, autonomousModeGetState());
}
