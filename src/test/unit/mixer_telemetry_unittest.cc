// Verify telemetry against the real mixer, including mode overrides and
// attitude desaturation. This test never writes to a motor device.
#include <cstring>

extern "C" {
#include "platform.h"
#include "build/debug.h"
#include "common/maths.h"
#include "config/feature.h"
#include "drivers/time.h"
#include "fc/controlrate_profile.h"
#include "fc/runtime_config.h"
#include "fc/rc_modes.h"
#include "flight/mixer.h"
#include "flight/mixer_init.h"
#include "flight/pid.h"
#include "fc/rc_controls.h"
#include "pg/pg_ids.h"
#include "pg/rx.h"
#include "rx/rx.h"
#include "sensors/gyro.h"
#include "io/gps.h"

PG_REGISTER(mixerConfig_t, mixerConfig, PG_MIXER_CONFIG, 0);
PG_REGISTER(flight3DConfig_t, flight3DConfig, PG_MOTOR_3D_CONFIG, 0);
mixerRuntime_t mixerRuntime;
gyro_t gyro;
gpsSolutionData_t gpsSol;
uint8_t armingFlags;
uint16_t flightModeFlags;
uint8_t stateFlags;
int16_t debug[DEBUG16_VALUE_COUNT];
uint8_t debugMode;
float rcCommand[4];
float rcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];
pidAxisData_t pidData[XYZ_AXIS_COUNT];
static pidProfile_t profile;
pidProfile_t *currentPidProfile = &profile;
static controlRateConfig_t rates;
controlRateConfig_t *currentControlRateProfile = &rates;
static timeUs_t nowUs;
static float holdThrottle;
static bool motorStop;
static bool airmode;
timeUs_t micros(void) { return nowUs; }
void delay(timeMs_t) { }
bool featureIsEnabled(uint32_t mask) { return motorStop && (mask & FEATURE_MOTOR_STOP); }
bool isAirmodeEnabled(void) { return airmode; }
bool IS_RC_MODE_ACTIVE(boxId_e) { return false; }
bool isMotorsReversed(void) { return false; }
bool sensors(uint32_t) { return false; }
float getCosTiltAngle(void) { return 1; }
bool mixerIsTricopter(void) { return false; }
float mixerTricopterMotorCorrection(int) { return 0; }
void parseRcChannels(const char *, rxConfig_t *) { }
bool isLaunchControlActive(void) { return false; }
bool isCrashFlipModeActive(void) { return false; }
bool autonomousModeIsFlipActive(void) { return false; }
bool failsafeIsActive(void) { return false; }
float getAutopilotThrottle(void) { return holdThrottle; }
void pidUpdateAntiGravityThrottleFilter(float) { }
void pidUpdateTpaFactor(float) { }
void pidResetIterm(void) { }
void motorWriteAll(float *) { }
float getRcDeflection(int) { return 0; }
float getRcDeflectionAbs(int) { return 0; }
float getMaxRcDeflectionAbs(void) { return 0; }
}

#include "gtest/gtest.h"

class MixerTelemetryTest : public ::testing::Test {
protected:
    void SetUp() override {
        memset(&mixerRuntime, 0, sizeof(mixerRuntime));
        memset(&profile, 0, sizeof(profile));
        memset(&rates, 0, sizeof(rates));
        memset(pidData, 0, sizeof(pidData));
        memset(rcCommand, 0, sizeof(rcCommand));
        armingFlags = ARMED;
        flightModeFlags = ALT_HOLD_MODE | POS_HOLD_MODE;
        motorStop = false;
        airmode = true;
        nowUs = 1000000;
        rcCommand[THROTTLE] = 1240;
        rxConfigMutable()->mincheck = 1050;
        holdThrottle = 0.7f;
        profile.pidSumLimit = 1000;
        profile.pidSumLimitYaw = 1000;
        mixerConfigMutable()->mixer_type = MIXER_LEGACY;
        mixerRuntime.motorCount = 4;
        mixerRuntime.motorOutputLow = 1000;
        mixerRuntime.motorOutputHigh = 2000;
        mixerRuntime.disarmMotorOutput = 900;
        for (unsigned i = 0; i < 4; i++) {
            mixerRuntime.currentMixer[i].throttle = 1;
            mixerRuntime.currentMixer[i].roll = (i & 1) ? 1 : -1;
            motor_disarmed[i] = 900;
        }
    }
};

TEST_F(MixerTelemetryTest, IncludesHoldOverrideAndAttitudeDesaturation)
{
    // +/-0.5 roll correction leaves exactly 0.5 collective, even though the
    // input is 0.24 and altitude hold requests 0.7.
    pidData[FD_ROLL].Sum = 500;
    mixTable(nowUs);
    uint16_t pwm;
    timeUs_t sampleTime;
    ASSERT_TRUE(mixerGetFinalThrottle(&pwm, &sampleTime));
    EXPECT_EQ(pwm, 1500);
    EXPECT_EQ(sampleTime, nowUs);
    EXPECT_NEAR(mixerGetThrottle(), 0.24f, 0.00001f);
    EXPECT_FLOAT_EQ(motor[0], 1000);
    EXPECT_FLOAT_EQ(motor[1], 2000);

    pidData[FD_ROLL].Sum = 0;
    holdThrottle = 0.3f;
    mixTable(++nowUs);
    ASSERT_TRUE(mixerGetFinalThrottle(&pwm, &sampleTime));
    EXPECT_EQ(pwm, 1300);
}

TEST_F(MixerTelemetryTest, ReportsDisarmedAndRejectsOldOr3dSamples)
{
    armingFlags = 0;
    mixTable(nowUs);
    uint16_t pwm;
    timeUs_t sampleTime;
    ASSERT_TRUE(mixerGetFinalThrottle(&pwm, &sampleTime));
    EXPECT_EQ(pwm, 1000);
    EXPECT_FLOAT_EQ(motor[0], 900);
    nowUs += 100000;
    EXPECT_FALSE(mixerGetFinalThrottle(&pwm, &sampleTime));
    mixTable(nowUs);
    mixerRuntime.feature3dEnabled = true;
    EXPECT_FALSE(mixerGetFinalThrottle(&pwm, &sampleTime));
}

TEST_F(MixerTelemetryTest, MotorStopAndExplicitStopReportZeroCollective)
{
    motorStop = true;
    airmode = false;
    flightModeFlags = 0;
    rcData[THROTTLE] = 1000;
    mixTable(nowUs);
    uint16_t pwm;
    timeUs_t sampleTime;
    ASSERT_TRUE(mixerGetFinalThrottle(&pwm, &sampleTime));
    EXPECT_EQ(pwm, 1000);
    EXPECT_FLOAT_EQ(motor[0], 900);

    motorStop = false;
    airmode = true;
    mixTable(++nowUs);
    ASSERT_TRUE(mixerGetFinalThrottle(&pwm, &sampleTime));
    EXPECT_GT(pwm, 1000);
    stopMotors();
    ASSERT_TRUE(mixerGetFinalThrottle(&pwm, &sampleTime));
    EXPECT_EQ(pwm, 1000);
}
