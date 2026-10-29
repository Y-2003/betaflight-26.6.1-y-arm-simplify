/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Betaflight. If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "common/axis.h"
#include "common/vector.h"
#include "common/time.h"


#ifndef USE_WING

#define AP_HOVER_THROTTLE_DEFAULT 1275U

// Samples of the values actually consumed by the control loops, before logging
// or OSD rounding. Positions are cm, velocities cm/s, in ENU (Up positive).
#define AUTOPILOT_TELEMETRY_TIMEOUT_US 100000
typedef struct {
    bool valid;
    timeUs_t timeUs;
    float targetAltitudeCm;
    float altitudeCm;
    float targetVelocityCmS;
    float velocityCmS;
} altitudeControlTelemetry_t;

typedef struct {
    bool valid;
    bool positionAnchored; // false: distanceError is integrated velocity error
    timeUs_t timeUs;
    vector2_t distanceErrorCm;
    vector2_t targetVelocityCmS;
    vector2_t velocityCmS;
} positionControlTelemetry_t;

bool autopilotGetAltitudeTelemetry(altitudeControlTelemetry_t *sample);
bool autopilotGetPositionTelemetry(positionControlTelemetry_t *sample);

extern float autopilotAngle[RP_AXIS_COUNT]; // NOTE: ANGLES ARE IN CENTIDEGREES

void autopilotInit(void);
void resetAltitudeControl(void);
void setSticksActiveStatus(bool areSticksActive);
void resetPositionControl(unsigned taskRateHz);
bool positionControl(void);
void altitudeControl(float targetAltitudeCm, float taskIntervalS, float targetAltitudeVelCmS, float velLimitCmS);
void moveTargetLocation(const vector2_t *stepEF, unsigned taskRateHz, bool forceAbortNav);// for nav modes to update the target position
void pitchForwardOverride(bool request);
void autopilotForceLevelPark(bool request); // heading/mag fault: force angle-mode self-level, never position hold
void autopilotSetNavHeadingOverride(bool valid, float headingDeg); // mission pre-turn: command nose heading directly
void initPositionHold(void);
void positionControlReanchor(void);
uint16_t autopilotGetEffectiveHoverThrottlePwm(void);
void autopilotCaptureHoverThrottleForAltHold(void);
void autopilotClearAltHoldHoverThrottle(void);
bool isBelowLandingAltitude(void);
float getAutopilotThrottle(void);

// Mission yaw control: rate injected as the yaw setpoint by rc.c while a
// navigation leg is being flown (see updateYawControl in autopilot_multirotor.c)
float autopilotGetYawRate(void);
bool autopilotYawControlActive(void);
void autopilotSetYawRateLimit(float rateLimitDps); // deg/s, 0 = no mission cap

#endif // !USE_WING
