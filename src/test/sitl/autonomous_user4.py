#!/usr/bin/env python3
"""Exercise USER4 handoff after autonomous ALT/POS HOLD engagement.

Build: make TARGET=SITL
Run: python3 src/test/sitl/autonomous_user4.py --binary obj/main/betaflight_SITL.elf
"""

import argparse
from pathlib import Path
import struct
import tempfile
import time

import sitl_harness as h

BOX_ANGLE = 1  # MSP permanent mode ID


class ReleaseModel(h.MotionModel):
    """Hold still for entry checks, inject freefall, then close the motor loop."""

    def __init__(self):
        super().__init__()
        self.released = False

    def step(self, dt, motors):
        if self.released:
            super().step(dt, motors)

    def release(self):
        self.pos[2] = 12.0
        self.released = True


def run(binary):
    h.TELEMETRY_PORT = 0
    workdir = Path(tempfile.mkdtemp(prefix="autonomous_user4_"))
    sitl = h.Sitl(binary, str(workdir))
    rc = h.RcFeed()
    motors = h.MotorFeed()
    fdm = h.FdmFeed(motors)
    fdm.model = ReleaseModel()

    def state():
        # DEBUG_FAILSAFE[4] exposes the actual autonomous state machine.
        return struct.unpack("<8h", sitl.msp.request(254))[4]

    def armed():
        return h.BOX_ARM in sitl.modes()

    def stable_for(seconds, predicate, message):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            assert predicate(), message
            time.sleep(0.04)

    def wait_state(expected, timeout=15):
        h.wait_for(f"autonomous stage {expected}", lambda: state() == expected,
                   timeout=timeout, interval=0.02)

    def log():
        return (workdir / "sitl.log").read_text()

    try:
        sitl.provision([
            "feature GPS",
            "set gps_provider = VIRTUAL",
            "set failsafe_procedure = DROP",
            "set small_angle = 180",
            "set trust_mag = ON",
            "set debug_mode = FAILSAFE",
            "set ap_velocity_drag_coeff = 50",
            "aux 0 0 0 1700 2100 0 0",   # ARM on AUX1
            "aux 1 40 1 900 2100 0 0",   # USER1 channel on AUX2
            "aux 2 42 2 1700 2100 0 0",  # USER3 on AUX3
            "aux 3 43 3 1700 2100 0 0",  # USER4 on AUX4
            "aux 4 27 4 1700 2100 0 0",  # FAILSAFE on AUX5
            "aux 5 41 5 1700 2100 0 0",  # USER2 on AUX6
            "aux 6 56 6 1700 2100 0 0",  # AUTOPILOT on AUX7
            f"waypoint insert 0 {h.WP_LAT:.7f} {h.HOME_LON:.7f} "
            f"{int((h.HOME_ALT_M + 12) * 100)} 300 flyover 0 none",
        ])
        sitl.start()
        motors.start()
        rc.start()
        fdm.start()
        h.wait_for("RX and startup protections clear",
                   lambda: sitl.status()["arming_flags"] == 0, timeout=40)
        sitl.acc_calibrate()
        time.sleep(2)
        h.wait_for("accelerometer calibration complete",
                   lambda: sitl.status()["arming_flags"] == 0, timeout=20)

        rc.set(7, h.RC_HIGH)
        stable_for(0.3, lambda: not armed(), "USER4 alone armed")
        rc.set(6, h.RC_HIGH)
        rc.set(4, h.RC_HIGH)
        wait_state(2)
        stable_for(0.5, lambda: not armed(), "Early USER4 bypassed freefall")
        rc.set(10, h.RC_HIGH)
        fdm.model.release()
        h.wait_for("normal freefall arms", armed, timeout=5, interval=0.02)
        wait_state(5)
        required = {BOX_ANGLE, h.BOX_ALTHOLD, h.BOX_POSHOLD}
        assert required <= sitl.modes(), "Stage 5 without all hold modes"
        stable_for(0.8, lambda: state() == 5 and h.BOX_AUTOPILOT not in sitl.modes(),
                   "Early USER4 or BOXAUTOPILOT engaged the mission")
        assert "stage 6:" not in log(), "Early USER4 queued a handoff"

        rc.set(7, h.RC_LOW)
        time.sleep(0.15)
        north_before = fdm.model.pos[1]
        rc.set(7, h.RC_HIGH)
        wait_state(8)
        assert required | {h.BOX_AUTOPILOT} <= sitl.modes()
        trace = log()
        assert trace.index("stage 6:") < trace.index("stage 7:") < trace.index("stage 8:")
        h.wait_for("mission moves away from autonomous hover anchor",
                   lambda: fdm.model.pos[1] - north_before > 5.0, timeout=15)

        rc.set(7, h.RC_LOW)
        wait_state(5)
        stable_for(0.4, lambda: h.BOX_AUTOPILOT not in sitl.modes(),
                   "USER4 off did not release AUTOPILOT")
        rc.set(7, h.RC_HIGH)
        wait_state(8)
        rc.set(4, h.RC_LOW)
        h.wait_for("ARM low disarms", lambda: not armed(), timeout=3)
        wait_state(0)
        assert h.BOX_AUTOPILOT not in sitl.modes()

        # A fresh USER2 cycle must also wait for freefall with USER4 held.
        fdm.model.released = False
        fdm.model.accel = [0.0, 0.0, 0.0]
        fdm.model.vel = [0.0, 0.0, 0.0]
        rc.set(6, h.RC_LOW)
        rc.set(10, h.RC_LOW)
        time.sleep(0.3)
        rc.set(4, h.RC_HIGH)
        h.wait_for("manual arm before USER2 cycle", armed, timeout=5)
        rc.set(9, h.RC_HIGH)
        wait_state(2, timeout=10)
        stable_for(0.4, lambda: not armed() and h.BOX_AUTOPILOT not in sitl.modes(),
                   "Held USER4 bypassed USER2 recovery/freefall")
        print("PASS: early USER4, USER2/USER3 freefall gates, stage 3 -> 4 -> 5 -> 6 -> 7 -> 8, "
              "mission control handoff, USER4 off/retry and ARM-low reset")
    finally:
        if rc.is_alive():
            rc.set(4, h.RC_LOW)
            time.sleep(0.2)
        for feed in (rc, fdm, motors):
            feed.shutdown()
            if feed.is_alive():
                feed.join(timeout=1)
        sitl.stop()
        print(f"Artifacts: {workdir}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", required=True)
    run(parser.parse_args().binary)
