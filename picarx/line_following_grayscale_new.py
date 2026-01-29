from time import sleep, time
from picarx_improved import Picarx

try:
    from robot_hat import ADC
except ImportError:
    from sim_robot_hat import ADC


# ============================================================
# configuration
# ============================================================

CONTROL_DT = 0.01
STARTUP_STRAIGHT_TIME = 0.2

MAX_STEER_DEG = 30.0
STEER_DEADZONE_DEG = 1.2

OFFSET_GAIN = 6.0
OFFSET_REF_ALPHA = 0.05    # how fast reference adapts
SIGNAL_MIN = 25.0

FORWARD_SPEED = 17
RECOVER_SPEED = 14


# ============================================================
# sensing
# ============================================================

class LineSensor:
    def __init__(self, pins=['A0','A1','A2']):
        self.adc = [ADC(p) for p in pins]

    def read(self):
        return [a.read() for a in self.adc]


# ============================================================
# interpretation
# ============================================================

class LineTracker:
    def __init__(self, polarity='dark'):
        self.polarity = polarity
        self.offset_ref = 0.0
        self.ref_initialized = False

    def compute_offset(self, v):
        L, C, R = v

        # remove background
        m = (L + C + R) / 3.0
        Ln = L - m
        Cn = C - m
        Rn = R - m

        signal = abs(Ln) + abs(Cn) + abs(Rn)
        if signal < SIGNAL_MIN:
            return None, signal

        # center of mass offset
        offset = (-1.0 * Ln + 1.0 * Rn) / signal

        if self.polarity == 'light':
            offset = -offset

        return offset, signal

    def update_reference(self, offset):
        if not self.ref_initialized:
            self.offset_ref = offset
            self.ref_initialized = True
        else:
            # slowly adapt reference along bends
            self.offset_ref = (
                (1 - OFFSET_REF_ALPHA) * self.offset_ref +
                OFFSET_REF_ALPHA * offset
            )


# ============================================================
# controller
# ============================================================

class OffsetController:
    def __init__(self, Kp=1.0):
        self.Kp = Kp

    def step(self, error):
        steer = self.Kp * error
        steer = max(-MAX_STEER_DEG, min(MAX_STEER_DEG, steer))
        if abs(steer) < STEER_DEADZONE_DEG:
            steer = 0.0
        return steer


# ============================================================
# main loop
# ============================================================

if __name__ == "__main__":

    px = Picarx()
    sensor = LineSensor()
    tracker = LineTracker(polarity='dark')
    ctrl = OffsetController(Kp=OFFSET_GAIN)

    start_time = time()
    lost = False
    last_steer = 0.0

    try:
        while True:

            elapsed = time() - start_time
            v = sensor.read()

            # startup straight
            if elapsed < STARTUP_STRAIGHT_TIME:
                px.set_dir_servo_angle(0)
                px.forward(FORWARD_SPEED)
                sleep(CONTROL_DT)
                continue

            offset, signal = tracker.compute_offset(v)

            # line lost when signal collapses
            if offset is None:
                px.set_dir_servo_angle(last_steer)
                px.backward(RECOVER_SPEED)

                print(
                    f"LOST  | adc={v} | "
                    f"signal={signal:.1f} | "
                    f"last_steer={last_steer:+.1f}"
                )

                sleep(CONTROL_DT)
                continue

            # update reference when signal is strong
            tracker.update_reference(offset)

            error = offset - tracker.offset_ref
            steer = ctrl.step(error)

            last_steer = steer

            px.set_dir_servo_angle(steer)
            px.forward(FORWARD_SPEED)

            print(
                f"TRACK | adc={v} | "
                f"signal={signal:.1f} | "
                f"offset={offset:+.3f} | "
                f"ref={tracker.offset_ref:+.3f} | "
                f"err={error:+.3f} | "
                f"steer={steer:+.1f}"
            )

            sleep(CONTROL_DT)

    except KeyboardInterrupt:
        px.stop()
        sleep(0.1)
