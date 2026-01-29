from time import sleep, time
from picarx import Picarx

try:
    from robot_hat import ADC
except ImportError:
    from sim_robot_hat import ADC


# ============================================================
# CONFIGURATION
# ============================================================

FILTER_ALPHA = 0.7
EDGE_THRESH  = 0.05
CONTROL_DT  = 0.01

FORWARD_SPEED = 4
REVERSE_SPEED = 3

LINE_LOST_DELAY = 0.5     # <<< only reverse if lost > 0.5 s
MAX_REVERSE_TIME = 1.0    # safety cap (can remove later)


# ============================================================
# SENSING
# ============================================================

class LineSensor:
    def __init__(self, pins=['A0','A1','A2']):
        self.adc = [ADC(p) for p in pins]
        self.filt = [0.0, 0.0, 0.0]

    def read(self):
        raw = [a.read() for a in self.adc]
        for i in range(3):
            self.filt[i] = (
                FILTER_ALPHA * raw[i]
                + (1 - FILTER_ALPHA) * self.filt[i]
            )
        return self.filt.copy()


# ============================================================
# INTERPRETATION — EDGE BASED
# ============================================================

class LineInterpreter:
    def __init__(self, polarity='dark'):
        self.polarity = polarity

    def compute_error(self, v):
        L, C, R = v

        dLC = C - L
        dCR = R - C

        if self.polarity == 'light':
            dLC = -dLC
            dCR = -dCR

        contrast = abs(L - C) + abs(C - R) + 1e-6
        dLC /= contrast
        dCR /= contrast

        if max(abs(dLC), abs(dCR)) < EDGE_THRESH:
            return None   # no reliable line

        if abs(dLC) > abs(dCR):
            e = +dLC     # line on left → steer right
        else:
            e = -dCR     # line on right → steer left

        return max(-1.0, min(1.0, 0.7 * e))

    def line_lost(self, v):
        # contrast collapse test
        return max(v) - min(v) < 20


# ============================================================
# PD CONTROLLER
# ============================================================

class PDController:
    def __init__(self, Kp=14.0, Kd=2.0, max_angle=30.0):
        self.Kp = Kp
        self.Kd = Kd
        self.max = max_angle
        self.e_last = 0.0
        self.t_last = time()

    def step(self, e):
        now = time()
        dt = max(now - self.t_last, 1e-4)

        de = (e - self.e_last) / dt
        u = self.Kp * e + self.Kd * de

        u = max(-self.max, min(self.max, u))

        self.e_last = e
        self.t_last = now
        return u

    def reset(self):
        self.e_last = 0.0
        self.t_last = time()


# ============================================================
# MAIN LOOP
# ============================================================

if __name__ == "__main__":

    px = Picarx()
    sensor = LineSensor()
    interp = LineInterpreter(polarity='dark')
    ctrl   = PDController()

    last_valid_time  = time()
    last_valid_steer = 0.0

    try:
        while True:
            v = sensor.read()
            err = interp.compute_error(v)
            now = time()

            # ---------- LINE LOST OR UNCERTAIN ----------
            if err is None or interp.line_lost(v):

                t_lost = now - last_valid_time

                # Ignore brief losses
                if t_lost < LINE_LOST_DELAY:
                    px.forward(FORWARD_SPEED)
                    sleep(CONTROL_DT)
                    continue

                # True loss → reverse
                reverse_time = min(t_lost, MAX_REVERSE_TIME)

                print(f"LINE LOST {t_lost:.2f}s → reversing {reverse_time:.2f}s")

                px.stop()
                px.set_dir_servo_angle(last_valid_steer)

                px.backward(REVERSE_SPEED)
                sleep(reverse_time)

                px.stop()
                ctrl.reset()
                sleep(0.05)
                continue

            # ---------- NORMAL TRACKING ----------
            steer = ctrl.step(err)

            px.set_dir_servo_angle(steer)
            px.forward(FORWARD_SPEED)

            last_valid_time  = now
            last_valid_steer = steer

            print(
                f"adc={[round(x,1) for x in v]} | "
                f"err={err:+.3f} | "
                f"steer={steer:+.1f}"
            )

            sleep(CONTROL_DT)

    except KeyboardInterrupt:
        print("\nStopping...")
        px.stop()
        sleep(0.1)
