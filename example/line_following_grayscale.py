from time import sleep, time
from picarx import Picarx

try:
    from robot_hat import ADC
except ImportError:
    from sim_robot_hat import ADC


# ============================================================
# CONFIGURATION
# ============================================================

FILTER_ALPHA = 0.7          # sensor low-pass filter
EDGE_THRESH  = 0.05         # minimum usable edge strength
CONTROL_DT  = 0.01

FORWARD_SPEED = 4           # slower speed for curves


# ============================================================
# SENSING
# ============================================================

class LineSensor:
    def __init__(self, pins=['A0', 'A1', 'A2']):
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
# INTERPRETATION — PURE EDGE METHOD (NO MEAN REMOVAL)
# ============================================================

class LineInterpreter:
    def __init__(self, polarity='dark'):
        self.polarity = polarity

    def compute_error(self, v):
        L, C, R = v

        # Adjacent differences (edges)
        dLC = C - L          # left–center edge
        dCR = R - C          # center–right edge

        if self.polarity == 'light':
            dLC = -dLC
            dCR = -dCR

        # Normalize by local contrast
        contrast = abs(L - C) + abs(C - R) + 1e-6
        dLC /= contrast
        dCR /= contrast

        edge_strength = max(abs(dLC), abs(dCR))

        # If no detectable edge → go straight
        if edge_strength < EDGE_THRESH:
            return 0.0

        # Choose stronger edge
        if abs(dLC) > abs(dCR):
            e = +dLC       # left darker → steer right
        else:
            e = -dCR       # right darker → steer left

        # Soft clamp
        return max(-1.0, min(1.0, 0.7 * e))

    def line_lost(self, v):
        # If all sensors are nearly equal → no contrast
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

    try:
        while True:
            v = sensor.read()

            # Lost line → slow, straight recovery
            if interp.line_lost(v):
                px.set_dir_servo_angle(0)
                px.forward(FORWARD_SPEED // 2)
                ctrl.reset()
                sleep(0.05)
                continue

            err = interp.compute_error(v)
            steer = ctrl.step(err)

            px.set_dir_servo_angle(steer)
            px.forward(FORWARD_SPEED)

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
