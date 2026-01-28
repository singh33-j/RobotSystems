"""
Line Following for PiCar-X using TARGET sensor profile

Goal:
- Maintain sensor readings near TARGET = [341, 186, 329]
- Left darker  -> steer RIGHT
- Right darker -> steer LEFT
- Error = 0 ONLY when near target vector
"""

from time import sleep, time
from picarx import Picarx

try:
    from robot_hat import ADC
except ImportError:
    from sim_robot_hat import ADC


# ============================================================
# CONFIG
# ============================================================
REFERENCE = [1400, 1400, 1400]
TARGET    = [341.0, 186.0, 329.0]

FILTER_ALPHA = 0.25


# ============================================================
# SENSING
# ============================================================
class LineSensor:
    def __init__(self, pins=['A0','A1','A2']):
        self.adc = [ADC(p) for p in pins]
        self.f = [0.0, 0.0, 0.0]

    def read(self):
        raw = [a.read() for a in self.adc]
        for i in range(3):
            self.f[i] = FILTER_ALPHA * raw[i] + (1 - FILTER_ALPHA) * self.f[i]
        return self.f.copy()

    def status(self, v):
        return [0 if v[i] <= REFERENCE[i] else 1 for i in range(3)]


# ============================================================
# INTERPRETATION (TARGET TRACKING)
# ============================================================
class LineInterpreter:
    def __init__(self, target):
        self.target = target

    def compute_error(self, v):
        """
        Error = (L - L*) - (R - R*)
        Positive -> steer RIGHT
        Negative -> steer LEFT
        """

        dL = v[0] - self.target[0]
        dR = v[2] - self.target[2]

        # Signed lateral error
        e = dL - dR

        # Normalize for stability
        scale = abs(self.target[0] - self.target[2]) + 1e-6
        return e / scale

    def line_lost(self, status):
        return status == [1, 1, 1]


# ============================================================
# CONTROL
# ============================================================
class PDController:
    def __init__(self, Kp=4.0, Kd=2.0, max_angle=30.0, beta=0.7):
        self.Kp = Kp
        self.Kd = Kd
        self.max = max_angle
        self.beta = beta

        self.ef = 0.0
        self.elast = 0.0
        self.tlast = time()

    def step(self, e):
        t = time()
        dt = max(t - self.tlast, 1e-4)

        self.ef = self.beta * self.ef + (1 - self.beta) * e

        u = self.Kp * self.ef + self.Kd * (self.ef - self.elast) / dt
        u = max(-self.max, min(self.max, u))

        self.elast = self.ef
        self.tlast = t
        return u

    def reset(self):
        self.ef = 0.0
        self.elast = 0.0
        self.tlast = time()


# ============================================================
# MAIN
# ============================================================
if __name__ == "__main__":

    px = Picarx()

    sensor = LineSensor()
    interp = LineInterpreter(TARGET)
    ctrl   = PDController()

    power = 10

    try:
        while True:
            v = sensor.read()
            s = sensor.status(v)

            if interp.line_lost(s):
                px.set_dir_servo_angle(0)
                px.forward(power // 2)
                ctrl.reset()
                sleep(0.1)
                continue

            err = interp.compute_error(v)
            steer = ctrl.step(err)

            px.set_dir_servo_angle(steer)
            px.forward(power)

            print(
                f"adc={[round(x,1) for x in v]} | "
                f"err={err:+.3f} | "
                f"steer={steer:+.1f}"
            )

            sleep(0.01)

    except KeyboardInterrupt:
        px.stop()
        sleep(0.1)
