"""
Robust Line Following for PiCar-X (Target-Based, Fast PD)

Goal:
- Maintain sensor readings near TARGET = [341, 186, 329]
- Smaller ADC = darker
- Left darker  -> steer RIGHT
- Right darker -> steer LEFT
"""

from time import sleep, time
from picarx import Picarx

try:
    from robot_hat import ADC
except ImportError:
    from sim_robot_hat import ADC


# ============================================================
# CONFIGURATION
# ============================================================
REFERENCE = [1400, 1400, 1400]          # fixed per your request
TARGET    = [341.0, 186.0, 329.0]       # measured centered profile

FILTER_ALPHA = 0.5                      # FAST sensor response


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
        # Only used for line-lost detection
        return [0 if v[i] <= REFERENCE[i] else 1 for i in range(3)]


# ============================================================
# INTERPRETATION — TARGET TRACKING
# ============================================================
class LineInterpreter:
    def __init__(self, target):
        self.target = target
        self.scale = abs(target[0] - target[2]) + 1e-6

    def compute_error(self, v):
        """
        Signed lateral error:
        (L - L*) - (R - R*)
        Positive -> steer RIGHT
        Negative -> steer LEFT
        """
        dL = v[0] - self.target[0]
        dR = v[2] - self.target[2]

        e = (dL - dR) / self.scale

        # Hard clamp to avoid spikes
        return max(-1.0, min(1.0, e))

    def line_lost(self, status):
        return status == [1, 1, 1]


# ============================================================
# FAST PD CONTROLLER (NO LAG)
# ============================================================
class PDController:
    def __init__(self, Kp=10.0, Kd=8.0, max_angle=30.0):
        self.Kp = Kp
        self.Kd = Kd
        self.max = max_angle

        self.elast = 0.0
        self.tlast = time()

    def step(self, e):
        now = time()
        dt = max(now - self.tlast, 1e-4)

        de = (e - self.elast) / dt

        u = self.Kp * e + self.Kd * de
        u = max(-self.max, min(self.max, u))

        self.elast = e
        self.tlast = now
        return u

    def reset(self):
        self.elast = 0.0
        self.tlast = time()


# ============================================================
# MAIN LOOP
# ============================================================
if __name__ == "__main__":

    px = Picarx()

    sensor = LineSensor()
    interp = LineInterpreter(TARGET)
    ctrl   = PDController()

    px_power = 10

    try:
        while True:
            v = sensor.read()
            s = sensor.status(v)

            if interp.line_lost(s):
                px.set_dir_servo_angle(0)
                px.forward(px_power // 2)
                ctrl.reset()
                sleep(0.05)
                continue

            err = interp.compute_error(v)
            steer = ctrl.step(err)

            px.set_dir_servo_angle(steer)
            px.forward(px_power)

            print(
                f"adc={[round(x,1) for x in v]} | "
                f"err={err:+.3f} | "
                f"steer={steer:+.1f}"
            )

            sleep(0.01)

    except KeyboardInterrupt:
        print("\nStopping")
        px.stop()
        sleep(0.1)
