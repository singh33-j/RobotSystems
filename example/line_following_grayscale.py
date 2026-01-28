"""
Line Following for PiCar-X using Grayscale Sensors (Robust, Correct Geometry)

Behavior:
- Smaller ADC = darker
- Error = 0 when center sensor is darkest and left ≈ right
- Left darker  -> steer RIGHT
- Right darker -> steer LEFT

Reference values are FIXED at [1400, 1400, 1400]
"""

from time import sleep, time
from picarx import Picarx

# ADC import (hardware or simulation)
try:
    from robot_hat import ADC
except ImportError:
    from sim_robot_hat import ADC


# ============================================================
# Section 3.1 — SENSING
# ============================================================
class LineSensor:
    """
    Direct ADC access:
    A0 = left, A1 = center, A2 = right
    """

    def __init__(self, pins=['A0','A1','A2'], reference=None, alpha=0.25):
        self.adc = [ADC(p) for p in pins]
        self.alpha = alpha
        self.filtered = [0.0, 0.0, 0.0]

        # EXACT reference logic you requested
        self.reference = reference if reference else [1400, 1400, 1400]

    def read(self):
        raw = [a.read() for a in self.adc]
        for i in range(3):
            self.filtered[i] = (
                self.alpha * raw[i] +
                (1.0 - self.alpha) * self.filtered[i]
            )
        return self.filtered.copy()

    def status(self, v):
        # Only for line-lost detection
        return [0 if v[i] <= self.reference[i] else 1 for i in range(3)]

    def read_all(self):
        v = self.read()
        return v, self.status(v)


# ============================================================
# Section 3.2 — INTERPRETATION (CORRECT MODEL)
# ============================================================
class LineInterpreter:
    """
    Error based on LEFT–RIGHT imbalance, gated by center confidence.
    """

    def __init__(self, polarity='dark'):
        self.polarity = polarity

    def compute_error(self, v, ref):
        """
        Returns signed error in [-1, 1]

        Positive error  -> steer RIGHT
        Negative error  -> steer LEFT
        """

        # Contrast: bigger = more line-like
        if self.polarity == 'dark':
            cL = ref[0] - v[0]
            cC = ref[1] - v[1]
            cR = ref[2] - v[2]
        else:
            cL = v[0] - ref[0]
            cC = v[1] - ref[1]
            cR = v[2] - ref[2]

        # Clamp contrasts
        cL = max(cL, 0.0)
        cC = max(cC, 0.0)
        cR = max(cR, 0.0)

        # LEFT–RIGHT imbalance
        denom = abs(cL) + abs(cR) + 1e-6
        imbalance = (cL - cR) / denom
        # cL > cR  -> positive -> steer RIGHT (desired)

        # Confidence: center darker than sides
        confidence = cC - 0.5 * (cL + cR)
        confidence = max(confidence, 0.0)

        # Normalize confidence (empirical but stable)
        conf_scale = 300.0
        conf_gain = min(confidence / conf_scale, 1.0)

        return imbalance * conf_gain

    def line_lost(self, status):
        # All sensors see background
        return status == [1, 1, 1]


# ============================================================
# Section 3.3 — CONTROL
# ============================================================
class PDController:
    def __init__(self, Kp=25.0, Kd=4.0, max_angle=30.0, beta=0.7):
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

        # Filter error
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
# MAIN PROGRAM
# ============================================================
if __name__ == "__main__":

    px = Picarx()

    sensor = LineSensor(reference=[1400,1400,1400])
    interp = LineInterpreter(polarity='dark')
    ctrl = PDController()

    power = 10

    try:
        while True:
            v, s = sensor.read_all()

            if interp.line_lost(s):
                # gentle forward search
                px.set_dir_servo_angle(0)
                px.forward(power // 2)
                sleep(0.1)
                ctrl.reset()
                continue

            err = interp.compute_error(v, sensor.reference)
            steer = ctrl.step(err)

            px.set_dir_servo_angle(steer)
            px.forward(power)

            print(
                f"adc={v} | "
                f"err={err:+.3f} | "
                f"steer={steer:+.1f}"
            )

            sleep(0.01)

    except KeyboardInterrupt:
        px.stop()
        sleep(0.1)
