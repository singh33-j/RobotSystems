"""
Line Following program for Picar-X with PD Control (Corrected Geometry)

Fixes:
    - Correct sensor coordinate frame
    - Error sign now matches PiCar-X steering geometry
"""

from time import sleep, time
from picarx import Picarx

try:
    from robot_hat import ADC
except ImportError:
    from sim_robot_hat import ADC


# ============================================================
# Section 3.1 — SENSING
# ============================================================
class LineSensor:
    def __init__(self, grayscale_pins=['A0','A1','A2'],
                 reference_values=None, alpha=0.2):

        self.adc = [ADC(p) for p in grayscale_pins]
        self.alpha = alpha
        self.filt = [0.0, 0.0, 0.0]

        self.reference = reference_values if reference_values else [1400,1400,1400]

    def read(self):
        raw = [a.read() for a in self.adc]
        for i in range(3):
            self.filt[i] = self.alpha*raw[i] + (1-self.alpha)*self.filt[i]
        return self.filt.copy()

    def status(self, v):
        return [0 if v[i] <= self.reference[i] else 1 for i in range(3)]

    def read_all(self):
        v = self.read()
        return v, self.status(v)


# ============================================================
# Section 3.2 — INTERPRETATION
# ============================================================
class LineInterpreter:
    def __init__(self, polarity='dark'):
        # CRITICAL FIX: sensor frame reversed
        self.pos = [1.0, 0.0, -1.0]
        self.polarity = polarity

    def error(self, v, ref):
        c = []
        for i in range(3):
            ci = ref[i] - v[i] if self.polarity == 'dark' else v[i] - ref[i]
            c.append(max(ci, 0.0))

        s = sum(c)
        if s == 0:
            return 0.0

        return sum(self.pos[i]*c[i] for i in range(3)) / s

    def lost(self, status):
        return status == [1,1,1]


# ============================================================
# Section 3.3 — CONTROL
# ============================================================
class PD:
    def __init__(self, Kp=15.0, Kd=4.0, max_angle=30.0, beta=0.7):
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

        self.ef = self.beta*self.ef + (1-self.beta)*e
        u = self.Kp*self.ef + self.Kd*(self.ef - self.elast)/dt

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

    sensor = LineSensor(reference_values=[1400,1400,1400])
    interp = LineInterpreter()
    ctrl = PD()

    power = 10

    try:
        while True:
            v, s = sensor.read_all()

            if interp.lost(s):
                px.set_dir_servo_angle(0)
                px.forward(power//2)
                sleep(0.1)
                ctrl.reset()
                continue

            e = interp.error(v, sensor.reference)
            steer = ctrl.step(e)

            px.set_dir_servo_angle(steer)
            px.forward(power)

            print(f"err={e:+.3f} steer={steer:+.1f} adc={v}")

            sleep(0.01)

    except KeyboardInterrupt:
        px.stop()
