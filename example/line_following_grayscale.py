"""
Robust Line Following for PiCar-X (Edge-Based Interpretation)

Interpretation method (per instructions):
- Detect sharp changes between adjacent sensors (edges)
- Use edge location + sign to determine left/right
- Use edge magnitude to determine how far off-center
- Robust to lighting via normalization
- Supports dark or light line via polarity
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
REFERENCE = [1400, 1400, 1400]   # used only for line-lost detection
FILTER_ALPHA = 0.5               # sensor low-pass filter

POLARITY = 'dark'                # 'dark' or 'light'
EDGE_SENSITIVITY = 0.25           # smaller → more aggressive response


# ============================================================
# SENSING
# ============================================================
class LineSensor:
    def __init__(self, pins=['A0','A1','A2']):
        self.adc = [ADC(p) for p in pins]
        self.f   = [0.0, 0.0, 0.0]

    def read(self):
        raw = [a.read() for a in self.adc]
        for i in range(3):
            self.f[i] = FILTER_ALPHA * raw[i] + (1 - FILTER_ALPHA) * self.f[i]
        return self.f.copy()

    def status(self, v):
        # 0 = dark (line), 1 = bright (floor)
        return [0 if v[i] <= REFERENCE[i] else 1 for i in range(3)]


# ============================================================
# INTERPRETATION — EDGE DETECTION (INSTRUCTION METHOD)
# ============================================================
class LineInterpreter:
    def __init__(self, polarity='dark', sensitivity=0.3):
        self.polarity = polarity
        self.sensitivity = sensitivity

    def compute_error(self, v):
        """
        Edge-based signed error in [-1, 1]

        Steps:
        1. Normalize for lighting
        2. Compute adjacent differences (edges)
        3. Select strongest edge
        4. Convert to signed magnitude error
        """

        L, C, R = v

        # --- Normalize for lighting robustness ---
        mean = (L + C + R) / 3.0
        if mean < 1e-6:
            return 0.0

        L /= mean
        C /= mean
        R /= mean

        # --- Adjacent differences (edges) ---
        dLC = C - L
        dCR = R - C

        # --- Polarity handling ---
        # For dark line, we want negative intensity transitions
        if self.polarity == 'dark':
            dLC = -dLC
            dCR = -dCR

        # --- Find strongest edge ---
        if abs(dLC) > abs(dCR):
            edge = dLC
            sign = -1.0    # edge on left → line is left → steer right
        else:
            edge = dCR
            sign = +1.0    # edge on right → line is right → steer left

        # --- Scale magnitude ---
        mag = min(abs(edge) / self.sensitivity, 1.0)

        return sign * mag

    def line_lost(self, status):
        # All sensors see floor
        return status == [1, 1, 1]


# ============================================================
# FAST PD CONTROLLER
# ============================================================
class PDController:
    def __init__(self, Kp=12.0, Kd=6.0, max_angle=30.0):
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

    interp = LineInterpreter(
        polarity=POLARITY,
        sensitivity=EDGE_SENSITIVITY
    )

    ctrl = PDController()
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
