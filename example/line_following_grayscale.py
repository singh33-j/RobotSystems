"""
Robust Line Following for PiCar-X
Edge-based interpretation (instruction-compliant)
Low-contrast and lighting-robust
Fixed PD controller
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
FILTER_ALPHA = 0.6        # sensor smoothing (0.4–0.7)
EDGE_MIN     = 0.15       # minimum normalized edge strength

PX_POWER  = 10
MAX_STEER = 30.0

# Fixed PD gains (tune these)
KP = 12.0
KD = 6.0


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
                FILTER_ALPHA * raw[i] +
                (1 - FILTER_ALPHA) * self.filt[i]
            )

        return self.filt.copy()


# ============================================================
# INTERPRETATION — LOW-CONTRAST EDGE DETECTION
# ============================================================
class LineInterpreter:
    def __init__(self, polarity='dark'):
        """
        polarity:
        'dark'  -> line darker than floor
        'light' -> line lighter than floor
        """
        self.polarity = polarity

    def compute_error(self, v):
        """
        Returns normalized error in [-1, 1]

        +error → line on LEFT  → steer RIGHT
        -error → line on RIGHT → steer LEFT
        """
        L, C, R = v

        # --- Normalize brightness (removes lighting bias) ---
        mu = (L + C + R) / 3.0
        Lr, Cr, Rr = L - mu, C - mu, R - mu

        # Adjacent differences (edge detection)
        dLC = Cr - Lr
        dCR = Rr - Cr

        # Polarity handling
        if self.polarity == 'light':
            dLC = -dLC
            dCR = -dCR

        # Normalize by local contrast
        spread = max(abs(Lr), abs(Cr), abs(Rr)) + 1e-6
        dLC /= spread
        dCR /= spread

        # Weak edge → treat as centered
        edge_strength = max(abs(dLC), abs(dCR))
        if edge_strength < EDGE_MIN:
            return 0.0

        # Decide direction and magnitude
        if abs(dLC) > abs(dCR):
            return +min(abs(dLC), 1.0)
        else:
            return -min(abs(dCR), 1.0)

    def line_lost(self, v):
        """
        Line lost when relative contrast disappears
        """
        mu = sum(v) / 3.0
        return max(abs(x - mu) for x in v) < 30


# ============================================================
# FIXED PD CONTROLLER
# ============================================================
class PDController:
    def __init__(self, Kp, Kd, max_angle):
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
    ctrl   = PDController(KP, KD, MAX_STEER)

    try:
        while True:
            v = sensor.read()

            if interp.line_lost(v):
                # Slow straight search
                px.set_dir_servo_angle(0)
                px.forward(PX_POWER // 2)
                ctrl.reset()
                sleep(0.05)
                continue

            err = interp.compute_error(v)
            steer = ctrl.step(err)

            px.set_dir_servo_angle(steer)
            px.forward(PX_POWER)

            print(
                f"adc={[round(x,1) for x in v]} | "
                f"err={err:+.3f} | "
                f"steer={steer:+.1f}"
            )

            sleep(0.01)

    except KeyboardInterrupt:
        print("\nStopping...")
        px.stop()
        sleep(0.1)
