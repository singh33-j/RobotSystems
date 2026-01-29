from time import sleep, time
from picarx import Picarx

try:
    from robot_hat import ADC
except ImportError:
    from sim_robot_hat import ADC


# ============================================================
# CONFIGURATION
# ============================================================
REFERENCE = [1400, 1400, 1400]   # required by assignment
FILTER_ALPHA = 0.5              # sensor LPF

EDGE_MAG_THRESH = 0.15          # minimum usable edge
CONTROL_DT = 0.01


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
            self.filt[i] = FILTER_ALPHA * raw[i] + (1 - FILTER_ALPHA) * self.filt[i]
        return self.filt.copy()

    def status(self, v):
        return [0 if v[i] <= REFERENCE[i] else 1 for i in range(3)]


# ============================================================
# INTERPRETATION — EDGE DETECTION
# ============================================================
class LineInterpreter:
    def __init__(self, polarity='dark'):
        self.polarity = polarity

    def compute_error(self, v):
        L, C, R = v

        # Remove global brightness
        mu = (L + C + R) / 3.0
        Lr, Cr, Rr = L - mu, C - mu, R - mu

        # Adjacent differences (edges)
        dLC = Cr - Lr
        dCR = Rr - Cr

        if self.polarity == 'light':
            dLC = -dLC
            dCR = -dCR

        # Normalize by local contrast
        spread = max(abs(Lr), abs(Cr), abs(Rr)) + 1e-6
        dLC /= spread
        dCR /= spread

        edge_mag = max(abs(dLC), abs(dCR))

        # ---- Corner override: both edges strong ----
        if abs(dLC) > 0.6 and abs(dCR) > 0.6:
            e = -(dLC + dCR)
            return max(-1.0, min(1.0, e))

        # ---- Weak edge → straight ----
        if edge_mag < EDGE_MAG_THRESH:
            return 0.0

        # ---- Dominant edge ----
        if abs(dLC) > abs(dCR):
            e = +dLC     # line on left → steer right
        else:
            e = -dCR     # line on right → steer left

        # Confidence scaling
        confidence = min(1.0, edge_mag / EDGE_MAG_THRESH)
        e *= confidence

        return max(-1.0, min(1.0, e))

    def line_lost(self, v):
        mu = sum(v) / 3.0
        return max(abs(x - mu) for x in v) < 25


# ============================================================
# PD CONTROLLER
# ============================================================
class PDController:
    def __init__(self, Kp=16.0, Kd=5.0, max_angle=30.0):
        self.Kp = Kp
        self.Kd = Kd
        self.max = max_angle
        self.e_last = 0.0
        self.t_last = time()

    def step(self, e):
        t = time()
        dt = max(t - self.t_last, 1e-4)

        de = (e - self.e_last) / dt
        u = self.Kp * e + self.Kd * de

        u = max(-self.max, min(self.max, u))

        self.e_last = e
        self.t_last = t
        return u

    def reset(self):
        self.e_last = 0.0
        self.t_last = time()


# ============================================================
# MAIN LOOP (WITH CURVE-AWARE SPEED CONTROL)
# ============================================================
if __name__ == "__main__":

    px = Picarx()
    sensor = LineSensor()
    interp = LineInterpreter(polarity='dark')
    ctrl   = PDController()

    px_power = 10

    try:
        while True:
            v = sensor.read()

            # Line lost → straighten and creep
            if interp.line_lost(v):
                px.set_dir_servo_angle(0)
                px.forward(px_power // 2)
                ctrl.reset()
                sleep(0.05)
                continue

            err = interp.compute_error(v)

            # -------------------------------
            # CURVATURE-AWARE SPEED SCHEDULING
            # -------------------------------
            err_mag = abs(err)
            if err_mag < 0.15:
                speed = px_power
            elif err_mag < 0.40:
                speed = int(px_power * 0.7)
            else:
                speed = int(px_power * 0.5)

            steer = ctrl.step(err)

            px.set_dir_servo_angle(steer)
            px.forward(speed)

            print(
                f"adc={[round(x,1) for x in v]} | "
                f"err={err:+.3f} | "
                f"steer={steer:+.1f} | "
                f"spd={speed}"
            )

            sleep(CONTROL_DT)

    except KeyboardInterrupt:
        print("\nStopping...")
        px.stop()
        sleep(0.1)
