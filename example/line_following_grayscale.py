from time import sleep, time
from picarx import Picarx

try:
    from robot_hat import ADC
except ImportError:
    from sim_robot_hat import ADC


# ============================================================
# CONFIGURATION
# ============================================================

REFERENCE = [1400, 1400, 1400]   # required by assignment (binary only)
FILTER_ALPHA = 0.7              # faster sensor response

EDGE_MAG_THRESH  = 0.05          # minimum usable edge
EDGE_ASYM_THRESH = 0.03       # edge must be directional

CONTROL_DT = 0.01

FORWARD_SPEED = 4               # slower motion = tighter curves


# ============================================================
# SENSING
# ============================================================

class LineSensor:
    def __init__(self, pins=['A0', 'A1', 'A2']):
        self.adc  = [ADC(p) for p in pins]
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
# INTERPRETATION — EDGE + FALLBACK
# ============================================================

class LineInterpreter:
    def __init__(self, polarity='dark'):
        self.polarity = polarity

    def compute_error(self, v):
        L, C, R = v

        # --------------------------------------------------------
        # 1. Remove global brightness (lighting invariance)
        # --------------------------------------------------------
        mu = (L + C + R) / 3.0
        Lr, Cr, Rr = L - mu, C - mu, R - mu

        # --------------------------------------------------------
        # 2. Adjacent differences = edge signals
        # --------------------------------------------------------
        dLC = Cr - Lr
        dCR = Rr - Cr

        if self.polarity == 'light':
            dLC = -dLC
            dCR = -dCR

        # --------------------------------------------------------
        # 3. Normalize by local contrast
        # --------------------------------------------------------
        spread = max(abs(Lr), abs(Cr), abs(Rr)) + 1e-6
        dLC /= spread
        dCR /= spread

        edge_mag  = max(abs(dLC), abs(dCR))
        edge_asym = abs(abs(dLC) - abs(dCR))

        use_edge = (
            edge_mag  > EDGE_MAG_THRESH and
            edge_asym > EDGE_ASYM_THRESH
        )

        # --------------------------------------------------------
        # 4A. PRIMARY: Edge-based decision
        # --------------------------------------------------------
        if use_edge:
            if abs(dLC) > abs(dCR):
                e = +dLC      # line on left → steer right
            else:
                e = -dCR      # line on right → steer left

        # --------------------------------------------------------
        # 4B. FALLBACK: Brightness centroid (curve-safe)
        # --------------------------------------------------------
        else:
            # darker = more weight for dark line
            wL = -Lr
            wC = -Cr
            wR = -Rr

            denom = abs(wL) + abs(wC) + abs(wR) + 1e-6
            e = (-1*wL + 0*wC + 1*wR) / denom

        # --------------------------------------------------------
        # 5. Soft clamp (prevents snapping)
        # --------------------------------------------------------
        return max(-1.0, min(1.0, 0.7 * e))

    def line_lost(self, v):
        mu = sum(v) / 3.0
        return max(abs(x - mu) for x in v) < 25


# ============================================================
# PD CONTROLLER
# ============================================================

class PDController:
    def __init__(self, Kp=16.0, Kd=3.0, max_angle=30.0):
        self.Kp = Kp
        self.Kd = Kd
        self.max = max_angle
        self.e_last = 0.0
        self.t_last = time()

    def step(self, e):
        now = time()
        dt = max(now - self.t_last, 1e-4)

        de = (e - self.e_last) / dt
        u  = self.Kp * e + self.Kd * de

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

            # Lost line → straighten + slow forward
            if interp.line_lost(v):
                px.set_dir_servo_angle(0)
                px.forward(FORWARD_SPEED // 2)
                ctrl.reset()
                sleep(0.05)
                continue

            err    = interp.compute_error(v)
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
