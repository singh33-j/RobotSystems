from time import sleep, time
from picarx import Picarx

try:
    from robot_hat import ADC
except ImportError:
    from sim_robot_hat import ADC


# ============================================================
# CONFIGURATION
# ============================================================
REFERENCE = [1400, 1400, 1400]   # assignment requirement
FILTER_ALPHA = 0.7              # sensor LPF

EDGE_MAG_THRESH = 0.12          # minimum usable edge strength
LOST_DELAY = 0.5             # seconds before declaring loss

CONTROL_DT = 0.01

FORWARD_SPEED = 4
REVERSE_SPEED = 4


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


# ============================================================
# INTERPRETATION — EDGE DETECTION (RETURNS VALIDITY)
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

        # Normalize by contrast
        spread = max(abs(Lr), abs(Cr), abs(Rr)) + 1e-6
        dLC /= spread
        dCR /= spread

        edge_mag = max(abs(dLC), abs(dCR))

        # ❗ No usable edge
        if edge_mag < EDGE_MAG_THRESH:
            return 0.0, False

        # Choose dominant edge
        if abs(dLC) > abs(dCR):
            e = +dLC      # line on left → steer right
        else:
            e = -dCR      # line on right → steer left

        # Soft clamp
        e = max(-1.0, min(1.0, 0.7 * e))
        return e, True


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
# MAIN LOOP
# ============================================================
if __name__ == "__main__":

    px = Picarx()
    sensor = LineSensor()
    interp = LineInterpreter(polarity='dark')
    ctrl   = PDController()

    last_seen_time = time()
    last_steer = 0.0

    try:
        while True:
            now = time()
            v = sensor.read()

            err, valid = interp.compute_error(v)

            # -------------------------
            # LINE CONFIDENTLY DETECTED
            # -------------------------
            if valid:
                last_seen_time = now
                steer = ctrl.step(err)
                last_steer = steer

                px.set_dir_servo_angle(steer)
                px.forward(FORWARD_SPEED)

            # -------------------------
            # LINE NOT CONFIDENT
            # -------------------------
            else:
                t_lost = now - last_seen_time

                # Ignore brief losses
                if t_lost < LOST_DELAY:
                    px.forward(FORWARD_SPEED)
                    continue

                # TRUE LINE LOSS → REVERSE
                print(f"LINE LOST ({t_lost:.2f}s) → REVERSING")

                px.set_dir_servo_angle(last_steer)
                px.backward(REVERSE_SPEED)
                sleep(t_lost)

                px.stop()
                ctrl.reset()

            print(
                f"adc={[round(x,1) for x in v]} | "
                f"err={err:+.3f} | "
                f"steer={last_steer:+.1f}"
            )

            sleep(CONTROL_DT)

    except KeyboardInterrupt:
        print("\nStopping...")
        px.stop()
        sleep(0.1)
