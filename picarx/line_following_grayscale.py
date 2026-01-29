from time import sleep, time
from picarx_improved import Picarx

try:
    from robot_hat import ADC
except ImportError:
    from sim_robot_hat import ADC


# ============================================================
# CONFIGURATION
# ============================================================

CONTROL_DT = 0.01
STARTUP_STRAIGHT_TIME = 0.2

# --- Line loss detection via spread ---
SPREAD_LOST  = 25    # enter lost if sensors agree
SPREAD_FOUND = 45    # exit lost if sensors disagree

# --- Steering ---
ERROR_GAIN         = 8.0
ERROR_DEADZONE     = 0.05
STEER_DEADZONE_DEG = 1.5
MAX_STEER_DEG      = 30.0

# --- Recovery ---
RECOVER_SPEED      = 15
RECOVER_MAX_TIME   = 2.0


# ============================================================
# SENSING
# ============================================================

class LineSensor:
    def __init__(self, pins=['A0','A1','A2']):
        self.adc = [ADC(p) for p in pins]

    def read(self):
        return [a.read() for a in self.adc]


# ============================================================
# INTERPRETATION
# ============================================================

class LineInterpreter:
    def __init__(self, polarity='dark'):
        self.polarity = polarity

    def compute_error(self, v):
        L, C, R = v
        denom = abs(L - C) + abs(R - C) + 1e-6
        e = (R - L) / denom

        if self.polarity == 'light':
            e = -e

        e *= ERROR_GAIN
        if abs(e) < ERROR_DEADZONE:
            e = 0.0

        return max(-1.0, min(1.0, e))


# ============================================================
# CONTROLLER
# ============================================================

class PDController:
    def __init__(self, Kp=18.0, Kd=4.0):
        self.Kp = Kp
        self.Kd = Kd
        self.e_last = 0.0
        self.t_last = time()

    def step(self, e):
        now = time()
        dt = max(now - self.t_last, 1e-4)
        de = (e - self.e_last) / dt
        u = self.Kp * e + self.Kd * de
        self.e_last = e
        self.t_last = now
        return max(-MAX_STEER_DEG, min(MAX_STEER_DEG, u))

    def reset(self):
        self.e_last = 0.0
        self.t_last = time()


# ============================================================
# MAIN LOOP
# ============================================================

if __name__ == "__main__":

    px     = Picarx()
    sensor = LineSensor()
    interp = LineInterpreter(polarity='dark')
    ctrl   = PDController()

    start_time = time()
    last_steer = 0.0
    is_lost    = False
    lost_t0    = 0.0

    try:
        while True:

            elapsed = time() - start_time
            v = sensor.read()
            spread = max(v) - min(v)

            # ---------------- STARTUP ----------------
            if elapsed < STARTUP_STRAIGHT_TIME:
                px.set_dir_servo_angle(0)
                px.forward(20)
                ctrl.reset()
                last_steer = 0.0
                sleep(CONTROL_DT)
                continue

            # ------------- LOSS DETECTION -------------
            if not is_lost:
                if spread < SPREAD_LOST:
                    is_lost = True
                    lost_t0 = time()
            else:
                if spread > SPREAD_FOUND:
                    is_lost = False
                    ctrl.reset()
                    continue

            # ---------------- LOST -------------------
            if is_lost:
                px.set_dir_servo_angle(last_steer)
                px.backward(RECOVER_SPEED)

                print(
                    f"LOST  | adc={v} | "
                    f"spread={spread:.1f} | "
                    f"last_steer={last_steer:+.1f}"
                )

                if time() - lost_t0 > RECOVER_MAX_TIME:
                    last_steer = -last_steer
                    lost_t0 = time()

                sleep(CONTROL_DT)
                continue

            # ---------------- TRACK ------------------
            err = interp.compute_error(v)
            steer = ctrl.step(err)

            if abs(steer) < STEER_DEADZONE_DEG:
                steer = 0.0

            last_steer = steer

            px.set_dir_servo_angle(steer)
            px.forward(20)

            print(
                f"TRACK | adc={v} | "
                f"spread={spread:.1f} | "
                f"err={err:+.3f} | "
                f"steer={steer:+.1f}"
            )

            sleep(CONTROL_DT)

    except KeyboardInterrupt:
        px.stop()
        sleep(0.1)
