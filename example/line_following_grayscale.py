from time import sleep, time
from picarx import Picarx

try:
    from robot_hat import ADC
except ImportError:
    from sim_robot_hat import ADC


# ============================================================
# CONFIGURATION
# ============================================================

FILTER_ALPHA = 0.7
CONTROL_DT   = 0.005

FORWARD_SPEED = 4
REVERSE_SPEED = 3

# Line-loss detection
DROP_THRESH   = 300      # brightness drop per sensor
LOSS_TIME_REQ = 0.2      # seconds line must be lost


# ============================================================
# SENSING
# ============================================================

class LineSensor:
    def __init__(self, pins=['A0','A1','A2']):
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
# ERROR COMPUTATION (simple edge steering)
# ============================================================

class LineInterpreter:
    def compute_error(self, v):
        L, C, R = v

        dLC = C - L
        dCR = R - C

        if abs(dLC) < 1e-6 and abs(dCR) < 1e-6:
            return 0.0

        if abs(dLC) > abs(dCR):
            e = +dLC
        else:
            e = -dCR

        e /= (abs(L) + abs(C) + abs(R) + 1e-6)
        return max(-1.0, min(1.0, 0.7 * e))


# ============================================================
# PD CONTROLLER
# ============================================================

class PDController:
    def __init__(self, Kp=14.0, Kd=2.0, max_angle=30.0):
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
        u  = max(-self.max, min(self.max, u))

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

    px     = Picarx()
    sensor = LineSensor()
    interp = LineInterpreter()
    ctrl   = PDController()

    prev_v = sensor.read()

    loss_start = None
    last_steer = 0.0

    try:
        while True:
            v = sensor.read()

            # --- Brightness drop detection ---
            drops = [prev_v[i] - v[i] for i in range(3)]
            line_lost = all(d > DROP_THRESH for d in drops)

            # --- Track loss timing ---
            if line_lost:
                if loss_start is None:
                    loss_start = time()
            else:
                loss_start = None

            loss_time = (time() - loss_start) if loss_start else 0.0

            # =============================
            # LINE LOST → REVERSE
            # =============================
            if loss_time >= LOSS_TIME_REQ:
                px.stop()
                px.set_dir_servo_angle(last_steer)

                px.backward(REVERSE_SPEED)
                sleep(loss_time)   # reverse same duration as loss

                px.stop()
                ctrl.reset()
                loss_start = None
                sleep(0.05)
                continue

            # =============================
            # NORMAL TRACKING
            # =============================
            err   = interp.compute_error(v)
            steer = ctrl.step(err)

            px.set_dir_servo_angle(steer)
            px.forward(FORWARD_SPEED)

            last_steer = steer
            prev_v     = v.copy()

            print(
                f"TRACK | "
                f"adc={[int(x) for x in v]} | "
                f"drops={[int(d) for d in drops]} | "
                f"line_lost={line_lost} | "
                f"loss_time={loss_time:.3f}s | "
                f"steer={steer:+.1f}"
            )

            sleep(CONTROL_DT)

    except KeyboardInterrupt:
        print("\nStopping...")
        px.stop()
        sleep(0.1)
