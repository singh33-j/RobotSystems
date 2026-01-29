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
EDGE_THRESH  = 0.05
CONTROL_DT   = 0.01

FORWARD_SPEED = 4
REVERSE_SPEED = 3

LOSS_CONFIRM_TIME = 1.0   # <<< REQUIRED: seconds without line


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
            self.filt[i] = (
                FILTER_ALPHA * raw[i]
                + (1 - FILTER_ALPHA) * self.filt[i]
            )
        return self.filt.copy()


# ============================================================
# INTERPRETATION — EDGE BASED
# ============================================================

class LineInterpreter:
    def __init__(self, polarity='dark'):
        self.polarity = polarity

    def compute_error(self, v):
        L, C, R = v

        dLC = C - L
        dCR = R - C

        if self.polarity == 'light':
            dLC = -dLC
            dCR = -dCR

        contrast = abs(L - C) + abs(C - R) + 1e-6
        dLC /= contrast
        dCR /= contrast

        if max(abs(dLC), abs(dCR)) < EDGE_THRESH:
            return None

        if abs(dLC) > abs(dCR):
            e = +dLC
        else:
            e = -dCR

        return max(-1.0, min(1.0, 0.7 * e))

    def weak_contrast(self, v):
        return max(v) - min(v) < 20


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
        u = self.Kp * e + self.Kd * de

        u = max(-self.max, min(self.max, u))

        self.e_last = e
        self.t_last = now
        return u

    def reset(self):
        self.e_last = 0.0
        self.t_last = time()


# ============================================================
# MAIN LOOP (LOSS-DEBOUNCED)
# ============================================================

if __name__ == "__main__":

    px = Picarx()
    sensor = LineSensor()
    interp = LineInterpreter(polarity='dark')
    ctrl   = PDController()

    last_valid_time   = time()
    last_valid_steer  = 0.0
    loss_start_time   = None

    try:
        while True:
            v = sensor.read()
            err = interp.compute_error(v)

            # ---------------- LINE PRESENT ----------------
            if err is not None and not interp.weak_contrast(v):
                loss_start_time = None

                steer = ctrl.step(err)
                px.set_dir_servo_angle(steer)
                px.forward(FORWARD_SPEED)

                last_valid_time  = time()
                last_valid_steer = steer

            # ---------------- POSSIBLE LINE LOSS ----------------
            else:
                if loss_start_time is None:
                    loss_start_time = time()

                # Not lost long enough → keep last command
                if time() - loss_start_time < LOSS_CONFIRM_TIME:
                    px.set_dir_servo_angle(last_valid_steer)
                    px.forward(FORWARD_SPEED)
                    sleep(CONTROL_DT)
                    continue

                # ---------------- CONFIRMED LINE LOSS ----------------
                t_lost = time() - last_valid_time
                reverse_time = min(t_lost, 1.0)

                px.stop()
                px.set_dir_servo_angle(last_valid_steer)
                px.backward(REVERSE_SPEED)
                sleep(reverse_time)

                px.stop()
                ctrl.reset()
                loss_start_time = None
                sleep(0.05)
                continue

            print(
                f"adc={[round(x,1) for x in v]} | "
                f"err={None if err is None else f'{err:+.3f}'} | "
                f"steer={last_valid_steer:+.1f}"
            )

            sleep(CONTROL_DT)

    except KeyboardInterrupt:
        print("\nStopping...")
        px.stop()
        sleep(0.1)
