from time import sleep, time
from picarx import Picarx

try:
    from robot_hat import ADC
except ImportError:
    from sim_robot_hat import ADC


# ============================================================
# CONFIG
# ============================================================

FILTER_ALPHA = 0.7
EDGE_THRESH  = 0.05
CONTROL_DT   = 0.01

FORWARD_SPEED = 4

STARTUP_STRAIGHT_TIME = 0.2   # <<< THIS IS THE FIX


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
# INTERPRETER
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
            return 0.0

        if abs(dLC) > abs(dCR):
            return max(-1.0, min(1.0, 0.7 * dLC))
        else:
            return max(-1.0, min(1.0, -0.7 * dCR))


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
# MAIN
# ============================================================

if __name__ == "__main__":

    px     = Picarx()
    sensor = LineSensor()
    interp = LineInterpreter()
    ctrl   = PDController()

    start_time = time()

    try:
        while True:

            # ---------------- STARTUP STRAIGHT ----------------
            if time() - start_time < STARTUP_STRAIGHT_TIME:
                px.set_dir_servo_angle(0)
                px.forward(FORWARD_SPEED)
                ctrl.reset()

                print("STARTUP | steering=0")
                sleep(CONTROL_DT)
                continue
            # --------------------------------------------------

            v   = sensor.read()
            err = interp.compute_error(v)
            steer = ctrl.step(err)

            px.set_dir_servo_angle(steer)
            px.forward(FORWARD_SPEED)

            print(
                f"TRACK | adc={[round(x) for x in v]} | "
                f"err={err:+.3f} | steer={steer:+.1f}"
            )

            sleep(CONTROL_DT)

    except KeyboardInterrupt:
        px.stop()
        sleep(0.1)
