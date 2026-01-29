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
EDGE_THRESH  = 0.05          # sensitivity to subtle edges
CONTROL_DT  = 0.01

FORWARD_SPEED = 4
REVERSE_SPEED = 3

LINE_LOST_DELAY = 0.5        # seconds before reversing


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
# INTERPRETATION — EDGE-BASED WITH CONFIDENCE
# ============================================================

class LineInterpreter:
    def __init__(self, polarity='dark'):
        self.polarity = polarity

    def compute_error(self, v):
        L, C, R = v

        # Adjacent differences = edges
        dLC = C - L
        dCR = R - C

        if self.polarity == 'light':
            dLC = -dLC
            dCR = -dCR

        # Normalize by local contrast
        contrast = abs(L - C) + abs(C - R) + 1e-6
        dLC /= contrast
        dCR /= contrast

        edge_strength = max(abs(dLC), abs(dCR))

        # Edge unreliable
        if edge_strength < EDGE_THRESH:
            return 0.0, False

        # Choose dominant edge
        if abs(dLC) > abs(dCR):
            e = +dLC        # line on left → steer right
        else:
            e = -dCR        # line on right → steer left

        e = max(-1.0, min(1.0, 0.7 * e))
        return e, True


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
# MAIN LOOP
# ============================================================

if __name__ == "__main__":

    px = Picarx()
    sensor = LineSensor()
    interp = LineInterpreter(polarity='dark')
    ctrl   = PDController()

    last_valid_time  = time()
    last_valid_steer = 0.0

    try:
        while True:
            now = time()
            v = sensor.read()

            err, valid = interp.compute_error(v)

            # -------- LINE UNRELIABLE / LOST --------
            if not valid:
                t_lost = now - last_valid_time

                # grace period — do nothing yet
                if t_lost < LINE_LOST_DELAY:
                    px.forward(FORWARD_SPEED)
                    sleep(CONTROL_DT)
                    continue

                # confirmed loss → reverse
                px.stop()
                px.set_dir_servo_angle(last_valid_steer)

                px.backward(REVERSE_SPEED)
                sleep(t_lost)

                px.stop()
                ctrl.reset()

                last_valid_time = now
                sleep(0.05)
                continue

            # -------- NORMAL TRACKING --------
            steer = ctrl.step(err)

            px.set_dir_servo_angle(steer)
            px.forward(FORWARD_SPEED)

            last_valid_time  = now
            last_valid_steer = steer

            print(
                f"adc={[round(x,1) for x in v]} | "
                f"valid={valid} | "
                f"err={err:+.3f} | "
                f"steer={steer:+.1f}"
            )

            sleep(CONTROL_DT)

    except KeyboardInterrupt:
        print("\nStopping...")
        px.stop()
        sleep(0.1)
