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
CONTROL_DT = 0.01

FORWARD_SPEED = 4
REVERSE_SPEED = 3

DROP_THRESH = 250        # brightness drop per sensor to trigger loss
LOSS_DELAY  = 0.5        # seconds before reversing


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
# EDGE-BASED INTERPRETER (USED ONLY WHEN LINE IS VALID)
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

        contrast = abs(dLC) + abs(dCR) + 1e-6
        e = (dLC - dCR) / contrast

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

    prev_adc = None

    last_valid_steer = 0.0

    loss_start_time = None

    try:
        while True:
            now = time()
            adc = sensor.read()

            # ---------- BRIGHTNESS DROP DETECTION ----------
            drops = [0, 0, 0]
            if prev_adc is not None:
                drops = [max(0, prev_adc[i] - adc[i]) for i in range(3)]

            big_drop = all(d > DROP_THRESH for d in drops)

            # ---------- LOSS TIMING (LATCHED) ----------
            if big_drop:
                if loss_start_time is None:
                    loss_start_time = now
            else:
                loss_start_time = None

            loss_time = 0.0
            if loss_start_time is not None:
                loss_time = now - loss_start_time

            line_lost = loss_time >= LOSS_DELAY

            # ---------- LINE LOST → REVERSE ----------
            if line_lost:
                px.stop()
                px.set_dir_servo_angle(last_valid_steer)
                px.backward(REVERSE_SPEED)
                sleep(loss_time)
                px.stop()
                ctrl.reset()
                loss_start_time = None
                prev_adc = None
                continue

            # ---------- NORMAL TRACKING ----------
            err = interp.compute_error(adc)
            steer = ctrl.step(err)

            px.set_dir_servo_angle(steer)
            px.forward(FORWARD_SPEED)

            last_valid_steer = steer
            prev_adc = adc.copy()

            # ---------- DEBUG ----------
            print(
                f"TRACK | adc={[int(x) for x in adc]} | "
                f"drops={[int(d) for d in drops]} | "
                f"line_lost={line_lost} | "
                f"loss_time={loss_time:.3f}s | "
                f"err={err:+.3f} | "
                f"steer={steer:+.1f}"
            )

            sleep(CONTROL_DT)

    except KeyboardInterrupt:
        print("\nStopping...")
        px.stop()
        sleep(0.1)
