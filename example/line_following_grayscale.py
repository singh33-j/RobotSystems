from time import sleep, time
from picarx import Picarx

try:
    from robot_hat import ADC
except ImportError:
    from sim_robot_hat import ADC


# ============================================================
# CONFIGURATION (TUNE THESE)
# ============================================================

FILTER_ALPHA = 0.6          # ADC low-pass filter
CONTROL_DT   = 0.01         # control loop dt

FORWARD_SPEED = 4
REVERSE_SPEED = 3

BRIGHTNESS_DROP_THRESH = 400    # absolute drop in avg ADC
LOSS_TIME = 0.5                 # seconds brightness must stay low


# ============================================================
# SENSOR
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
# INTERPRETER (EDGE → ERROR ONLY)
# ============================================================

class LineInterpreter:
    def compute_error(self, v):
        L, C, R = v

        # simple proportional steering: center minus average
        err = (R - L) / (abs(L) + abs(R) + 1e-6)

        return max(-1.0, min(1.0, err))


# ============================================================
# PD STEERING CONTROLLER
# ============================================================

class PDController:
    def __init__(self, Kp=18.0, Kd=2.0, max_angle=30.0):
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
    interp = LineInterpreter()
    ctrl   = PDController()

    # brightness reference (what "on line" looks like)
    ref_adc = sensor.read()
    ref_brightness = sum(ref_adc) / 3

    last_seen_time  = time()
    last_steer      = 0.0
    loss_start_time = None

    print(f"Reference brightness: {int(ref_brightness)}")

    try:
        while True:
            adc = sensor.read()
            avg_brightness = sum(adc) / 3

            brightness_drop = (ref_brightness - avg_brightness)

            # --------------------------------------------------
            # LINE LOST DETECTION (BRIGHTNESS ONLY)
            # --------------------------------------------------
            if brightness_drop > BRIGHTNESS_DROP_THRESH:

                if loss_start_time is None:
                    loss_start_time = time()

                if time() - loss_start_time > LOSS_TIME:
                    # ---- RECOVERY ----
                    lost_duration = time() - last_seen_time

                    px.stop()
                    px.set_dir_servo_angle(last_steer)

                    px.backward(REVERSE_SPEED)
                    sleep(lost_duration)

                    px.stop()
                    ctrl.reset()
                    loss_start_time = None
                    sleep(0.05)
                    continue

            else:
                # line visible again
                loss_start_time = None
                last_seen_time = time()

            # --------------------------------------------------
            # NORMAL TRACKING
            # --------------------------------------------------
            err = interp.compute_error(adc)
            steer = ctrl.step(err)

            px.set_dir_servo_angle(steer)
            px.forward(FORWARD_SPEED)

            last_steer = steer

            print(
                f"TRACK | adc={list(map(int, adc))} | "
                f"drop={int(brightness_drop)} | "
                f"steer={steer:+.1f}"
            )

            sleep(CONTROL_DT)

    except KeyboardInterrupt:
        print("\nStopping...")
        px.stop()
        sleep(0.1)
