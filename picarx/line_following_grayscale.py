from time import sleep, time
from picarx_improved import Picarx

try:
    from robot_hat import ADC
except ImportError:
    from sim_robot_hat import ADC


# ============================================================
# CONFIGURATION
# ============================================================

FILTER_ALPHA = 0.7
CONTROL_DT   = 0.01

STARTUP_STRAIGHT_TIME = 0.2
LINE_LOSS_ENABLE_TIME = 1.0

DROP_SUM_THRESH     = 400
REACQUIRE_THRESH    = 100     # <<< hysteresis lower bound
REACQUIRE_COUNT_REQ = 5       # <<< consecutive samples

# --- Steering shaping ---
ERROR_GAIN         = 8.0
ERROR_DEADZONE     = 0.05
STEER_DEADZONE_DEG = 1.5
MAX_STEER_DEG      = 30.0

# --- Recovery ---
RECOVER_REVERSE_SPEED   = 15
RECOVER_OVERSTEER_GAIN  = 1.3
RECOVER_OVERSTEER_TIME  = 0.25
RECOVER_SCAN_TIME       = 0.6
RECOVER_MAX_TIME        = 2.0


# ============================================================
# SENSING
# ============================================================

class LineSensor:
    def __init__(self, pins=['A0','A1','A2']):
        self.adc = [ADC(p) for p in pins]
        self.filt = [0.0, 0.0, 0.0]
        self.prev = None

    def read(self):
        raw = [a.read() for a in self.adc]
        for i in range(3):
            self.filt[i] = (
                FILTER_ALPHA * raw[i]
                + (1 - FILTER_ALPHA) * self.filt[i]
            )
        if self.prev is None:
            self.prev = self.filt.copy()
        return self.filt.copy()

    def brightness_drop(self, v):
        drops = [
            max(0.0, self.prev[i] - v[i])
            for i in range(3)
        ]
        self.prev = v.copy()
        return drops


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

    def line_lost(self, drop_sum):
        return drop_sum > DROP_SUM_THRESH

    def line_reacquired(self, drop_sum):
        return drop_sum < REACQUIRE_THRESH


# ============================================================
# CONTROLLER
# ============================================================

class PDController:
    def __init__(self, Kp=18.0, Kd=4.0, max_angle=MAX_STEER_DEG):
        self.Kp = Kp
        self.Kd = Kd
        self.max = max_angle
        self.e_last = 0.0
        self.d_filt = 0.0
        self.t_last = time()

    def step(self, e):
        now = time()
        dt = max(now - self.t_last, 1e-4)

        de = (e - self.e_last) / dt
        self.d_filt = 0.7 * self.d_filt + 0.3 * de

        u = self.Kp * e + self.Kd * self.d_filt
        return max(-self.max, min(self.max, u))

    def reset(self):
        self.e_last = 0.0
        self.d_filt = 0.0
        self.t_last = time()


# ============================================================
# MAIN LOOP
# ============================================================

if __name__ == "__main__":

    px     = Picarx()
    sensor = LineSensor()
    interp = LineInterpreter(polarity='dark')
    ctrl   = PDController()

    start_time    = time()
    last_steer    = 0.0

    recover_state = 0   # 0=tracking, 1=oversteer, 2=scan
    recover_start = 0.0
    scan_start    = 0.0

    reacquire_cnt = 0

    try:
        while True:

            elapsed = time() - start_time
            v = sensor.read()
            drops = sensor.brightness_drop(v)
            drop_sum = sum(drops)

            # --------------------------------------------------
            # STARTUP
            # --------------------------------------------------
            if elapsed < STARTUP_STRAIGHT_TIME:
                px.set_dir_servo_angle(0)
                px.forward(20)
                ctrl.reset()
                sleep(CONTROL_DT)
                continue

            lost = (
                elapsed > LINE_LOSS_ENABLE_TIME and
                interp.line_lost(drop_sum)
            )

            # --------------------------------------------------
            # RECOVERY MODE
            # --------------------------------------------------
            if recover_state != 0 or lost:

                now = time()

                if recover_state == 0:
                    recover_state = 1
                    recover_start = now
                    scan_start = now
                    reacquire_cnt = 0

                # ---------- reacquire detection ----------
                if interp.line_reacquired(drop_sum):
                    reacquire_cnt += 1
                    if reacquire_cnt >= REACQUIRE_COUNT_REQ:
                        recover_state = 0
                        ctrl.reset()
                        continue
                else:
                    reacquire_cnt = 0
                # ----------------------------------------

                if now - recover_start > RECOVER_MAX_TIME:
                    px.set_dir_servo_angle(0)
                    px.backward(RECOVER_REVERSE_SPEED)
                    sleep(0.3)
                    recover_state = 0
                    ctrl.reset()
                    continue

                # Phase 1: oversteer reverse
                if recover_state == 1:
                    steer = max(
                        -MAX_STEER_DEG,
                        min(MAX_STEER_DEG,
                            RECOVER_OVERSTEER_GAIN * last_steer)
                    )
                    px.set_dir_servo_angle(steer)
                    px.backward(RECOVER_REVERSE_SPEED)

                    if now - recover_start > RECOVER_OVERSTEER_TIME:
                        recover_state = 2
                        scan_start = now

                # Phase 2: arc scan reverse
                elif recover_state == 2:
                    t = min(1.0, (now - scan_start) / RECOVER_SCAN_TIME)
                    sweep = 15.0 * t
                    steer = last_steer + sweep * (-1 if last_steer > 0 else 1)
                    steer = max(-MAX_STEER_DEG, min(MAX_STEER_DEG, steer))

                    px.set_dir_servo_angle(steer)
                    px.backward(RECOVER_REVERSE_SPEED)

                sleep(CONTROL_DT)
                continue

            # --------------------------------------------------
            # NORMAL TRACKING
            # --------------------------------------------------
            err   = interp.compute_error(v)
            steer = ctrl.step(err)

            if abs(steer) < STEER_DEADZONE_DEG:
                steer = 0.0

            last_steer = steer

            px.set_dir_servo_angle(steer)
            px.forward(20)

            print(
                f"TRACK | adc={[round(x) for x in v]} | "
                f"drop_sum={round(drop_sum)} | "
                f"err={err:+.3f} | steer={steer:+.1f}"
            )

            sleep(CONTROL_DT)

    except KeyboardInterrupt:
        px.stop()
        sleep(0.1)
