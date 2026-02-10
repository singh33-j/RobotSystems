from time import sleep, time
from threading import Event
from concurrent.futures import ThreadPoolExecutor

# (c) header import per manual
from readerwriterlock import rwlock

from picarx_improved import Picarx

try:
    from robot_hat import ADC
except ImportError:
    from sim_robot_hat import ADC


# ============================================================
# CONFIGURATION
# ============================================================

FILTER_ALPHA = 0.7

# Keep these modest to reduce I2C half-duplex collisions (manual note)
SENSOR_DT       = 0.01
INTERPRETER_DT  = 0.01
CONTROL_DT      = 0.01

STARTUP_STRAIGHT_TIME = 0.2
LINE_LOSS_ENABLE_TIME = 1.0

DROP_SUM_THRESH     = 400
REACQUIRE_THRESH    = 100
REACQUIRE_COUNT_REQ = 5

ERROR_GAIN         = 10.0
STEER_DEADZONE_DEG = 1.5
MAX_STEER_DEG      = 30.0

RECOVER_REVERSE_SPEED  = 15
RECOVER_OVERSTEER_GAIN = 1.3
RECOVER_OVERSTEER_TIME = 0.25
RECOVER_SCAN_TIME      = 0.6
RECOVER_MAX_TIME       = 2.0


# ============================================================
# BUS (broadcast, latest-value) WITH LOCKING
# ============================================================

class Bus:
    def __init__(self, initial=None):
        self.message = initial
        # (d) writer-priority RW lock per manual
        self.lock = rwlock.RWLockWriteD()

    def write(self, message):
        # (e) write lock per manual
        with self.lock.gen_wlock():
            self.message = message

    def read(self):
        # (e) read lock per manual
        with self.lock.gen_rlock():
            message = self.message
        return message


# ============================================================
# SENSOR (producer)
# ============================================================

class LineSensor:
    def __init__(self, pins=['A0', 'A1', 'A2']):
        self.adc  = [ADC(p) for p in pins]
        self.filt = [0.0, 0.0, 0.0]
        self.prev = None

    def read(self):
        raw = [a.read() for a in self.adc]
        for i in range(3):
            self.filt[i] = FILTER_ALPHA * raw[i] + (1 - FILTER_ALPHA) * self.filt[i]
        if self.prev is None:
            self.prev = self.filt.copy()
        return self.filt.copy()

    def brightness_drop_sum(self, v):
        drops = [max(0.0, self.prev[i] - v[i]) for i in range(3)]
        self.prev = v.copy()
        return sum(drops)


def sensor_task(sensor_bus, shutdown_event, dt):
    sensor = LineSensor()
    while not shutdown_event.is_set():
        v = sensor.read()
        drop_sum = sensor.brightness_drop_sum(v)
        # publish latest snapshot
        sensor_bus.write((v, drop_sum))
        sleep(dt)


# ============================================================
# INTERPRETER (consumer–producer)
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
        return ERROR_GAIN * e

    def line_lost(self, drop_sum):
        return drop_sum > DROP_SUM_THRESH

    def line_reacquired(self, drop_sum):
        return drop_sum < REACQUIRE_THRESH


def interpreter_task(sensor_bus, interp_bus, shutdown_event, dt):
    interp = LineInterpreter(polarity='dark')
    start_time = time()

    while not shutdown_event.is_set():
        data = sensor_bus.read()
        if data is None:
            sleep(dt)
            continue

        v, drop_sum = data
        elapsed = time() - start_time

        err = interp.compute_error(v)
        lost = (elapsed > LINE_LOSS_ENABLE_TIME) and interp.line_lost(drop_sum)

        interp_bus.write({
            "error": err,
            "drop_sum": drop_sum,
            "lost": lost,
            "adc": v
        })

        sleep(dt)


# ============================================================
# CONTROLLER (consumer)
# ============================================================

class PDController:
    def __init__(self, Kp=25.0, Kd=2.0):
        self.Kp = Kp
        self.Kd = Kd
        self.e_last = 0.0
        self.d_filt = 0.0
        self.t_last = time()

    def step(self, e):
        now = time()
        dt = max(now - self.t_last, 1e-4)
        de = (e - self.e_last) / dt
        self.d_filt = 0.7 * self.d_filt + 0.3 * de
        self.e_last = e
        self.t_last = now
        return self.Kp * e + self.Kd * self.d_filt

    def reset(self):
        self.e_last = 0.0
        self.d_filt = 0.0
        self.t_last = time()


def control_task(interp_bus, shutdown_event, dt):
    px = Picarx()
    ctrl = PDController()

    start_time = time()
    last_steer = 0.0

    recover_state = 0
    recover_start = 0.0
    scan_start = 0.0
    reacquire_cnt = 0

    try:
        while not shutdown_event.is_set():
            data = interp_bus.read()
            if data is None:
                sleep(dt)
                continue

            err = data["error"]
            lost = data["lost"]
            drop_sum = data["drop_sum"]
            adc = data["adc"]
            elapsed = time() - start_time

            # ---------------- STARTUP ----------------
            if elapsed < STARTUP_STRAIGHT_TIME:
                px.set_dir_servo_angle(0)
                px.forward(28)
                ctrl.reset()
                sleep(dt)
                continue

            # ---------------- RECOVERY ----------------
            if recover_state != 0 or lost:
                now = time()

                if recover_state == 0:
                    recover_state = 1
                    recover_start = now
                    scan_start = now
                    reacquire_cnt = 0

                # reacquire hysteresis
                if drop_sum < REACQUIRE_THRESH:
                    reacquire_cnt += 1
                    if reacquire_cnt >= REACQUIRE_COUNT_REQ:
                        recover_state = 0
                        ctrl.reset()
                        continue
                else:
                    reacquire_cnt = 0

                if now - recover_start > RECOVER_MAX_TIME:
                    px.set_dir_servo_angle(0)
                    px.backward(RECOVER_REVERSE_SPEED)
                    sleep(0.3)
                    recover_state = 0
                    ctrl.reset()
                    continue

                # Phase 1: oversteer reverse
                if recover_state == 1:
                    steer = RECOVER_OVERSTEER_GAIN * last_steer
                    steer = max(-MAX_STEER_DEG, min(MAX_STEER_DEG, steer))
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

                sleep(dt)
                continue

            # ---------------- TRACKING ----------------
            steer = ctrl.step(err)
            steer = max(-MAX_STEER_DEG, min(MAX_STEER_DEG, steer))
            if abs(steer) < STEER_DEADZONE_DEG:
                steer = 0.0

            last_steer = steer
            px.set_dir_servo_angle(steer)
            px.forward(28)

            print(
                f"TRACK | adc={[round(x) for x in adc]} | "
                f"drop_sum={round(drop_sum)} | "
                f"err={err:+.3f} | steer={steer:+.1f}"
            )

            sleep(dt)

    finally:
        px.stop()


# ============================================================
# MAIN (ThreadPoolExecutor + graceful shutdown)
# ============================================================

def handle_exception(future):
    e = future.exception()
    if e:
        print(f"Exception in worker thread: {e}")


if __name__ == "__main__":
    shutdown_event = Event()

    sensor_bus = Bus(initial=None)
    interp_bus = Bus(initial=None)

    futures = []
    with ThreadPoolExecutor(max_workers=3) as executor:
        futures.append(executor.submit(sensor_task, sensor_bus, shutdown_event, SENSOR_DT))
        futures.append(executor.submit(interpreter_task, sensor_bus, interp_bus, shutdown_event, INTERPRETER_DT))
        futures.append(executor.submit(control_task, interp_bus, shutdown_event, CONTROL_DT))

        for f in futures:
            f.add_done_callback(handle_exception)

        try:
            while not shutdown_event.is_set():
                sleep(0.5)
        except KeyboardInterrupt:
            print("Shutting down")
            shutdown_event.set()
        finally:
            executor.shutdown(wait=True)
