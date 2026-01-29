from time import sleep
from picarx import Picarx

try:
    from robot_hat import ADC
except ImportError:
    from sim_robot_hat import ADC


# ============================================================
# CONFIGURATION
# ============================================================

FILTER_ALPHA   = 0.7
CONTROL_DT     = 0.01

FORWARD_SPEED  = 14
REVERSE_SPEED  = 14

DROP_THRESHOLD = 600   # sum of brightness drops
STOP_TIME      = 1.0   # seconds to stop before reversing


# ============================================================
# SENSOR
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
            drops = [0, 0, 0]
        else:
            drops = [
                max(0, self.prev[i] - self.filt[i])
                for i in range(3)
            ]

        self.prev = self.filt.copy()
        return self.filt.copy(), drops


# ============================================================
# INTERPRETER
# ============================================================

class LineInterpreter:
    def compute_error(self, v):
        L, C, R = v
        denom = abs(L - R) + 1e-6
        return (L - R) / denom


# ============================================================
# MAIN
# ============================================================

if __name__ == "__main__":

    px = Picarx()
    sensor = LineSensor()
    interp = LineInterpreter()

    last_steer = 0.0
    reversing = False

    try:
        while True:

            adc, drops = sensor.read()
            drop_sum = sum(drops)
            line_lost = drop_sum >= DROP_THRESHOLD

            if line_lost:
                # ---------- STOP ----------
                px.stop()

                print(
                    f"STOP | adc={[int(x) for x in adc]} "
                    f"| drops={[int(d) for d in drops]} "
                    f"| drop_sum={int(drop_sum)} "
                    f"| line_lost={line_lost}"
                )

                sleep(STOP_TIME)

                # ---------- REVERSE ----------
                px.set_dir_servo_angle(last_steer)
                px.backward(REVERSE_SPEED)

                print(
                    f"REVERSE | steer={last_steer:+.1f}"
                )

                sleep(CONTROL_DT)
                continue

            # ---------- NORMAL TRACKING ----------
            err = interp.compute_error(adc)
            steer = max(-30.0, min(30.0, 25.0 * err))

            px.set_dir_servo_angle(steer)
            px.forward(FORWARD_SPEED)

            last_steer = steer

            print(
                f"TRACK | adc={[int(x) for x in adc]} "
                f"| drops={[int(d) for d in drops]} "
                f"| drop_sum={int(drop_sum)} "
                f"| line_lost={line_lost} "
                f"| err={err:+.3f} "
                f"| steer={steer:+.1f}"
            )

            sleep(CONTROL_DT)

    except KeyboardInterrupt:
        print("\nStopping")
        px.stop()
        sleep(0.1)
