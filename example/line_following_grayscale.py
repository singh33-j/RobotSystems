from time import sleep, time
from picarx import Picarx

try:
    from robot_hat import ADC
except ImportError:
    from sim_robot_hat import ADC


# ============================================================
# CONFIG
# ============================================================

CONTROL_DT     = 0.01

FORWARD_SPEED  = 4
REVERSE_SPEED  = 3

STEER_MAX      = 25.0
KP             = 18.0

DROP_THRESHOLD = 600        # sum of brightness drops
SPREAD_THRESH  = 150        # minimum structure to steer
STOP_TIME     = 1.0         # pause before reverse


# ============================================================
# SENSOR
# ============================================================

class LineSensor:
    def __init__(self, pins=('A0','A1','A2')):
        self.adc = [ADC(p) for p in pins]
        self.last = None

    def read(self):
        v = [a.read() for a in self.adc]

        if self.last is None:
            drops = [0, 0, 0]
        else:
            drops = [max(0, self.last[i] - v[i]) for i in range(3)]

        self.last = v.copy()
        return v, drops


# ============================================================
# INTERPRETER
# ============================================================

class LineInterpreter:
    def compute(self, v, drops):
        L, C, R = v

        # ---------- LINE LOSS (brightness drop) ----------
        drop_sum = sum(drops)
        line_lost = drop_sum >= DROP_THRESHOLD

        # ---------- STRUCTURE CHECK (no edge → no steering) ----------
        spread = max(v) - min(v)
        if spread < SPREAD_THRESH:
            err = 0.0
        else:
            err = (L - R) / spread
            err = max(-1.0, min(1.0, err))

        return err, line_lost, drop_sum


# ============================================================
# CONTROLLER
# ============================================================

class SteeringController:
    def step(self, err):
        steer = KP * err
        steer = max(-STEER_MAX, min(STEER_MAX, steer))
        return steer


# ============================================================
# MAIN LOOP
# ============================================================

if __name__ == "__main__":

    px      = Picarx()
    sensor = LineSensor()
    interp = LineInterpreter()
    ctrl   = SteeringController()

    try:
        while True:
            adc, drops = sensor.read()
            err, line_lost, drop_sum = interp.compute(adc, drops)

            if line_lost:
                print(
                    f"REVERSE | adc={adc} | drops={drops} | "
                    f"drop_sum={drop_sum} | line_lost=True"
                )

                px.stop()
                sleep(STOP_TIME)

                px.backward(REVERSE_SPEED)
                sleep(0.4)

                px.stop()
                sleep(0.1)
                continue

            steer = ctrl.step(err)

            px.set_dir_servo_angle(steer)
            px.forward(FORWARD_SPEED)

            print(
                f"TRACK | adc={adc} | drops={drops} | "
                f"drop_sum={drop_sum} | line_lost=False | "
                f"err={err:+.3f} | steer={steer:+.1f}"
            )

            sleep(CONTROL_DT)

    except KeyboardInterrupt:
        print("\nStopping...")
        px.stop()
