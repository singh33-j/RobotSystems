"""
Robust Line Following for PiCar-X
Edge-based interpretation + Adaptive PD control

Instruction-compliant:
- Detects sharp changes between adjacent sensors (edges)
- Determines left/right + how far off-center
- Robust to lighting changes
- Adaptive gains for smooth + fast steering
"""

from time import sleep, time
from picarx import Picarx

try:
    from robot_hat import ADC
except ImportError:
    from sim_robot_hat import ADC


# ============================================================
# CONFIGURATION
# ============================================================
FILTER_ALPHA = 0.6        # sensor smoothing (higher = faster)
EDGE_THRESH  = 60.0       # minimum contrast to consider an edge
MAX_STEER    = 30.0
PX_POWER     = 10


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
                FILTER_ALPHA * raw[i] +
                (1 - FILTER_ALPHA) * self.filt[i]
            )
        return self.filt.copy()


# ============================================================
# INTERPRETATION — EDGE DETECTION (INSTRUCTION STYLE)
# ============================================================
class LineInterpreter:
    def __init__(self, polarity='dark'):
        self.polarity = polarity

    def compute_error(self, v):
        """
        Returns signed error in [-1, 1]
        Positive  -> line on LEFT -> steer RIGHT
        Negative  -> line on RIGHT -> steer LEFT
        """

        L, C, R = v

        # Edge magnitudes
        dLC = C - L
        dCR = R - C

        if self.polarity == 'light':
            dLC = -dLC
            dCR = -dCR

        # No edge detected
        if abs(dLC) < EDGE_THRESH and abs(dCR) < EDGE_THRESH:
            return 0.0

        # Left edge stronger
        if abs(dLC) > abs(dCR):
            mag = min(abs(dLC) / (EDGE_THRESH * 4), 1.0)
            return +mag

        # Right edge stronger
        else:
            mag = min(abs(dCR) / (EDGE_THRESH * 4), 1.0)
            return -mag

    def line_lost(self, v):
        spread = max(v) - min(v)
        return spread < EDGE_THRESH


# ============================================================
# ADAPTIVE PD CONTROLLER
# ============================================================
class AdaptivePDController:
    def __init__(
        self,
        Kp_min=4.0,
        Kp_max=22.0,
        Kd_min=1.0,
        Kd_max=12.0,
        max_angle=30.0
    ):
        self.Kp_min = Kp_min
        self.Kp_max = Kp_max
        self.Kd_min = Kd_min
        self.Kd_max = Kd_max
        self.max = max_angle

        self.e_last = 0.0
        self.t_last = time()

    def step(self, e):
        now = time()
        dt = max(now - self.t_last, 1e-4)

        de = (e - self.e_last) / dt
        mag = abs(e)

        # Adaptive gains
        Kp = self.Kp_min + (self.Kp_max - self.Kp_min) * mag
        Kd = self.Kd_max - (self.Kd_max - self.Kd_min) * mag

        u = Kp * e + Kd * de
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
    ctrl   = AdaptivePDController(max_angle=MAX_STEER)

    try:
        while True:
            v = sensor.read()

            if interp.line_lost(v):
                px.set_dir_servo_angle(0)
                px.forward(PX_POWER // 2)
                ctrl.reset()
                sleep(0.05)
                continue

            err = interp.compute_error(v)
            steer = ctrl.step(err)

            px.set_dir_servo_angle(steer)
            px.forward(PX_POWER)

            print(
                f"adc={[round(x,1) for x in v]} | "
                f"err={err:+.3f} | "
                f"steer={steer:+.1f}"
            )

            sleep(0.01)

    except KeyboardInterrupt:
        print("\nStopping...")
        px.stop()
        sleep(0.1)
