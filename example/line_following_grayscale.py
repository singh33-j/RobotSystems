
from time import sleep, time
from picarx import Picarx

try:
    from robot_hat import ADC
except ImportError:
    from sim_robot_hat import ADC


REFERENCE = [1400, 1400, 1400]   # required by assignment
FILTER_ALPHA = 0.5              # sensor LPF

EDGE_MAG_THRESH   = 0.15        # minimum edge strength
EDGE_ASYM_THRESH  = 0.10        # must be directional

CONTROL_DT = 0.01



#Sensing
class LineSensor:
    def __init__(self, pins=['A0','A1','A2']):
        self.adc = [ADC(p) for p in pins]
        self.filt = [0.0, 0.0, 0.0]

    def read(self):
        raw = [a.read() for a in self.adc]
        for i in range(3):
            self.filt[i] = FILTER_ALPHA * raw[i] + (1 - FILTER_ALPHA) * self.filt[i]
        return self.filt.copy()

    def status(self, v):
        # binary comparator (used only for lost-line detection)
        return [0 if v[i] <= REFERENCE[i] else 1 for i in range(3)]


#Interpreter
class LineInterpreter:
    def __init__(self, polarity='dark'):
        self.polarity = polarity

    def compute_error(self, v):
        L, C, R = v

        #  Remove global brightness
        mu = (L + C + R) / 3.0
        Lr, Cr, Rr = L - mu, C - mu, R - mu

        # Adjacent differences = edges
        dLC = Cr - Lr
        dCR = Rr - Cr

        if self.polarity == 'light':
            dLC = -dLC
            dCR = -dCR

        # Normalize by local contrast
        spread = max(abs(Lr), abs(Cr), abs(Rr)) + 1e-6
        dLC /= spread
        dCR /= spread

        edge_mag  = max(abs(dLC), abs(dCR))
        edge_asym = abs(abs(dLC) - abs(dCR))

        # No reliable edge → go straight
        if edge_mag < EDGE_MAG_THRESH or edge_asym < EDGE_ASYM_THRESH:
            return 0.0

        # Choose dominant edge (soft, signed) 
        if abs(dLC) > abs(dCR):
            e = +dLC     # left edge → line on left → steer right
        else:
            e = -dCR     # right edge → line on right → steer left

        # Soft clamp (prevents ±1 snapping)
        e = max(-1.0, min(1.0, 0.7 * e))
        return e

    def line_lost(self, v):
        # If contrast collapses entirely
        mu = sum(v) / 3.0
        return max(abs(x - mu) for x in v) < 25


#PD Controller
class PDController:
    def __init__(self, Kp=16.0, Kd=2.0, max_angle=30.0):
        self.Kp = Kp
        self.Kd = Kd
        self.max = max_angle
        self.e_last = 0.0
        self.t_last = time()

    def step(self, e):
        t = time()
        dt = max(t - self.t_last, 1e-4)

        de = (e - self.e_last) / dt
        u  = self.Kp * e + self.Kd * de

        u = max(-self.max, min(self.max, u))

        self.e_last = e
        self.t_last = t
        return u

    def reset(self):
        self.e_last = 0.0
        self.t_last = time()


#Main
if __name__ == "__main__":

    px = Picarx()
    sensor = LineSensor()
    interp = LineInterpreter(polarity='dark')
    ctrl   = PDController()

    px_power = 10

    try:
        while True:
            v = sensor.read()
            s = sensor.status(v)

            # Line lost → straighten + slow forward
            if interp.line_lost(v):
                px.set_dir_servo_angle(0)
                px.forward(px_power // 2)
                ctrl.reset()
                sleep(0.05)
                continue

            err    = interp.compute_error(v)
            steer = ctrl.step(err)

            px.set_dir_servo_angle(steer)
            px.forward(px_power)

            print(
                f"adc={[round(x,1) for x in v]} | "
                f"err={err:+.3f} | "
                f"steer={steer:+.1f}"
            )

            sleep(CONTROL_DT)

    except KeyboardInterrupt:
        print("\nStopping...")
        px.stop()
        sleep(0.1)
