"""
Line Following program for Picar-X with PD Control (Robust, Sign-Corrected)

Section 3.1 — Sensing:
    - Direct ADC access (A0 = left, A1 = center, A2 = right)
    - Raw grayscale readings with low-pass filtering
    - Fixed reference values [1400, 1400, 1400]

Section 3.2 — Interpretation:
    - Explicit polarity parameter (dark vs light line)
    - Continuous contrast-based geometric error

Section 3.3 — Control:
    - PD steering controller
    - Error filtering before derivative
    - Output saturation for hardware safety
"""

from time import sleep, time
from picarx import Picarx

# ADC import (hardware or simulation)
try:
    from robot_hat import ADC
except ImportError:
    from sim_robot_hat import ADC


# ============================================================
# Section 3.1 — SENSING
# ============================================================
class LineSensor:
    """
    Reads grayscale sensors directly from ADC hardware.
    A0 = left, A1 = center, A2 = right
    """

    def __init__(self, grayscale_pins=['A0', 'A1', 'A2'],
                 reference_values=None, alpha=0.2):

        self.adc_channels = [ADC(pin) for pin in grayscale_pins]
        self.alpha = alpha
        self.filtered = [0.0, 0.0, 0.0]

        # SAME REFERENCE LOGIC AS ORIGINAL
        if reference_values is None:
            self.reference = [1400, 1400, 1400]
        else:
            self.reference = reference_values

    def read_grayscale_data(self):
        raw = [adc.read() for adc in self.adc_channels]

        # Low-pass filter ADC values
        for i in range(3):
            self.filtered[i] = (
                self.alpha * raw[i] +
                (1.0 - self.alpha) * self.filtered[i]
            )

        return self.filtered.copy()

    def read_line_status(self, gm_vals):
        """
        Binary comparator (used ONLY for line-lost detection)
        0 = line (dark)
        1 = background (light)
        """
        return [
            0 if gm_vals[i] <= self.reference[i] else 1
            for i in range(3)
        ]

    def read_all(self):
        gm = self.read_grayscale_data()
        return gm, self.read_line_status(gm)


# ============================================================
# Section 3.2 — INTERPRETATION
# ============================================================
class LineInterpreter:
    """
    Converts sensor readings into a geometric line-position error.
    """

    def __init__(self, polarity='dark'):
        self.sensor_positions = [-1.0, 0.0, 1.0]
        self.polarity = polarity

    def calculate_error(self, gm_vals, reference):
        """
        Continuous centroid-based error in [-1, 1].
        Negative = line left, Positive = line right
        """
        contrasts = []

        for i, val in enumerate(gm_vals):
            if self.polarity == 'dark':
                c = reference[i] - val
            else:
                c = val - reference[i]

            contrasts.append(max(c, 0.0))

        wsum = sum(contrasts)
        if wsum == 0:
            return 0.0

        return sum(
            self.sensor_positions[i] * contrasts[i]
            for i in range(3)
        ) / wsum

    def is_line_lost(self, line_status):
        return line_status == [1, 1, 1]


# ============================================================
# Section 3.3 — CONTROL
# ============================================================
class PDController:
    """
    Proportional-Derivative controller with filtered error.
    """

    def __init__(self, Kp=15.0, Kd=4.0, max_angle=30.0, beta=0.7):
        self.Kp = Kp
        self.Kd = Kd
        self.max_angle = max_angle

        self.beta = beta
        self.err_filt = 0.0

        self.last_error = 0.0
        self.last_time = time()

    def compute(self, error):
        now = time()
        dt = max(now - self.last_time, 1e-4)

        # Filter error before derivative
        self.err_filt = self.beta * self.err_filt + (1.0 - self.beta) * error

        P = self.Kp * self.err_filt
        D = self.Kd * (self.err_filt - self.last_error) / dt

        u = max(-self.max_angle, min(self.max_angle, P + D))

        self.last_error = self.err_filt
        self.last_time = now
        return u

    def reset(self):
        self.err_filt = 0.0
        self.last_error = 0.0
        self.last_time = time()


# ============================================================
# LINE LOST RECOVERY
# ============================================================
def handle_line_lost(px, controller, px_power):
    """
    Forward-biased search to prevent oscillation.
    """
    angle = controller.max_angle if controller.last_error > 0 else -controller.max_angle
    px.set_dir_servo_angle(angle)
    px.forward(px_power // 2)
    sleep(0.15)
    controller.reset()


# ============================================================
# MAIN PROGRAM
# ============================================================
if __name__ == "__main__":

    px = Picarx()

    sensor = LineSensor(
        grayscale_pins=['A0', 'A1', 'A2'],
        reference_values=[1400, 1400, 1400]
    )

    interpreter = LineInterpreter(polarity='dark')

    controller = PDController(
        Kp=15.0,
        Kd=4.0,
        max_angle=30.0
    )

    px_power = 10

    try:
        while True:
            gm_vals, line_status = sensor.read_all()

            if interpreter.is_line_lost(line_status):
                handle_line_lost(px, controller, px_power)
                continue

            error = interpreter.calculate_error(gm_vals, sensor.reference)
            steering_cmd = controller.compute(error)

            # CRITICAL FIX: steering sign
            px.set_dir_servo_angle(-steering_cmd)
            px.forward(px_power)

            print(
                f"err={error:+.3f} | "
                f"steer={-steering_cmd:+.1f}° | "
                f"status={line_status} | "
                f"adc={gm_vals}"
            )

            sleep(0.01)

    except KeyboardInterrupt:
        print("\nStopping...")

    finally:
        px.stop()
        sleep(0.1)
