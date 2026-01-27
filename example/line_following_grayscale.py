"""
Line Following program for Picar-X with PD Control
Fully rewritten to explicitly satisfy Section 3 requirements.

Section 3.1 — Sensing:
    - Direct ADC access (A0 = left, A1 = center, A2 = right)
    - Raw grayscale readings + software thresholding

Section 3.2 — Interpretation:
    - Explicit polarity parameter (dark vs light line)
    - Explicit sensitivity parameter (contrast aggressiveness)
    - Converts sensor readings → geometric line error

Section 3.3 — Control:
    - PD steering controller
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

    def __init__(self, grayscale_pins=['A0', 'A1', 'A2'], reference_values=None):
        self.adc_channels = [ADC(pin) for pin in grayscale_pins]

        if reference_values is None:
            self.reference = [1400, 1400, 1400]
        else:
            self.reference = reference_values

    def read_grayscale_data(self):
        """
        Returns raw ADC readings:
        [left, center, right]
        """
        return [adc.read() for adc in self.adc_channels]

    def read_line_status(self, gm_vals=None):
        """
        Software comparator:
        0 = line (dark)
        1 = background (light)
        """
        if gm_vals is None:
            gm_vals = self.read_grayscale_data()

        return [
            0 if gm_vals[i] <= self.reference[i] else 1
            for i in range(3)
        ]

    def read_all(self):
        gm_vals = self.read_grayscale_data()
        return gm_vals, self.read_line_status(gm_vals)


# ============================================================
# Section 3.2 — INTERPRETATION
# ============================================================
class LineInterpreter:
    """
    Converts sensor readings into a geometric line-position error.
    """

    def __init__(self, sensitivity=1.0, polarity='dark'):
        """
        Args:
            sensitivity (float > 0):
                Controls how strongly contrast is weighted.
                Higher = more aggressive interpretation.

            polarity ('dark' or 'light'):
                'dark'  -> line darker than floor
                'light' -> line lighter than floor
        """
        self.sensor_positions = [-1.0, 0.0, 1.0]
        self.sensitivity = max(sensitivity, 1e-3)
        self.polarity = polarity

    def calculate_error(self, gm_vals, line_status):
        """
        Returns a signed error in [-1, 1].
        Negative = line left, Positive = line right
        """

        # Discrete robust cases
        if line_status == [1, 0, 1]:
            return 0.0
        if line_status == [0, 1, 1]:
            return -0.8
        if line_status == [0, 0, 1]:
            return -0.5
        if line_status == [1, 0, 0]:
            return 0.5
        if line_status == [1, 1, 0]:
            return 0.8

        # Fallback to continuous weighted interpretation
        return self._weighted_error(gm_vals)

    def _weighted_error(self, gm_vals):
        total = sum(gm_vals)
        if total == 0:
            return 0.0

        weights = []
        for val in gm_vals:
            if self.polarity == 'dark':
                contrast = (total - val) / total
            else:
                contrast = val / total

            weights.append(contrast ** self.sensitivity)

        wsum = sum(weights)
        if wsum == 0:
            return 0.0

        return sum(
            self.sensor_positions[i] * weights[i]
            for i in range(3)
        ) / wsum

    def is_line_lost(self, line_status):
        """
        Line lost if all sensors see background.
        """
        return line_status == [1, 1, 1]


# ============================================================
# Section 3.3 — CONTROL
# ============================================================
class PDController:
    """
    Proportional-Derivative controller for steering.
    """

    def __init__(self, Kp=15.0, Kd=5.0, max_angle=30.0):
        self.Kp = Kp
        self.Kd = Kd
        self.max_angle = max_angle
        self.last_error = 0.0
        self.last_time = time()

    def compute(self, error):
        now = time()
        dt = max(now - self.last_time, 1e-4)

        P = self.Kp * error
        D = self.Kd * (error - self.last_error) / dt

        u = P + D
        u = max(-self.max_angle, min(self.max_angle, u))

        self.last_error = error
        self.last_time = now
        return u

    def reset(self):
        self.last_error = 0.0
        self.last_time = time()


# ============================================================
# LINE LOST RECOVERY
# ============================================================
def handle_line_lost(px, controller, px_power):
    if controller.last_error < 0:
        px.set_dir_servo_angle(-controller.max_angle)
    else:
        px.set_dir_servo_angle(controller.max_angle)

    px.backward(px_power)
    sleep(0.1)
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

    interpreter = LineInterpreter(
        sensitivity=1.2,
        polarity='dark'
    )

    controller = PDController(
        Kp=15.0,
        Kd=5.0,
        max_angle=30.0
    )

    px_power = 10

    try:
        while True:
            gm_vals, line_status = sensor.read_all()

            if interpreter.is_line_lost(line_status):
                handle_line_lost(px, controller, px_power)
                continue

            error = interpreter.calculate_error(gm_vals, line_status)
            steering_angle = controller.compute(error)

            px.set_dir_servo_angle(steering_angle)
            px.forward(px_power)

            print(
                f"Error: {error:.3f} | "
                f"Steer: {steering_angle:.1f}° | "
                f"Status: {line_status} | "
                f"ADC: {gm_vals}"
            )

            sleep(0.01)

    except KeyboardInterrupt:
        print("\nStopping...")

    finally:
        px.stop()
        sleep(0.1)
