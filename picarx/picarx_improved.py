import os
import sys
import time
import logging
import atexit
import math

from logdecorator import log_on_start, log_on_end, log_on_error

# Add in check if we have access to pi or are in sim mode
try:
    from robot_hat import Pin, ADC, PWM, Servo, fileDB
    from robot_hat import Grayscale_Module, Ultrasonic, utils
    on_the_robot = True
except ImportError:
    on_the_robot = False
    sys.path.append(
        os.path.abspath(
            os.path.join(os.path.dirname(__file__), "..")
        )
    )
    from sim_robot_hat import Pin, ADC, PWM, Servo, fileDB
    from sim_robot_hat import Grayscale_Module, Ultrasonic, utils

# Initialize logging
logging_format = "%(asctime)s: %(message)s"
logging.basicConfig(
    format=logging_format,
    level=logging.INFO,
    datefmt="%H:%M:%S"
)
logging.getLogger().setLevel(logging.DEBUG)


def constrain(x, min_val, max_val):
    """Constrain value to a range."""
    return max(min_val, min(max_val, x))


class Picarx(object):
    CONFIG = '/opt/picar-x/picar-x.conf'

    DEFAULT_LINE_REF = [1000, 1000, 1000]
    DEFAULT_CLIFF_REF = [500, 500, 500]

    DIR_MIN = -30
    DIR_MAX = 30
    CAM_PAN_MIN = -90
    CAM_PAN_MAX = 90
    CAM_TILT_MIN = -35
    CAM_TILT_MAX = 65

    PERIOD = 4095
    PRESCALER = 10
    TIMEOUT = 0.02

    def __init__(
        self,
        servo_pins=['P0', 'P1', 'P2'],
        motor_pins=['D4', 'D5', 'P13', 'P12'],
        grayscale_pins=['A0', 'A1', 'A2'],
        ultrasonic_pins=['D2', 'D3'],
        config=CONFIG,
    ):
        utils.reset_mcu()
        time.sleep(0.2)

        if on_the_robot:
            self.config_flie = fileDB(config, 777, os.getlogin())
        else:
            self.config_flie = fileDB(config, 777, None)

        # Servos
        self.cam_pan = Servo(servo_pins[0])
        self.cam_tilt = Servo(servo_pins[1])
        self.dir_servo_pin = Servo(servo_pins[2])

        self.dir_cali_val = float(self.config_flie.get("picarx_dir_servo", 0))
        self.cam_pan_cali_val = float(self.config_flie.get("picarx_cam_pan_servo", 0))
        self.cam_tilt_cali_val = float(self.config_flie.get("picarx_cam_tilt_servo", 0))

        self.dir_servo_pin.angle(self.dir_cali_val)
        self.cam_pan.angle(self.cam_pan_cali_val)
        self.cam_tilt.angle(self.cam_tilt_cali_val)

        # Motors
        self.left_rear_dir_pin = Pin(motor_pins[0])
        self.right_rear_dir_pin = Pin(motor_pins[1])
        self.left_rear_pwm_pin = PWM(motor_pins[2])
        self.right_rear_pwm_pin = PWM(motor_pins[3])

        self.motor_direction_pins = [
            self.left_rear_dir_pin,
            self.right_rear_dir_pin
        ]
        self.motor_speed_pins = [
            self.left_rear_pwm_pin,
            self.right_rear_pwm_pin
        ]

        self.cali_dir_value = self.config_flie.get(
            "picarx_dir_motor",
            "[1, 1]"
        )
        self.cali_dir_value = [
            int(i.strip())
            for i in self.cali_dir_value.strip("[]").split(",")
        ]

        self.cali_speed_value = [0, 0]
        self.dir_current_angle = 0

        for pin in self.motor_speed_pins:
            pin.period(self.PERIOD)
            pin.prescaler(self.PRESCALER)

        # Grayscale
        adc0, adc1, adc2 = [ADC(pin) for pin in grayscale_pins]
        self.grayscale = Grayscale_Module(adc0, adc1, adc2)

        self.line_reference = [
            float(i) for i in
            self.config_flie.get(
                "line_reference",
                str(self.DEFAULT_LINE_REF)
            ).strip("[]").split(",")
        ]

        self.cliff_reference = [
            float(i) for i in
            self.config_flie.get(
                "cliff_reference",
                str(self.DEFAULT_CLIFF_REF)
            ).strip("[]").split(",")
        ]

        self.grayscale.reference(self.line_reference)

        # Ultrasonic
        trig, echo = ultrasonic_pins
        self.ultrasonic = Ultrasonic(
            Pin(trig),
            Pin(echo, mode=Pin.IN, pull=Pin.PULL_DOWN)
        )

        atexit.register(self.stop)

    def set_motor_speed(self, motor, speed):
        speed = constrain(speed, -100, 100)
        motor -= 1

        direction = (
            self.cali_dir_value[motor]
            if speed >= 0 else
            -self.cali_dir_value[motor]
        )

        speed = abs(speed)
        speed = constrain(speed, 0, 100)
        speed -= self.cali_speed_value[motor]

        if direction < 0:
            self.motor_direction_pins[motor].high()
        else:
            self.motor_direction_pins[motor].low()

        self.motor_speed_pins[motor].pulse_width_percent(speed)

    def set_power(self, speed):
        self.set_motor_speed(1, speed)
        self.set_motor_speed(2, speed)

    def ackerman_scaling(self, steering_angle_deg):
        steering_angle_deg = constrain(
            steering_angle_deg,
            self.DIR_MIN,
            self.DIR_MAX
        )
        return math.cos(math.radians(abs(steering_angle_deg)))

    def forward(self, speed):
        if self.dir_current_angle == 0:
            self.set_motor_speed(1, speed)
            self.set_motor_speed(2, -speed)
            return

        scale = self.ackerman_scaling(self.dir_current_angle)
        if self.dir_current_angle > 0:
            self.set_motor_speed(1, speed * scale)
            self.set_motor_speed(2, -speed)
        else:
            self.set_motor_speed(1, speed)
            self.set_motor_speed(2, -speed * scale)

    def backward(self, speed):
        self.forward(-speed)

    def stop(self):
        for _ in range(2):
            self.motor_speed_pins[0].pulse_width_percent(0)
            self.motor_speed_pins[1].pulse_width_percent(0)
            time.sleep(0.002)

    def close(self):
        self.stop()
        self.ultrasonic.close()


if __name__ == "__main__":
    px = Picarx()

    command_map = {
        "f": px.forward_backward,
        "t": px.three_point_turn,
        "p": px.parallel_park,
    }

    print("\nPiCar-X Control")
    print("----------------")
    print("f : Forward / Backward")
    print("t : Three-point turn")
    print("p : Parallel park")
    print("q : Quit\n")

    try:
        while True:
            cmd = input("Enter command (f/t/p/q): ").strip().lower()

            if cmd == "q":
                break
            elif cmd in command_map:
                command_map[cmd]()
            else:
                print("Invalid command.")

    finally:
        px.stop()
        px.close()
