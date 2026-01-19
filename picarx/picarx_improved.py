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

#Initialize logging
logging_format = "%(asctime)s: %(message)s"
logging.basicConfig(format=logging_format, level=logging.INFO,
datefmt="%H:%M:%S")
logging.getLogger().setLevel(logging.DEBUG)


@log_on_start(logging.DEBUG, "Constrain function starting")
@log_on_error(logging.DEBUG, "Error encountered in constrain fxn")
@log_on_end(logging.DEBUG, "Constrain fxn ended succesfully: {result!r}")

#Ensure output is clamped btwn min and max value
def constrain(x, min_val, max_val):
    '''
    Constrains value to be within a range.
    '''
    return max(min_val, min(max_val, x))

#Create/initialize Picar class
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

    # servo_pins: camera_pan_servo, camera_tilt_servo, direction_servo
    # motor_pins: left_swicth, right_swicth, left_pwm, right_pwm
    # grayscale_pins: 3 adc channels
    # ultrasonic_pins: trig, echo2
    # config: path of config file
    def __init__(self, 
                servo_pins:list=['P0', 'P1', 'P2'], 
                motor_pins:list=['D4', 'D5', 'P13', 'P12'],
                grayscale_pins:list=['A0', 'A1', 'A2'],
                ultrasonic_pins:list=['D2','D3'],
                config:str=CONFIG,
                ):

        # reset robot_hat
        utils.reset_mcu()
        time.sleep(0.2)

        # --------- config_flie ---------
        # Add in if statement for on robot vs off robot
        if on_the_robot:
            self.config_flie = fileDB(config, 777, os.getlogin())
        else:
            self.config_flie = fileDB(config, 777, None)


        # --------- servos init ---------
        self.cam_pan = Servo(servo_pins[0])
        self.cam_tilt = Servo(servo_pins[1])   
        self.dir_servo_pin = Servo(servo_pins[2])
        # get calibration values
        self.dir_cali_val = float(self.config_flie.get("picarx_dir_servo", default_value=0))
        self.cam_pan_cali_val = float(self.config_flie.get("picarx_cam_pan_servo", default_value=0))
        self.cam_tilt_cali_val = float(self.config_flie.get("picarx_cam_tilt_servo", default_value=0))
        # set servos to init angle
        self.dir_servo_pin.angle(self.dir_cali_val)
        self.cam_pan.angle(self.cam_pan_cali_val)
        self.cam_tilt.angle(self.cam_tilt_cali_val)

        # --------- motors init ---------
        self.left_rear_dir_pin = Pin(motor_pins[0])
        self.right_rear_dir_pin = Pin(motor_pins[1])
        self.left_rear_pwm_pin = PWM(motor_pins[2])
        self.right_rear_pwm_pin = PWM(motor_pins[3])
        self.motor_direction_pins = [self.left_rear_dir_pin, self.right_rear_dir_pin]
        self.motor_speed_pins = [self.left_rear_pwm_pin, self.right_rear_pwm_pin]
        # get calibration values
        self.cali_dir_value = self.config_flie.get("picarx_dir_motor", default_value="[1, 1]")
        self.cali_dir_value = [int(i.strip()) for i in self.cali_dir_value.strip().strip("[]").split(",")]
        self.cali_speed_value = [0, 0]
        self.dir_current_angle = 0
        # init pwm
        for pin in self.motor_speed_pins:
            pin.period(self.PERIOD)
            pin.prescaler(self.PRESCALER)

        # --------- grayscale module init ---------
        adc0, adc1, adc2 = [ADC(pin) for pin in grayscale_pins]
        self.grayscale = Grayscale_Module(adc0, adc1, adc2, reference=None)
        # get reference
        self.line_reference = self.config_flie.get("line_reference", default_value=str(self.DEFAULT_LINE_REF))
        self.line_reference = [float(i) for i in self.line_reference.strip().strip('[]').split(',')]
        self.cliff_reference = self.config_flie.get("cliff_reference", default_value=str(self.DEFAULT_CLIFF_REF))
        self.cliff_reference = [float(i) for i in self.cliff_reference.strip().strip('[]').split(',')]
        # transfer reference
        self.grayscale.reference(self.line_reference)

        # --------- ultrasonic init ---------
        trig, echo= ultrasonic_pins
        self.ultrasonic = Ultrasonic(Pin(trig), Pin(echo, mode=Pin.IN, pull=Pin.PULL_DOWN))

        #Stop motors using atextit 
        atexit.register(self.stop)

    
    @log_on_start(logging.DEBUG, "Set_motor_speed fxn starting")
    @log_on_error(logging.DEBUG, "Error encountered in set_motor_speed function  ")
    @log_on_end(logging.DEBUG, "Set_motor_speed fxn ended succesfully: {result!r}")

    def set_motor_speed(self, motor, speed):
        ''' set motor speed
        
        param motor: motor index, 1 means left motor, 2 means right motor
        type motor: int
        param speed: speed
        type speed: int      
        '''
        speed = constrain(speed, -100, 100)
        #motor 0 - left motor, motor 1 - right motor
        motor -= 1
        motor_name = "LEFT" if motor == 0 else "RIGHT"
        if speed >= 0:
            direction = 1 * self.cali_dir_value[motor]
        elif speed < 0:
            direction = -1 * self.cali_dir_value[motor]
        speed = abs(speed)
        if speed != 0:
            # speed = int(speed /2 ) + 50
            #Get rid of speed scaling and constrain 
            speed = abs(speed)
            speed = constrain(speed,0,100)
            print("Speed value:", speed)
        speed = speed - self.cali_speed_value[motor]

        logging.debug(
        f"Motor {motor+1} ({motor_name}) | "
        f"PWM={speed} | "
        f"Direction={'BACKWARD' if direction < 0 else 'FORWARD'} | "
        f"DIR_PIN={'HIGH' if direction < 0 else 'LOW'}")

        if direction < 0:
            self.motor_direction_pins[motor].high()
            self.motor_speed_pins[motor].pulse_width_percent(speed)
        else:
            self.motor_direction_pins[motor].low()
            self.motor_speed_pins[motor].pulse_width_percent(speed)

    @log_on_start(logging.DEBUG, "Motor speed calibration fxn starting")
    @log_on_error(logging.DEBUG, "Error encountered in motor speed calibration fxn")
    @log_on_end(logging.DEBUG, "Motor speed calibration fxn ended succesfully: {result!r}")

    def motor_speed_calibration(self, value):
        self.cali_speed_value = value
        if value < 0:
            self.cali_speed_value[0] = 0
            self.cali_speed_value[1] = abs(self.cali_speed_value)
        else:
            self.cali_speed_value[0] = abs(self.cali_speed_value)
            self.cali_speed_value[1] = 0
    
    @log_on_start(logging.DEBUG, "Motor direction calibration fxn starting")
    @log_on_error(logging.DEBUG, "Error encountered in motor direction calibration fxn")
    @log_on_end(logging.DEBUG, "Motor direction calibration fxn ended succesfully: {result!r}")
    
    def motor_direction_calibrate(self, motor, value):
        ''' set motor direction calibration value
        
        param motor: motor index, 1 means left motor, 2 means right motor
        type motor: int
        param value: speed
        type value: int
        '''      
        motor -= 1
        if value == 1:
            self.cali_dir_value[motor] = 1
        elif value == -1:
            self.cali_dir_value[motor] = -1
        self.config_flie.set("picarx_dir_motor", self.cali_dir_value)
        
    @log_on_start(logging.DEBUG, "Servo dir calibration fxn starting")
    @log_on_error(logging.DEBUG, "Error encountered in servo dir calibration fxn")
    @log_on_end(logging.DEBUG, "Servo dir calibration fxn ended succesfully: {result!r}")
    
    def dir_servo_calibrate(self, value):
        self.dir_cali_val = value
        self.config_flie.set("picarx_dir_servo", "%s"%value)
        self.dir_servo_pin.angle(value)

    @log_on_start(logging.DEBUG, "Set servo angle fxn starting")
    @log_on_error(logging.DEBUG, "Error encountered in servo d fxn")
    @log_on_end(logging.DEBUG, "Servo dir calibration fxn ended succesfully: {result!r}")

    def set_dir_servo_angle(self, value):
        self.dir_current_angle = constrain(value, self.DIR_MIN, self.DIR_MAX)
        angle_value  = self.dir_current_angle + self.dir_cali_val
        self.dir_servo_pin.angle(angle_value)
        logging.debug(f"STEERING | requested={value} | applied={self.dir_current_angle}")


    @log_on_start(logging.DEBUG, "Set Pan servo calibration")
    @log_on_error(logging.DEBUG, "Error encountered in pan servo calibration")
    @log_on_end(logging.DEBUG, "Pan servo calibration fxn ended succesfully: {result!r}")
    
    def cam_pan_servo_calibrate(self, value):
        self.cam_pan_cali_val = value
        self.config_flie.set("picarx_cam_pan_servo", "%s"%value)
        self.cam_pan.angle(value)

    @log_on_start(logging.DEBUG, "Set tilt servo calibration ")
    @log_on_error(logging.DEBUG, "Error encountered in tilt servo calibration")
    @log_on_end(logging.DEBUG, "Tilt servo calibration fxn ended succesfully: {result!r}")
    
    def cam_tilt_servo_calibrate(self, value):
        self.cam_tilt_cali_val = value
        self.config_flie.set("picarx_cam_tilt_servo", "%s"%value)
        self.cam_tilt.angle(value)

    @log_on_start(logging.DEBUG, "Set Pan servo angle")
    @log_on_error(logging.DEBUG, "Error encountered in pan servo angle")
    @log_on_end(logging.DEBUG, "Pan servo angle fxn ended succesfully: {result!r}")

    def set_cam_pan_angle(self, value):
        value = constrain(value, self.CAM_PAN_MIN, self.CAM_PAN_MAX)
        self.cam_pan.angle(-1*(value + -1*self.cam_pan_cali_val))

    @log_on_start(logging.DEBUG, "Set cam tilt servo angle")
    @log_on_error(logging.DEBUG, "Error encountered in cam tilt servo angle")
    @log_on_end(logging.DEBUG, "Cam tilt servo angle fxn ended succesfully: {result!r}")

    def set_cam_tilt_angle(self,value):
        value = constrain(value, self.CAM_TILT_MIN, self.CAM_TILT_MAX)
        self.cam_tilt.angle(-1*(value + -1*self.cam_tilt_cali_val))

    @log_on_start(logging.DEBUG, "Set motor power/speed")
    @log_on_error(logging.DEBUG, "Error encountered in set motor power/speed")
    @log_on_end(logging.DEBUG, "Set motor power/speed ended succesfully: {result!r}")

    def set_power(self, speed):
        self.set_motor_speed(1, speed)
        self.set_motor_speed(2, speed)
        logging.debug(f"SET_POWER | speed={speed} (both motors)")


    @log_on_start(logging.DEBUG, "Forward-backward sequence started")
    @log_on_error(logging.DEBUG, "Error in forward_backward sequence")
    @log_on_end(logging.DEBUG, "Forward-backward sequence completed")

    def forward_backward(self, speed=40, duration=1.0, cycles=2):
        """
        Move straight forward and backward for a fixed duration.
        """
        self.set_dir_servo_angle(0)

        for i in range(cycles):
            logging.debug(f"CYCLE {i+1}/{cycles} | FORWARD")
            self.forward(speed)
            time.sleep(duration)
            self.stop()
            time.sleep(0.5)

            logging.debug(f"CYCLE {i+1}/{cycles} | BACKWARD")
            self.backward(speed)
            time.sleep(duration)
            self.stop()
            time.sleep(0.5)

    @log_on_start(logging.DEBUG, "3-point turn started")
    @log_on_error(logging.DEBUG, "Error during 3-point turn")
    @log_on_end(logging.DEBUG, "3-point turn completed")

    def three_point_turn(self, speed=35, turn_time=3.5, settle_time=0.5):
        """
        Perform a 3-point (K) turn.
        """
        self.set_dir_servo_angle(0)
        logging.debug("3PT | Step 1: Forward with left steering")
        self.set_dir_servo_angle(30)
        self.forward(speed)
        time.sleep(turn_time)
        self.stop()
        time.sleep(settle_time)

        logging.debug("3PT | Step 2: Backward with right steering")
        self.set_dir_servo_angle(-30)
        self.backward(speed)
        time.sleep(turn_time)
        self.stop()
        time.sleep(settle_time)

        logging.debug("3PT | Step 3: Straighten out")
        self.set_dir_servo_angle(5)
        self.forward(speed)
        time.sleep(turn_time * 0.75)
        self.stop()
        
        logging.debug("4PT | Step 4: Go Straight")
        self.set_dir_servo_angle(0)
        self.forward(speed)
        time.sleep(turn_time * 0.75)
        self.stop()

    @log_on_start(logging.DEBUG, "Parallel parking started")
    @log_on_error(logging.DEBUG, "Error during parallel parking")
    @log_on_end(logging.DEBUG, "Parallel parking completed")

    def parallel_park(
        self,
        speed=35,
        forward_time=2.0,
        reverse_time=2.0,
        settle_time=0.5
    ):
        """
        Perform a simple parallel parking maneuver.
        """

        logging.debug("PARALLEL PARK | Step 1: Pull forward")
        self.set_dir_servo_angle(0)
        self.forward(speed)
        time.sleep(forward_time)
        self.stop()
        time.sleep(settle_time)

        logging.debug("PARALLEL PARK | Step 2: Reverse right")
        self.set_dir_servo_angle(-20)
        self.backward(speed)
        time.sleep(reverse_time)
        self.stop()
        time.sleep(settle_time)

        logging.debug("PARALLEL PARK | Step 3: Reverse left")
        self.set_dir_servo_angle(20)
        self.backward(speed)
        time.sleep(reverse_time)
        self.stop()
        time.sleep(settle_time)

        logging.debug("PARALLEL PARK | Step 4: Final adjust")
        self.set_dir_servo_angle(0)
        self.backward(speed * 0.5)
        time.sleep(0.5)
        self.stop()

                
        
    

    #Write function to use cos scaling instead of linear
    def ackerman_scaling(self,steering_angle_deg):
        steering_angle_deg = constrain(steering_angle_deg, self.DIR_MIN, self.DIR_MAX)
        steering_angle_rad = math.radians(abs(steering_angle_deg))
        ackerman_scale = math.cos(steering_angle_rad)

        return ackerman_scale

    def backward(self, speed):
        logging.debug(f"BACKWARD | speed={speed} | steering={self.dir_current_angle} deg")
        current_angle = self.dir_current_angle
        if current_angle != 0:
            abs_current_angle = abs(current_angle)
            if abs_current_angle > self.DIR_MAX:
                abs_current_angle = self.DIR_MAX
            #power_scale = (100 - abs_current_angle) / 100.0 
            power_scale = self.ackerman_scaling(abs_current_angle)
            logging.debug(f"FORWARD TURN | scale={power_scale:.3f} | "f"{'LEFT' if current_angle > 0 else 'RIGHT'} wheel slowed")
            if (current_angle / abs_current_angle) > 0:
                self.set_motor_speed(1, -1*speed)
                self.set_motor_speed(2, speed * power_scale)
            else:
                self.set_motor_speed(1, -1*speed * power_scale)
                self.set_motor_speed(2, speed )
        else:
            self.set_motor_speed(1, -1*speed)
            self.set_motor_speed(2, speed)  

    @log_on_start(logging.DEBUG, "Forward fxn started")
    @log_on_error(logging.DEBUG, "Error encountered in forward fxn")
    @log_on_end(logging.DEBUG, "Forward fxn ended succesfully: {result!r}")

    def forward(self, speed):
        logging.debug(f"FORWARD | speed={speed} | steering={self.dir_current_angle} deg")
        current_angle = self.dir_current_angle
        if current_angle != 0:
            abs_current_angle = abs(current_angle)
            if abs_current_angle > self.DIR_MAX:
                abs_current_angle = self.DIR_MAX
            #power_scale = (100 - abs_current_angle) / 100.0
            power_scale = self.ackerman_scaling(abs_current_angle)
            logging.debug(f"FORWARD TURN | scale={power_scale:.3f} | "f"{'LEFT' if current_angle > 0 else 'RIGHT'} wheel slowed")
            if (current_angle / abs_current_angle) > 0:
                self.set_motor_speed(1, 1*speed * power_scale)
                self.set_motor_speed(2, -speed) 
            else:
                self.set_motor_speed(1, speed)
                self.set_motor_speed(2, -1*speed * power_scale)
        else:
            self.set_motor_speed(1, speed)
            self.set_motor_speed(2, -1*speed)    
              

    @log_on_start(logging.DEBUG, "Stopping fxn started")
    @log_on_error(logging.DEBUG, "Error encountered in stopping fxn")
    @log_on_end(logging.DEBUG, "Stopping fxn ended succesfully: {result!r}")

    def stop(self):
        '''
        Execute twice to make sure it stops
        '''
        logging.debug("STOP | All motors set to PWM=0")
        for _ in range(2):
            self.motor_speed_pins[0].pulse_width_percent(0)
            self.motor_speed_pins[1].pulse_width_percent(0)
            time.sleep(0.002)

    
    @log_on_start(logging.DEBUG, "Getting distance")
    @log_on_error(logging.DEBUG, "Error encountered in getting distance")
    @log_on_end(logging.DEBUG, "Getting distance worked: {result!r}")

    def get_distance(self):
        return self.ultrasonic.read()

    @log_on_start(logging.DEBUG, "Setting grayscale reference")
    @log_on_error(logging.DEBUG, "Error encountered in setting grayscale reference")
    @log_on_end(logging.DEBUG, "Setting grayscale reference worked: {result!r}")

    def set_grayscale_reference(self, value):
        if isinstance(value, list) and len(value) == 3:
            self.line_reference = value
            self.grayscale.reference(self.line_reference)
            self.config_flie.set("line_reference", self.line_reference)
        else:
            raise ValueError("grayscale reference must be a 1*3 list")

    @log_on_start(logging.DEBUG, "Getting grayscale data")
    @log_on_error(logging.DEBUG, "Error encountered in getting grayscale data")
    @log_on_end(logging.DEBUG, "Getting grayscale data worked: {result!r}")

    def get_grayscale_data(self):
        return list.copy(self.grayscale.read())

    @log_on_start(logging.DEBUG, "Getting line status")
    @log_on_error(logging.DEBUG, "Error encountered in getting line status")
    @log_on_end(logging.DEBUG, "Getting line status worked: {result!r}")

    def get_line_status(self,gm_val_list):
        return self.grayscale.read_status(gm_val_list)

    @log_on_start(logging.DEBUG, "Setting line referemce")
    @log_on_error(logging.DEBUG, "Error encountered in setting line reference")
    @log_on_end(logging.DEBUG, "Setting line reference worked: {result!r}")

    def set_line_reference(self, value):
        self.set_grayscale_reference(value)

    
    @log_on_start(logging.DEBUG, "Getting cliff status")
    @log_on_error(logging.DEBUG, "Error encountered in getting cliff status")
    @log_on_end(logging.DEBUG, "Getting cliff status worked: {result!r}")

    def get_cliff_status(self,gm_val_list):
        for i in range(0,3):
            if gm_val_list[i]<=self.cliff_reference[i]:
                return True
        return False

    @log_on_start(logging.DEBUG, "Setting cliff reference")
    @log_on_error(logging.DEBUG, "Error encountered in setting cliff reference")
    @log_on_end(logging.DEBUG, "Setting cliff reference worked: {result!r}")

    def set_cliff_reference(self, value):
        if isinstance(value, list) and len(value) == 3:
            self.cliff_reference = value
            self.config_flie.set("cliff_reference", self.cliff_reference)
        else:
            raise ValueError("grayscale reference must be a 1*3 list")

    
    @log_on_start(logging.DEBUG, "Resetting started")
    @log_on_error(logging.DEBUG, "Error encountered in resetting")
    @log_on_end(logging.DEBUG, "Resetting worked: {result!r}")

    def reset(self):
        self.stop()
        self.set_dir_servo_angle(0)
        self.set_cam_tilt_angle(0)
        self.set_cam_pan_angle(0)

    @log_on_start(logging.DEBUG, "Closing")
    @log_on_error(logging.DEBUG, "Error encountered in closing")
    @log_on_end(logging.DEBUG, "Closing worked: {result!r}")

    def close(self):
        self.reset()
        self.ultrasonic.close()

    
if __name__ == "__main__":
    px = Picarx()

    px.three_point_turn(
        speed=35,
        turn_time=3.5
    )

    time.sleep(1)
    px.stop()



