'''
    Line Following program for Picar-X with PD Control:

    Pay attention to modify the reference value of the grayscale module 
    according to the practical usage scenarios.
    Auto calibrate grayscale values:
        Please run ./calibration/grayscale_calibration.py
    Manual modification:
        Use the following: 
            px.set_line_reference([1400, 1400, 1400])
        The reference value be close to the middle of the line gray value
        and the background gray value.

    PD Control Parameters:
        Kp: Proportional gain (adjusts response to current error)
        Kd: Derivative gain (adjusts response to rate of error change)
        max_angle: Maximum steering angle in degrees
'''
from picarx import Picarx
from time import sleep, time

px = Picarx()
# px = Picarx(grayscale_pins=['A0', 'A1', 'A2'])

# Please run ./calibration/grayscale_calibration.py to Auto calibrate grayscale values
# or manual modify reference value by follow code
# px.set_line_reference([1400, 1400, 1400])

# PD Control Parameters
px_power = 10
Kp = 15.0  # Proportional gain - adjust this to change responsiveness
Kd = 5.0   # Derivative gain - adjust this to reduce oscillations
max_angle = 30  # Maximum steering angle in degrees

# PD Control state variables
last_error = 0.0
last_time = time()

def calculate_error(gm_val_list, line_status):
    """
    Calculate error from grayscale sensor readings.
    Returns error in range [-1, 1] where:
        -1 = line is far left
        0 = line is centered
        1 = line is far right
    """
    # Sensor positions: left=-1, center=0, right=1
    sensor_positions = [-1, 0, 1]
    
    # If all sensors see line (all 0), return 0 (centered)
    if line_status == [0, 0, 0]:
        return 0.0
    
    # If only center sensor sees line, we're centered
    if line_status == [1, 0, 1]:
        return 0.0
    
    # Calculate weighted position based on which sensors see the line
    # Use normalized grayscale values to get more precise positioning
    total = sum(gm_val_list)
    if total == 0:
        return 0.0
    
    # Normalize values (invert so lower values = more line detected)
    # Line is detected when value < reference (status = 0)
    # So we want to weight sensors that see the line more heavily
    weights = []
    for i, status in enumerate(line_status):
        if status == 0:  # Line detected
            # Use inverse of normalized value (lower value = more line)
            weight = (1.0 - gm_val_list[i] / total) if total > 0 else 1.0
        else:  # Background
            weight = 0.0
        weights.append(weight)
    
    # Normalize weights
    weight_sum = sum(weights)
    if weight_sum == 0:
        # Fallback: use simple position based on which sensor sees line
        if line_status[0] == 0:  # Left sensor sees line
            return -0.5
        elif line_status[2] == 0:  # Right sensor sees line
            return 0.5
        else:
            return 0.0
    
    # Calculate weighted average position
    error = sum(sensor_positions[i] * weights[i] for i in range(3)) / weight_sum
    
    # Alternative simpler method: use line status directly
    # This gives discrete positions but is more reliable
    if line_status[0] == 0 and line_status[1] == 1 and line_status[2] == 1:
        error = -0.8  # Line on left
    elif line_status[0] == 1 and line_status[1] == 0 and line_status[2] == 1:
        error = 0.0   # Line centered
    elif line_status[0] == 1 and line_status[1] == 1 and line_status[2] == 0:
        error = 0.8   # Line on right
    elif line_status[0] == 0 and line_status[1] == 0 and line_status[2] == 1:
        error = -0.5  # Line on left-center
    elif line_status[0] == 1 and line_status[1] == 0 and line_status[2] == 0:
        error = 0.5   # Line on right-center
    
    return error

def pd_control(error, last_error, dt):
    """
    PD Controller: calculates steering angle based on error and its derivative.
    
    Args:
        error: Current error value
        last_error: Previous error value
        dt: Time step since last calculation
    
    Returns:
        Steering angle in degrees
    """
    # Proportional term
    P = Kp * error
    
    # Derivative term (rate of change of error)
    if dt > 0:
        D = Kd * (error - last_error) / dt
    else:
        D = 0.0
    
    # Total control output
    control_output = P + D
    
    # Limit to maximum angle
    angle = max(-max_angle, min(max_angle, control_output))
    
    return angle

def outHandle():
    """Handle case when line is lost - use last known direction"""
    global last_error
    # Back up slightly and turn in last known direction
    if last_error < 0:
        px.set_dir_servo_angle(-max_angle)
        px.backward(px_power)
    else:
        px.set_dir_servo_angle(max_angle)
        px.backward(px_power)
    
    # Try to find line again
    for _ in range(10):
        gm_val_list = px.get_grayscale_data()
        line_status = px.get_line_status(gm_val_list)
        if line_status != [0, 0, 0]:  # Found line again
            break
        sleep(0.01)
    sleep(0.001)

if __name__=='__main__':
    try:
        while True:
            current_time = time()
            dt = current_time - last_time
            last_time = current_time
            
            # Read sensor data
            gm_val_list = px.get_grayscale_data()
            line_status = px.get_line_status(gm_val_list)
            
            # Check if line is lost (all sensors see line)
            if line_status == [0, 0, 0]:
                outHandle()
                continue
            
            # Calculate error
            error = calculate_error(gm_val_list, line_status)
            
            # PD Control
            steering_angle = pd_control(error, last_error, dt)
            
            # Update last error for next iteration
            last_error = error
            
            # Apply control
            px.set_dir_servo_angle(steering_angle)
            px.forward(px_power)
            
            # Debug output
            print("Error: {:.3f}, Angle: {:.1f}°, Status: {}, Values: {}".format(
                error, steering_angle, line_status, gm_val_list))
            
            sleep(0.01)  # Small delay for control loop

    except KeyboardInterrupt:
        print("\nKeyboardInterrupt: stop and exit")

    finally:
        px.stop()
        print("stop and exit")
        sleep(0.1)
