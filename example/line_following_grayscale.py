from picarx import Picarx
import time

# -----------------------------
# Robot setup
# -----------------------------
px = Picarx()

# -----------------------------
# Tunable parameters
# -----------------------------
BASE_SPEED        = 12          # forward speed
REVERSE_SPEED     = -10         # reverse speed
KP_STEER          = 35          # proportional steering gain (deg per error)
MAX_STEER         = 30          # steering limit (deg)

LINE_LOST_TIME    = 0.5         # seconds line must be gone before recovery
BRIGHTNESS_JUMP   = 700         # brightness range threshold for "off line"
ASYM_THRESHOLD    = 60          # min L/R diff to consider valid asymmetry

CONTROL_DT        = 0.05        # loop period (s)

# -----------------------------
# State variables
# -----------------------------
STATE_TRACKING   = "TRACKING"
STATE_LOST_WAIT  = "LOST_WAIT"
STATE_RECOVERING = "RECOVERING"

state = STATE_TRACKING

lost_start_time   = None
recover_start_time = None

last_valid_steer  = 0.0
last_seen_time    = time.time()

# -----------------------------
# Helper functions
# -----------------------------
def read_grayscale():
    return px.get_grayscale_data()

def clamp(val, lo, hi):
    return max(lo, min(hi, val))

def brightness_range(v):
    return max(v) - min(v)

def compute_error(v):
    """
    Normalized weighted error:
      left  -> -1
      mid   ->  0
      right -> +1
    """
    left, mid, right = v
    total = left + mid + right
    if total < 1e-6:
        return 0.0

    pos = (-1 * left + 0 * mid + 1 * right) / total
    return pos

def asymmetry(v):
    return abs(v[0] - v[2])

# -----------------------------
# Main control loop
# -----------------------------
print("Starting line following with recovery...")
time.sleep(1)

try:
    while True:
        t_now = time.time()
        adc = read_grayscale()

        bright_rng = brightness_range(adc)
        asym = asymmetry(adc)

        # -----------------------------
        # TRACKING STATE
        # -----------------------------
        if state == STATE_TRACKING:

            # Detect possible line loss
            if bright_rng > BRIGHTNESS_JUMP:
                state = STATE_LOST_WAIT
                lost_start_time = t_now

                px.set_motor_speed(0, 0)
                print("→ LOST_WAIT (brightness jump)", adc)

            else:
                # Normal tracking
                err = compute_error(adc)
                steer = KP_STEER * err
                steer = clamp(steer, -MAX_STEER, MAX_STEER)

                px.set_dir_servo_angle(steer)
                px.forward(BASE_SPEED)

                last_valid_steer = steer
                last_seen_time = t_now

                print(f"TRACK | adc={adc} | err={err:+.3f} | steer={steer:+.1f}")

        # -----------------------------
        # LOST WAIT (debounce)
        # -----------------------------
        elif state == STATE_LOST_WAIT:

            # Line reappeared → return to tracking
            if bright_rng <= BRIGHTNESS_JUMP:
                state = STATE_TRACKING
                print("← recovered quickly, back to TRACKING")

            # Line gone long enough → recover
            elif t_now - lost_start_time >= LINE_LOST_TIME:
                state = STATE_RECOVERING
                recover_start_time = t_now

                px.set_dir_servo_angle(last_valid_steer)
                px.backward(abs(REVERSE_SPEED))

                print("→ RECOVERING (reverse)")

            else:
                px.set_motor_speed(0, 0)

        # -----------------------------
        # RECOVERY STATE
        # -----------------------------
        elif state == STATE_RECOVERING:

            # If contrast normal again → resume tracking
            if bright_rng <= BRIGHTNESS_JUMP and asym > ASYM_THRESHOLD:
                state = STATE_TRACKING
                print("← line reacquired, TRACKING")

            else:
                px.set_dir_servo_angle(last_valid_steer)
                px.backward(abs(REVERSE_SPEED))

                print(f"RECOVER | adc={adc} | steer={last_valid_steer:+.1f}")

        time.sleep(CONTROL_DT)

except KeyboardInterrupt:
    print("\nStopping robot.")
    px.stop()
