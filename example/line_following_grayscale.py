from picarx import Picarx
import time

# -----------------------------
# Robot setup
# -----------------------------
px = Picarx()

# -----------------------------
# Tunable parameters
# -----------------------------
BASE_SPEED        = 12
REVERSE_SPEED     = -10
KP_STEER          = 35
MAX_STEER         = 30

LINE_LOST_TIME    = 0.5          # seconds
BRIGHTNESS_JUMP   = 600          # per-sensor threshold

CONTROL_DT        = 0.05

# -----------------------------
# States
# -----------------------------
TRACKING   = "TRACKING"
LOST_WAIT  = "LOST_WAIT"
RECOVERING = "RECOVERING"

state = TRACKING

lost_start_time = None
last_valid_steer = 0.0
last_good_adc = None

# -----------------------------
# Helper functions
# -----------------------------
def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def compute_error(adc):
    left, mid, right = adc
    s = left + mid + right
    if s < 1e-6:
        return 0.0
    return (-left + right) / s

def all_sensors_jumped(adc, ref):
    return all(abs(a - r) > BRIGHTNESS_JUMP for a, r in zip(adc, ref))

# -----------------------------
# Main loop
# -----------------------------
print("Line following with 3-sensor brightness loss detection")
time.sleep(1)

try:
    while True:
        now = time.time()
        adc = px.get_grayscale_data()

        # Initialize reference
        if last_good_adc is None:
            last_good_adc = adc[:]

        jumped_all = all_sensors_jumped(adc, last_good_adc)

        # -------------------------------------------------
        # TRACKING
        # -------------------------------------------------
        if state == TRACKING:

            if jumped_all:
                state = LOST_WAIT
                lost_start_time = now
                px.stop()

                print("→ LOST_WAIT (3-sensor brightness jump)", adc)

            else:
                err = compute_error(adc)
                steer = clamp(KP_STEER * err, -MAX_STEER, MAX_STEER)

                px.set_dir_servo_angle(steer)
                px.forward(BASE_SPEED)

                last_valid_steer = steer
                last_good_adc = adc[:]   # update reference only when tracking

                print(f"TRACK | adc={adc} | err={err:+.3f} | steer={steer:+.1f}")

        # -------------------------------------------------
        # LOST_WAIT (debounce)
        # -------------------------------------------------
        elif state == LOST_WAIT:

            if not jumped_all:
                state = TRACKING
                last_good_adc = adc[:]
                print("← false alarm, back to TRACKING")

            elif now - lost_start_time >= LINE_LOST_TIME:
                state = RECOVERING
                px.set_dir_servo_angle(last_valid_steer)
                px.backward(abs(REVERSE_SPEED))

                print("→ RECOVERING")

            else:
                px.stop()

        # -------------------------------------------------
        # RECOVERING
        # -------------------------------------------------
        elif state == RECOVERING:

            if not jumped_all:
                state = TRACKING
                last_good_adc = adc[:]
                print("← line reacquired, TRACKING")

            else:
                px.set_dir_servo_angle(last_valid_steer)
                px.backward(abs(REVERSE_SPEED))

                print(f"RECOVER | adc={adc} | steer={last_valid_steer:+.1f}")

        time.sleep(CONTROL_DT)

except KeyboardInterrupt:
    px.stop()
    print("\nStopped.")
