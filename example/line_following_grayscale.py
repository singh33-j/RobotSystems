import time
from picarx import Picarx

# =========================
# Parameters
# =========================
CONTROL_DT = 0.01        # loop timestep [s]
BASE_SPEED = 18          # forward speed
REVERSE_SPEED = -14      # reverse speed
STEER_MAX = 30           # max steering angle [deg]

DROP_SUM_THRESH = 600    # <<< line loss threshold (your request)
K_STEER = 14.0           # steering gain

# =========================
# Init
# =========================
px = Picarx()
prev_adc = None
last_seen_steer = 0.0

# =========================
# Helpers
# =========================
def compute_error(adc):
    """
    Normalized centroid error in [-1, 1]
    """
    weights = [-1.0, 0.0, 1.0]
    s = sum(adc)
    if s < 1e-6:
        return 0.0
    return sum(w * a for w, a in zip(weights, adc)) / s


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


# =========================
# Main Loop
# =========================
try:
    while True:
        adc = px.get_grayscale_data()
        if adc is None:
            time.sleep(CONTROL_DT)
            continue

        # -------------------------
        # Brightness drop detection
        # -------------------------
        if prev_adc is None:
            drops = [0, 0, 0]
        else:
            drops = [max(prev_adc[i] - adc[i], 0) for i in range(3)]

        drop_sum = sum(drops)
        line_lost = drop_sum >= DROP_SUM_THRESH

        # -------------------------
        # Control
        # -------------------------
        if line_lost:
            mode = "REVERSE"
            steer = last_seen_steer
            err = 0.0

            px.set_dir_servo_angle(steer)
            px.forward(REVERSE_SPEED)

        else:
            mode = "TRACK"
            err = compute_error(adc)
            steer = clamp(K_STEER * err, -STEER_MAX, STEER_MAX)
            last_seen_steer = steer

            px.set_dir_servo_angle(steer)
            px.forward(BASE_SPEED)

        # -------------------------
        # Debug print
        # -------------------------
        print(
            f"{mode} | "
            f"adc={list(map(int, adc))} | "
            f"drops={list(map(int, drops))} | "
            f"drop_sum={int(drop_sum)} | "
            f"line_lost={line_lost} | "
            f"err={err:+.3f} | "
            f"steer={steer:+.1f}"
        )

        prev_adc = adc
        time.sleep(CONTROL_DT)

except KeyboardInterrupt:
    px.stop()
    print("Stopped.")
