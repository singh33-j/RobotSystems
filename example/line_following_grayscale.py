import time
from picarx import Picarx

# =========================
# Parameters
# =========================
CONTROL_DT = 0.01            # control loop timestep [s]
BASE_SPEED = 18              # forward speed
REVERSE_SPEED = -14          # reverse speed
STEER_MAX = 30               # max steering angle [deg]

DROP_SUM_THRESH = 600        # <<< YOUR REQUEST
LOSS_CONFIRM_TIME = 0.5      # seconds before reversing
K_STEER = 14.0               # proportional steering gain

# =========================
# Init
# =========================
px = Picarx()
prev_adc = None

loss_start_time = None
last_seen_steer = 0.0

mode = "TRACK"

# =========================
# Helper functions
# =========================
def compute_error(adc):
    """
    Normalized line position error using weighted centroid.
    Returns value in [-1, 1].
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
        t_now = time.time()

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

        # BIG DROP = LINE LOSS SIGNAL
        big_drop = drop_sum >= DROP_SUM_THRESH

        # -------------------------
        # Loss timing logic
        # -------------------------
        if big_drop:
            if loss_start_time is None:
                loss_start_time = t_now
        else:
            loss_start_time = None

        loss_time = 0.0 if loss_start_time is None else (t_now - loss_start_time)
        line_lost = loss_start_time is not None

        # -------------------------
        # Control logic
        # -------------------------
        if mode == "TRACK":
            err = compute_error(adc)
            steer = clamp(K_STEER * err, -STEER_MAX, STEER_MAX)
            last_seen_steer = steer

            px.set_dir_servo_angle(steer)
            px.forward(BASE_SPEED)

            if line_lost and loss_time >= LOSS_CONFIRM_TIME:
                mode = "REVERSE"
                reverse_start_time = t_now

        else:  # REVERSE MODE
            reverse_duration = loss_time

            px.set_dir_servo_angle(last_seen_steer)
            px.forward(REVERSE_SPEED)

            if t_now - reverse_start_time >= reverse_duration:
                mode = "TRACK"
                loss_start_time = None

            err = 0.0
            steer = last_seen_steer

        # -------------------------
        # Debug print
        # -------------------------
        print(
            f"{mode} | "
            f"adc={list(map(int, adc))} | "
            f"drops={list(map(int, drops))} | "
            f"drop_sum={int(drop_sum)} | "
            f"line_lost={line_lost} | "
            f"loss_time={loss_time:.3f}s | "
            f"err={err:+.3f} | "
            f"steer={steer:+.1f}"
        )

        prev_adc = adc
        time.sleep(CONTROL_DT)

except KeyboardInterrupt:
    px.stop()
    print("Stopped.")
