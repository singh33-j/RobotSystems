# ============================================================
# SAFE LINE FOLLOWING — PiCar-X + Picamera2 (HEADLESS)
# ============================================================

from picamera2 import Picamera2
from picarx import Picarx
import cv2
import numpy as np
from time import sleep, time
import signal
import sys
import atexit

# ============================================================
# CONFIG
# ============================================================
FRAME_WIDTH  = 320
FRAME_HEIGHT = 240

BOTTOM_CROP_RATIO = 0.35
THRESHOLD = 60                 # black line on light floor
STEERING_GAIN = 0.35
BASE_SPEED = 25
SLOW_SPEED = 12

STEER_LIMIT = 30               # degrees

PRINT_PERIOD = 0.5             # seconds
HEADLESS = True                # MUST be True over SSH

MIN_WHITE_RATIO = 0.003
MAX_WHITE_RATIO = 0.25

# ============================================================
# INIT CAR
# ============================================================
print("[INIT] Initializing PiCar-X")
px = Picarx()
px.set_dir_servo_angle(0)
px.stop()
sleep(0.2)

# ============================================================
# HARD EMERGENCY STOP (MULTI-LAYER)
# ============================================================
def stop_all():
    try:
        px.set_dir_servo_angle(0)
        px.stop()
    except:
        pass

def emergency_stop(sig=None, frame=None):
    print(f"\n[EMERGENCY] Signal {sig} — stopping motors NOW")
    stop_all()
    sys.exit(0)

# Signal handlers
signal.signal(signal.SIGINT, emergency_stop)
signal.signal(signal.SIGTERM, emergency_stop)

# Runs on ANY Python exit (including crashes)
atexit.register(stop_all)

# ============================================================
# INIT CAMERA (Picamera2, no Qt)
# ============================================================
print("[INIT] Starting camera")
picam2 = Picamera2()
picam2.configure(
    picam2.create_video_configuration(
        main={"size": (FRAME_WIDTH, FRAME_HEIGHT),
              "format": "BGR888"}
    )
)
picam2.start()
sleep(1)

print("[INIT] Camera ready ✅")

# ============================================================
# MAIN LOOP
# ============================================================
last_print = time()
print("[RUN] Line following active")

try:
    while True:
        frame = picam2.capture_array()

        if frame is None:
            print("[WARN] No camera frame → STOPPING")
            stop_all()
            sleep(0.05)
            continue

        # ----------------------------
        # Vision pipeline
        # ----------------------------
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(
            gray, THRESHOLD, 255, cv2.THRESH_BINARY_INV
        )

        h = binary.shape[0]
        crop_y = int(h * (1 - BOTTOM_CROP_RATIO))
        roi = binary[crop_y:h, :]

        # ----------------------------
        # Camera confidence check
        # ----------------------------
        white_ratio = np.count_nonzero(roi) / roi.size

        now = time()
        do_print = (now - last_print) > PRINT_PERIOD

        if not (MIN_WHITE_RATIO < white_ratio < MAX_WHITE_RATIO):
            stop_all()

            if do_print:
                print(
                    f"[CAMERA] Floor not visible | white_ratio={white_ratio:.4f}"
                )
                last_print = now
            continue

        # ----------------------------
        # Line centroid
        # ----------------------------
        moments = cv2.moments(roi)

        if moments["m00"] > 0:
            cx = int(moments["m10"] / moments["m00"])
            error = cx - (FRAME_WIDTH // 2)

            steer = -STEERING_GAIN * error
            steer = max(min(steer, STEER_LIMIT), -STEER_LIMIT)

            px.set_dir_servo_angle(steer)
            px.forward(BASE_SPEED)

            if do_print:
                print(
                    f"[TRACK] cx={cx:3d} | err={error:4d} | "
                    f"steer={steer:6.2f} | speed={BASE_SPEED}"
                )
                last_print = now

        else:
            px.set_dir_servo_angle(0)
            px.forward(SLOW_SPEED)

            if do_print:
                print("[LOST] Line lost → slow")
                last_print = now

        # ----------------------------
        # GUI (DISABLED when headless)
        # ----------------------------
        if not HEADLESS:
            cv2.imshow("Binary ROI", roi)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

except KeyboardInterrupt:
    emergency_stop()

finally:
    print("[SHUTDOWN] Cleaning up")
    stop_all()
    picam2.stop()

    if not HEADLESS:
        cv2.destroyAllWindows()

    print("[SHUTDOWN] Done ✅")
