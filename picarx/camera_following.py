from picamera2 import Picamera2
import cv2
import numpy as np
from picarx import Picarx
from time import sleep, time
import signal
import sys

# ============================================================
# PARAMETERS (TUNE THESE)
# ============================================================
FRAME_WIDTH  = 320
FRAME_HEIGHT = 240

BOTTOM_CROP_RATIO = 0.35        # Use bottom 35% of image
THRESHOLD = 60                 # Black line threshold
STEERING_GAIN = 0.35
BASE_SPEED = 25
SLOW_SPEED = 15

STEER_LIMIT = 30               # degrees
PRINT_PERIOD = 0.5             # seconds

MIN_WHITE_RATIO = 0.003        # camera pointing check
MAX_WHITE_RATIO = 0.25

# ============================================================
# INITIALIZE CAR
# ============================================================
print("[INIT] Initializing PiCar-X")
px = Picarx()
px.set_dir_servo_angle(0)
px.stop()

# ============================================================
# EMERGENCY STOP (Ctrl-C, kill, SSH drop)
# ============================================================
def emergency_stop(sig=None, frame=None):
    print("\n[EMERGENCY] Stopping motors NOW")
    px.set_dir_servo_angle(0)
    px.stop()
    sleep(0.1)
    sys.exit(0)

signal.signal(signal.SIGINT, emergency_stop)
signal.signal(signal.SIGTERM, emergency_stop)

# ============================================================
# INITIALIZE CAMERA (Picamera2)
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
print("[RUN] Line following started")

try:
    px.forward(BASE_SPEED)

    while True:
        frame = picam2.capture_array()
        if frame is None:
            print("[WARN] No camera frame → STOPPING")
            px.stop()
            continue

        # ----------------------------
        # Convert to grayscale
        # ----------------------------
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        _, binary = cv2.threshold(
            gray, THRESHOLD, 255, cv2.THRESH_BINARY_INV
        )

        # ----------------------------
        # Crop bottom ROI
        # ----------------------------
        h = binary.shape[0]
        crop_y = int(h * (1 - BOTTOM_CROP_RATIO))
        roi = binary[crop_y:h, :]

        # ----------------------------
        # Camera orientation check
        # ----------------------------
        white_pixels = np.count_nonzero(roi)
        white_ratio = white_pixels / roi.size

        now = time()
        do_print = (now - last_print) > PRINT_PERIOD

        if not (MIN_WHITE_RATIO < white_ratio < MAX_WHITE_RATIO):
            px.set_dir_servo_angle(0)
            px.stop()

            if do_print:
                print(
                    f"[CAMERA] No floor detected | white_ratio={white_ratio:.4f}"
                )
                last_print = now
            continue

        # ----------------------------
        # Centroid detection
        # ----------------------------
        moments = cv2.moments(roi)

        if moments["m00"] > 0:
            cx = int(moments["m10"] / moments["m00"])
            error = cx - (FRAME_WIDTH // 2)

            steering = -STEERING_GAIN * error
            steering = max(min(steering, STEER_LIMIT), -STEER_LIMIT)

            px.set_dir_servo_angle(steering)
            px.forward(BASE_SPEED)

            if do_print:
                print(
                    f"[TRACK] cx={cx:3d} | err={error:4d} | "
                    f"steer={steering:6.2f} | speed={BASE_SPEED}"
                )
                last_print = now

        else:
            px.set_dir_servo_angle(0)
            px.forward(SLOW_SPEED)

            if do_print:
                print("[LOST] Line lost → slow + straight")
                last_print = now

        # ----------------------------
        # Debug visualization
        # ----------------------------
        cv2.line(
            roi,
            (FRAME_WIDTH // 2, 0),
            (FRAME_WIDTH // 2, roi.shape[0]),
            128,
            2
        )
        cv2.imshow("Binary ROI", roi)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("[EXIT] 'q' pressed")
            break

except KeyboardInterrupt:
    emergency_stop()

finally:
    print("[SHUTDOWN] Cleaning up")
    px.set_dir_servo_angle(0)
    px.stop()
    picam2.stop()
    cv2.destroyAllWindows()
    print("[SHUTDOWN] Done ✅")
