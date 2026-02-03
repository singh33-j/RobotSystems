import cv2
import numpy as np
from picarx import Picarx
from time import sleep, time

# ------------------------------------------------------------
# Parameters
# ------------------------------------------------------------
FRAME_WIDTH  = 320
FRAME_HEIGHT = 240

BOTTOM_CROP_RATIO = 0.35
STEERING_GAIN = 0.35
BASE_SPEED = 25
THRESHOLD = 60

PRINT_PERIOD = 0.5   # seconds between diagnostic prints

# ------------------------------------------------------------
# Initialize car + camera
# ------------------------------------------------------------
print("[INIT] Initializing Picar-X...")
px = Picarx()
px.set_dir_servo_angle(0)
px.forward(0)

print("[INIT] Opening camera...")
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

sleep(1)

if not cap.isOpened():
    raise RuntimeError("Camera failed to open")

print("[INIT] System ready ✅")

last_print = time()

try:
    px.forward(BASE_SPEED)
    print(f"[RUN] Starting line following | speed={BASE_SPEED}")

    # --------------------------------------------------------
    # Main control loop
    # --------------------------------------------------------
    while True:
        ret, frame = cap.read()
        if not ret:
            print("[WARN] Camera frame not received")
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        _, binary = cv2.threshold(
            gray, THRESHOLD, 255, cv2.THRESH_BINARY_INV
        )

        h = binary.shape[0]
        crop_y = int(h * (1 - BOTTOM_CROP_RATIO))
        roi = binary[crop_y:h, :]

        moments = cv2.moments(roi)

        now = time()
        do_print = (now - last_print) > PRINT_PERIOD

        if moments["m00"] > 0:
            cx = int(moments["m10"] / moments["m00"])
            error = cx - (FRAME_WIDTH // 2)

            steering_angle = -STEERING_GAIN * error
            steering_angle = max(min(steering_angle, 30), -30)

            px.set_dir_servo_angle(steering_angle)
            px.forward(BASE_SPEED)

            if do_print:
                print(
                    f"[TRACK] cx={cx:3d} | error={error:4d} | "
                    f"steer={steering_angle:6.2f} | speed={BASE_SPEED}"
                )

        else:
            px.set_dir_servo_angle(0)
            px.forward(15)

            if do_print:
                print("[LOST] Line lost → slowing + straightening")

        if do_print:
            last_print = now

        # Debug window
        cv2.line(
            roi,
            (FRAME_WIDTH // 2, 0),
            (FRAME_WIDTH // 2, roi.shape[0]),
            128,
            2
        )
        cv2.imshow("Binary ROI", roi)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("[EXIT] 'q' pressed — exiting loop")
            break

except KeyboardInterrupt:
    print("\n[INTERRUPT] Ctrl-C detected!")

finally:
    # --------------------------------------------------------
    # Guaranteed cleanup
    # --------------------------------------------------------
    print("[SHUTDOWN] Stopping motors...")
    px.set_dir_servo_angle(0)
    px.stop()
    sleep(0.1)

    print("[SHUTDOWN] Releasing camera + closing windows")
    cap.release()
    cv2.destroyAllWindows()

    print("[SHUTDOWN] Clean shutdown complete ✅")
