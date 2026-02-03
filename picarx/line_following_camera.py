import cv2
import numpy as np
from time import sleep
from picamera2 import Picamera2
from picarx_improved import Picarx


# ============================================================
# configuration
# ============================================================

FRAME_WIDTH  = 320
FRAME_HEIGHT = 240

# MATCHES STREAMER (validated)
ROI_Y_START = 160
ROI_HEIGHT  = 80

THRESH_VAL = 120

MAX_STEER_DEG = 30.0
STEER_GAIN    = 0.3      # no deadzone

FORWARD_SPEED = 28


# ============================================================
# camera line follower (Picamera2, headless)
# ============================================================

class CameraLineFollower:
    def __init__(self):
        print("[INIT] starting Picamera2")

        self.picam2 = Picamera2()
        self.picam2.configure(
            self.picam2.create_preview_configuration(
                main={
                    "size": (FRAME_WIDTH, FRAME_HEIGHT),
                    "format": "RGB888"
                }
            )
        )
        self.picam2.start()

        # Camera warm-up
        sleep(0.5)

        print("[INIT] camera started")

    def read_error(self):
        frame = self.picam2.capture_array()

        if frame is None:
            print("[CAM] no frame captured")
            return None

        h, w, _ = frame.shape
        print(f"[CAM] frame {w}x{h}")

        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

        # Region of Interest (EXACT SAME AS STREAM)
        roi = gray[
            ROI_Y_START : ROI_Y_START + ROI_HEIGHT,
            :
        ]

        print(
            f"[ROI] y={ROI_Y_START}:{ROI_Y_START + ROI_HEIGHT} "
            f"shape={roi.shape}"
        )

        # Threshold (black line on light floor)
        _, binary = cv2.threshold(
            roi,
            THRESH_VAL,
            255,
            cv2.THRESH_BINARY_INV
        )

        white_px = cv2.countNonZero(binary)
        total_px = roi.size
        fill_ratio = white_px / total_px

        print(
            f"[THRESH] white_px={white_px} "
            f"fill_ratio={fill_ratio:.4f}"
        )

        # Reject weak detections (blank table / glare)
        if white_px < 200:
            print("[REJECT] too few white pixels → no line")
            return None

        moments = cv2.moments(binary)
        if moments["m00"] == 0:
            print("[REJECT] zero moment → centroid undefined")
            return None

        # Centroid calculation
        cx = moments["m10"] / moments["m00"]
        center_x = w / 2
        error_px = cx - center_x

        print(
            f"[CENTROID] cx={cx:.1f} | "
            f"center_x={center_x:.1f} | "
            f"error_px={error_px:+.1f}"
        )

        return int(error_px)

    def close(self):
        print("[CLEANUP] stopping camera")
        self.picam2.stop()


# ============================================================
# main loop
# ============================================================

if __name__ == "__main__":

    print("[MAIN] PiCar-X camera line follower starting")

    px = Picarx()

    # --- camera tilt (critical) ---
    print("[INIT] setting camera tilt to -35 deg")
    px.set_cam_tilt_angle(-45)
    sleep(0.4)  # let servo settle

    follower = CameraLineFollower()

    try:
        while True:
            error_px = follower.read_error()

            # ------------------------------------------------
            # FAIL-SAFE: stop if vision is lost
            # ------------------------------------------------
            if error_px is None:
                print("[SAFE] vision lost → STOP")
                px.stop()
                sleep(0.1)
                continue

            # Proportional steering (no deadzone)
            steer = STEER_GAIN * error_px
            steer = max(-MAX_STEER_DEG, min(MAX_STEER_DEG, steer))

            print(
                f"[CTRL] error_px={error_px:+d} | "
                f"steer={steer:+.2f} deg | "
                f"speed={FORWARD_SPEED}"
            )

            px.set_dir_servo_angle(steer)
            px.forward(FORWARD_SPEED)

            sleep(0.02)

    except KeyboardInterrupt:
        print("\n[EXIT] Ctrl-C received → stopping robot")

    finally:
        px.stop()
        follower.close()
