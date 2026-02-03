import cv2
import numpy as np
from time import sleep
from picarx_improved import Picarx


# ============================================================
# configuration
# ============================================================

CAM_INDEX = 0
FRAME_WIDTH = 320
FRAME_HEIGHT = 240

ROI_Y_START = 160
ROI_HEIGHT  = 80

THRESH_VAL = 120

MAX_STEER_DEG = 30.0
STEER_GAIN = 0.15
STEER_DEADZONE = 2.0

FORWARD_SPEED = 28


# ============================================================
# camera line follower with diagnostics
# ============================================================

class CameraLineFollower:
    def __init__(self):
        print("[INIT] opening camera")
        self.cap = cv2.VideoCapture(CAM_INDEX)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

        if not self.cap.isOpened():
            print("[ERROR] camera not opened")
        else:
            print("[INIT] camera opened successfully")

    def read_error(self):
        ret, frame = self.cap.read()

        if not ret:
            print("[CAM] no frame captured")
            return None

        h, w, _ = frame.shape
        print(f"[CAM] frame size {w}x{h}")

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        roi = gray[
            ROI_Y_START : ROI_Y_START + ROI_HEIGHT,
            :
        ]

        print(
            f"[ROI] y={ROI_Y_START}:{ROI_Y_START + ROI_HEIGHT} "
            f"shape={roi.shape}"
        )

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
            f"total_px={total_px} "
            f"fill_ratio={fill_ratio:.4f}"
        )

        if white_px < 200:
            print("[REJECT] too few white pixels → likely no line")
            cv2.imshow("binary", binary)
            cv2.waitKey(1)
            return None

        moments = cv2.moments(binary)

        if moments["m00"] == 0:
            print("[REJECT] zero moment m00 → centroid undefined")
            cv2.imshow("binary", binary)
            cv2.waitKey(1)
            return None

        cx = moments["m10"] / moments["m00"]
        center_x = w / 2
        error_px = cx - center_x

        print(
            f"[CENTROID] cx={cx:.1f} | "
            f"center_x={center_x:.1f} | "
            f"error_px={error_px:+.1f}"
        )

        # Visualization
        vis = cv2.cvtColor(roi, cv2.COLOR_GRAY2BGR)

        cv2.line(
            vis,
            (int(center_x), 0),
            (int(center_x), ROI_HEIGHT),
            (255, 0, 0),
            2
        )

        cv2.circle(
            vis,
            (int(cx), ROI_HEIGHT // 2),
            6,
            (0, 0, 255),
            -1
        )

        cv2.imshow("roi", vis)
        cv2.imshow("binary", binary)
        cv2.waitKey(1)

        return int(error_px)

    def close(self):
        print("[CLEANUP] releasing camera")
        self.cap.release()
        cv2.destroyAllWindows()


# ============================================================
# main loop
# ============================================================

if __name__ == "__main__":

    print("[MAIN] starting PiCar-X line follower")

    px = Picarx()
    follower = CameraLineFollower()

    try:
        while True:
            error_px = follower.read_error()

            if error_px is None:
                print("[CTRL] no error → steering = 0")
                px.set_dir_servo_angle(0)
                px.forward(FORWARD_SPEED)
                sleep(0.05)
                continue

            steer = STEER_GAIN * error_px
            steer = max(-MAX_STEER_DEG, min(MAX_STEER_DEG, steer))

            if abs(steer) < STEER_DEADZONE:
                print(
                    f"[CTRL] steer {steer:+.2f} "
                    f"inside deadzone → 0"
                )
                steer = 0.0

            print(
                f"[CTRL] error_px={error_px:+d} | "
                f"steer_cmd={steer:+.2f} deg | "
                f"speed={FORWARD_SPEED}"
            )

            px.set_dir_servo_angle(steer)
            px.forward(FORWARD_SPEED)

            sleep(0.02)

    except KeyboardInterrupt:
        print("\n[EXIT] keyboard interrupt → stopping")
        px.stop()
        follower.close()
