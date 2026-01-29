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

FORWARD_SPEED = 18


# ============================================================
# camera line follower with diagnostics
# ============================================================

class CameraLineFollower:
    def __init__(self):
        self.cap = cv2.VideoCapture(CAM_INDEX)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

        if not self.cap.isOpened():
            print("camera not opened")

    def read_error(self):
        ret, frame = self.cap.read()

        if not ret:
            print("no frame captured")
            return None

        h, w, _ = frame.shape
        print(f"frame size {w}x{h}")

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        roi = gray[
            ROI_Y_START:ROI_Y_START + ROI_HEIGHT,
            :
        ]

        print(f"roi shape {roi.shape}")

        _, binary = cv2.threshold(
            roi, THRESH_VAL, 255, cv2.THRESH_BINARY_INV
        )

        white_px = cv2.countNonZero(binary)
        print(f"binary white pixels {white_px}")

        if white_px < 200:
            print("binary too empty, likely no line detected")
            cv2.imshow("binary", binary)
            cv2.waitKey(1)
            return None

        moments = cv2.moments(binary)

        if moments["m00"] == 0:
            print("zero moments, centroid undefined")
            cv2.imshow("binary", binary)
            cv2.waitKey(1)
            return None

        cx = int(moments["m10"] / moments["m00"])
        center_x = w // 2
        error_px = cx - center_x

        print(
            f"centroid_x {cx} | "
            f"center_x {center_x} | "
            f"error_px {error_px}"
        )

        vis = cv2.cvtColor(roi, cv2.COLOR_GRAY2BGR)
        cv2.line(vis, (center_x, 0), (center_x, ROI_HEIGHT), (255, 0, 0), 2)
        cv2.circle(vis, (cx, ROI_HEIGHT // 2), 6, (0, 0, 255), -1)
        cv2.imshow("roi", vis)
        cv2.waitKey(1)

        return error_px

    def close(self):
        self.cap.release()
        cv2.destroyAllWindows()


# ============================================================
# main loop
# ============================================================

if __name__ == "__main__":

    px = Picarx()
    follower = CameraLineFollower()

    try:
        while True:
            error_px = follower.read_error()

            if error_px is None:
                print("no error -> steering 0")
                px.set_dir_servo_angle(0)
                px.forward(10)
                sleep(0.05)
                continue

            steer = STEER_GAIN * error_px
            steer = max(-MAX_STEER_DEG, min(MAX_STEER_DEG, steer))

            if abs(steer) < STEER_DEADZONE:
                steer = 0.0

            print(
                f"steer command {steer:+.2f} deg"
            )

            px.set_dir_servo_angle(steer)
            px.forward(FORWARD_SPEED)

            sleep(0.02)

    except KeyboardInterrupt:
        px.stop()
        follower.close()
