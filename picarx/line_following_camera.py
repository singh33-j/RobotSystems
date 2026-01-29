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

ROI_Y_START = 160     # bottom third
ROI_HEIGHT  = 80

THRESH_VAL = 80      # fixed since lighting is constant

MAX_STEER_DEG = 30.0
STEER_GAIN = 0.15     # pixels -> degrees
STEER_DEADZONE = 2.0

FORWARD_SPEED = 28


# ============================================================
# camera line follower
# ============================================================

class CameraLineFollower:
    def __init__(self):
        self.cap = cv2.VideoCapture(CAM_INDEX)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

    def read_error(self):
        ret, frame = self.cap.read()
        if not ret:
            return None

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        roi = gray[
            ROI_Y_START:ROI_Y_START + ROI_HEIGHT,
            :
        ]

        _, binary = cv2.threshold(
            roi, THRESH_VAL, 255, cv2.THRESH_BINARY_INV
        )

        kernel = np.ones((5, 5), np.uint8)
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)

        moments = cv2.moments(binary)

        if moments["m00"] == 0:
            return None

        cx = int(moments["m10"] / moments["m00"])
        center_x = FRAME_WIDTH // 2

        error_pixels = cx - center_x

        # visualize (optional)
        cv2.circle(roi, (cx, ROI_HEIGHT // 2), 5, 255, -1)
        cv2.imshow("roi", roi)
        cv2.waitKey(1)

        return error_pixels

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
                px.set_dir_servo_angle(0)
                px.forward(28)
                continue

            steer = STEER_GAIN * error_px
            steer = max(-MAX_STEER_DEG, min(MAX_STEER_DEG, steer))

            if abs(steer) < STEER_DEADZONE:
                steer = 0.0

            px.set_dir_servo_angle(steer)
            px.forward(FORWARD_SPEED)

            print(
                f"TRACK | err_px={error_px:+4d} | steer={steer:+.1f}"
            )

            sleep(0.02)

    except KeyboardInterrupt:
        px.stop()
        follower.close()
