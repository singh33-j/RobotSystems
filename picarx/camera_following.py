import cv2
import numpy as np
from picarx import Picarx
from time import sleep

# ------------------------------------------------------------
# Parameters
# ------------------------------------------------------------
FRAME_WIDTH  = 320
FRAME_HEIGHT = 240

BOTTOM_CROP_RATIO = 0.35   # bottom 35% of frame
STEERING_GAIN = 0.35       # proportional steering gain
BASE_SPEED = 25            # forward speed

THRESHOLD = 60             # grayscale threshold (tune this!)

# ------------------------------------------------------------
# Initialize car + camera
# ------------------------------------------------------------
px = Picarx()
px.forward(BASE_SPEED)

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

sleep(1)

# ------------------------------------------------------------
# Main control loop
# ------------------------------------------------------------
while True:
    ret, frame = cap.read()
    if not ret:
        continue

    # 1. Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # 2. Binary threshold (black line -> white)
    _, binary = cv2.threshold(
        gray, THRESHOLD, 255, cv2.THRESH_BINARY_INV
    )

    # 3. Crop bottom region
    h = binary.shape[0]
    crop_y = int(h * (1 - BOTTOM_CROP_RATIO))
    roi = binary[crop_y:h, :]

    # 4. Find centroid of white pixels
    moments = cv2.moments(roi)

    if moments["m00"] > 0:
        cx = int(moments["m10"] / moments["m00"])
        error = cx - (FRAME_WIDTH // 2)

        # 5. Steering control
        steering_angle = -STEERING_GAIN * error
        steering_angle = max(min(steering_angle, 30), -30)

        px.set_dir_servo_angle(steering_angle)
        px.forward(BASE_SPEED)

    else:
        # Line lost → slow & straighten
        px.set_dir_servo_angle(0)
        px.forward(15)

    # Optional debug window
    cv2.line(roi, (FRAME_WIDTH//2, 0),
             (FRAME_WIDTH//2, roi.shape[0]), 128, 2)
    cv2.imshow("Binary ROI", roi)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# ------------------------------------------------------------
# Cleanup
# ------------------------------------------------------------
cap.release()
cv2.destroyAllWindows()
px.stop()
