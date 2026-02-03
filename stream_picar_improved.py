from flask import Flask, Response
from picamera2 import Picamera2
import cv2
import numpy as np
import time

# =========================
# Configuration
# =========================
FRAME_W, FRAME_H = 640, 480

ROI_Y_START = 320      # bottom part of image
ROI_HEIGHT  = 120

THRESH_VAL = 120       # adjust while watching stream
INVERT     = True      # True for black line on light floor

JPEG_QUALITY = 70

# =========================
# Camera + Flask
# =========================
app = Flask(__name__)

picam2 = Picamera2()
picam2.configure(
    picam2.create_video_configuration(
        main={"size": (FRAME_W, FRAME_H), "format": "RGB888"}
    )
)
picam2.start()
time.sleep(0.5)

print("[INIT] Picamera2 started")

# =========================
# Frame generator
# =========================
def gen_frames():
    while True:
        frame = picam2.capture_array()  # RGB888

        # --- grayscale ---
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

        # --- ROI ---
        roi = gray[
            ROI_Y_START : ROI_Y_START + ROI_HEIGHT,
            :
        ]

        # --- threshold ---
        thresh_type = cv2.THRESH_BINARY_INV if INVERT else cv2.THRESH_BINARY
        _, binary = cv2.threshold(roi, THRESH_VAL, 255, thresh_type)

        # --- visualize: upscale for easier viewing ---
        vis = cv2.resize(binary, None, fx=1.5, fy=1.5, interpolation=cv2.INTER_NEAREST)

        # --- encode JPEG ---
        ok, jpg = cv2.imencode(
            ".jpg",
            vis,
            [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY]
        )
        if not ok:
            continue

        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n" +
            jpg.tobytes() +
            b"\r\n"
        )

# =========================
# Routes
# =========================
@app.route("/")
def index():
    return (
        "<h2>Threshold Debug Stream</h2>"
        "<p><a href='/video'>Open /video</a></p>"
        "<p>Adjust THRESH_VAL, ROI_Y_START, ROI_HEIGHT in code.</p>"
    )

@app.route("/video")
def video():
    return Response(
        gen_frames(),
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )

# =========================
# Main
# =========================
if __name__ == "__main__":
    print("[MAIN] Streaming thresholded video on port 8080")
    app.run(host="0.0.0.0", port=8080, threaded=True)
