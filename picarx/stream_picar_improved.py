from flask import Flask, Response
from picamera2 import Picamera2
import cv2
import time
from picarx_improved import Picarx

# ============================================================
# Configuration (MATCHES LINE FOLLOWER)
# ============================================================
FRAME_W, FRAME_H = 320, 240

ROI_Y_START = 160
ROI_HEIGHT  = 80

THRESH_VAL = 120
INVERT     = True

JPEG_QUALITY = 70

# ============================================================
# Init Flask + Robot
# ============================================================
app = Flask(__name__)

print("[INIT] Initializing PiCar-X")
px = Picarx()

print("[INIT] Setting camera tilt to -35 deg")
px.set_cam_tilt_angle(-35)
time.sleep(0.4)  # let servo settle

# ============================================================
# Init Camera
# ============================================================
print("[INIT] Starting Picamera2")
picam2 = Picamera2()
picam2.configure(
    picam2.create_video_configuration(
        main={"size": (FRAME_W, FRAME_H), "format": "RGB888"}
    )
)
picam2.start()
time.sleep(0.5)

print("[INIT] Camera started")

# ============================================================
# Frame generator
# ============================================================
def gen_frames():
    while True:
        frame = picam2.capture_array()  # RGB888

        # --- grayscale ---
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

        # --- ROI (EXACT SAME AS LINE FOLLOWER) ---
        roi = gray[
            ROI_Y_START : ROI_Y_START + ROI_HEIGHT,
            :
        ]

        # --- threshold ---
        thresh_type = cv2.THRESH_BINARY_INV if INVERT else cv2.THRESH_BINARY
        _, binary = cv2.threshold(roi, THRESH_VAL, 255, thresh_type)

        # --- upscale for visibility ---
        vis = cv2.resize(
            binary,
            None,
            fx=2.0,
            fy=2.0,
            interpolation=cv2.INTER_NEAREST
        )

        ok, jpg = cv2.imencode(
            ".jpg",
            vis,
            [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY]
        )
        if not ok:
            continue

        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n"
            + jpg.tobytes()
            + b"\r\n"
        )

# ============================================================
# Routes
# ============================================================
@app.route("/")
def index():
    return (
        "<h2>PiCar-X Threshold Debug Stream</h2>"
        "<p><a href='/video'>Open /video</a></p>"
        "<p>ROI: y=160:240, THRESH_BINARY_INV</p>"
    )

@app.route("/video")
def video():
    return Response(
        gen_frames(),
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )

# ============================================================
# Main
# ============================================================
if __name__ == "__main__":
    print("[MAIN] Threshold stream running on port 8080")
    app.run(host="0.0.0.0", port=8080, threaded=True)
