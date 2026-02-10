import cv2
import numpy as np
from time import sleep, time
from threading import Event
from concurrent.futures import ThreadPoolExecutor

# Bus locking per manual
from readerwriterlock import rwlock

from picamera2 import Picamera2
from picarx_improved import Picarx


# ============================================================
# configuration
# ============================================================

FRAME_WIDTH  = 320
FRAME_HEIGHT = 240

# MATCHES STREAMER (validated)
ROI_Y_START = 160
ROI_HEIGHT  = 120

THRESH_VAL = 120

MAX_STEER_DEG = 30.0
STEER_GAIN    = 0.6      # no deadzone

FORWARD_SPEED = 28

# Thread rates (keep modest to reduce I2C contention)
SENSOR_DT      = 0.03    # camera capture + preprocess
INTERPRETER_DT = 0.03
CONTROL_DT     = 0.02

# Detection gating (same intent as your code)
MIN_WHITE_PX = 200


# ============================================================
# BUS (broadcast, latest-value) WITH LOCKING
# ============================================================

class Bus:
    def __init__(self, initial=None):
        self.message = initial
        self.lock = rwlock.RWLockWriteD()  # writer-priority lock

    def write(self, message):
        with self.lock.gen_wlock():
            self.message = message

    def read(self):
        with self.lock.gen_rlock():
            message = self.message
        return message


# ============================================================
# CAMERA SENSOR (producer)
# Publishes: {"roi_gray": roi_gray, "frame_w": w, "ts": t}
# ============================================================

class CameraSensor:
    def __init__(self):
        print("[INIT] starting Picamera2")

        self.picam2 = Picamera2()
        self.picam2.configure(
            self.picam2.create_preview_configuration(
                main={"size": (FRAME_WIDTH, FRAME_HEIGHT), "format": "RGB888"}
            )
        )
        self.picam2.start()

        sleep(0.5)  # warm-up
        print("[INIT] camera started")

    def read_roi_gray(self):
        frame = self.picam2.capture_array()
        if frame is None:
            return None

        h, w, _ = frame.shape

        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

        # ROI (same slice as your original)
        roi = gray[ROI_Y_START:ROI_Y_START + ROI_HEIGHT, :]

        # Important: publish a copy so other threads don't see mutation
        return roi.copy(), w

    def close(self):
        print("[CLEANUP] stopping camera")
        self.picam2.stop()


def sensor_task(sensor_bus, shutdown_event, dt):
    cam = CameraSensor()
    try:
        while not shutdown_event.is_set():
            out = cam.read_roi_gray()
            if out is None:
                sensor_bus.write({"valid": False, "reason": "no_frame", "ts": time()})
                sleep(dt)
                continue

            roi_gray, frame_w = out
            sensor_bus.write({
                "valid": True,
                "roi_gray": roi_gray,
                "frame_w": frame_w,
                "ts": time()
            })
            sleep(dt)
    finally:
        cam.close()


# ============================================================
# INTERPRETER (consumer–producer)
# Reads ROI gray; outputs error_px (or None if lost)
# Publishes: {"error_px": int|None, "ok": bool, "white_px": int, "ts": t}
# ============================================================

class CameraInterpreter:
    def __init__(self):
        pass

    def interpret(self, roi_gray, frame_w):
        # Threshold (black line on light floor)
        _, binary = cv2.threshold(
            roi_gray,
            THRESH_VAL,
            255,
            cv2.THRESH_BINARY_INV
        )

        white_px = cv2.countNonZero(binary)
        if white_px < MIN_WHITE_PX:
            return {"ok": False, "error_px": None, "white_px": int(white_px), "reason": "too_few_white"}

        moments = cv2.moments(binary)
        if moments["m00"] == 0:
            return {"ok": False, "error_px": None, "white_px": int(white_px), "reason": "zero_moment"}

        cx = moments["m10"] / moments["m00"]
        center_x = frame_w / 2.0
        error_px = cx - center_x

        return {"ok": True, "error_px": int(error_px), "white_px": int(white_px), "reason": "ok"}


def interpreter_task(sensor_bus, interp_bus, shutdown_event, dt):
    interp = CameraInterpreter()

    while not shutdown_event.is_set():
        msg = sensor_bus.read()
        if msg is None:
            sleep(dt)
            continue

        if not msg.get("valid", False):
            interp_bus.write({
                "ok": False,
                "error_px": None,
                "white_px": 0,
                "reason": msg.get("reason", "invalid_sensor"),
                "ts": time()
            })
            sleep(dt)
            continue

        roi_gray = msg["roi_gray"]
        frame_w  = msg["frame_w"]

        out = interp.interpret(roi_gray, frame_w)
        out["ts"] = time()
        interp_bus.write(out)

        sleep(dt)


# ============================================================
# CONTROLLER (consumer)
# Reads interpreted error; drives steering + forward
# ============================================================

def control_task(interp_bus, shutdown_event, dt):
    px = Picarx()

    # --- camera tilt (your original behavior) ---
    print("[INIT] setting camera tilt to -45 deg")
    px.set_cam_tilt_angle(-45)
    sleep(0.4)

    try:
        while not shutdown_event.is_set():
            msg = interp_bus.read()
            if msg is None:
                sleep(dt)
                continue

            if not msg.get("ok", False) or msg.get("error_px", None) is None:
                # FAIL-SAFE: stop if vision is lost
                px.stop()
                sleep(dt)
                continue

            error_px = msg["error_px"]

            steer = STEER_GAIN * error_px
            steer = max(-MAX_STEER_DEG, min(MAX_STEER_DEG, steer))

            px.set_dir_servo_angle(steer)
            px.forward(FORWARD_SPEED)

            sleep(dt)

    finally:
        px.stop()


# ============================================================
# MAIN (ThreadPoolExecutor + graceful shutdown + exception surfacing)
# ============================================================

def handle_exception(future):
    e = future.exception()
    if e:
        print(f"Exception in worker thread: {e}")


if __name__ == "__main__":
    print("[MAIN] PiCar-X camera line follower (Simultaneity + locked busses)")

    shutdown_event = Event()

    sensor_bus = Bus(initial=None)   # camera ROI snapshots
    interp_bus = Bus(initial=None)   # error_px + status

    futures = []
    with ThreadPoolExecutor(max_workers=3) as executor:
        futures.append(executor.submit(sensor_task, sensor_bus, shutdown_event, SENSOR_DT))
        futures.append(executor.submit(interpreter_task, sensor_bus, interp_bus, shutdown_event, INTERPRETER_DT))
        futures.append(executor.submit(control_task, interp_bus, shutdown_event, CONTROL_DT))

        for f in futures:
            f.add_done_callback(handle_exception)

        try:
            while not shutdown_event.is_set():
                sleep(0.5)
        except KeyboardInterrupt:
            print("Shutting down")
            shutdown_event.set()
        finally:
            executor.shutdown(wait=True)
