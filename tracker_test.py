import argparse
import math
import os
import queue
import time
import threading

import cv2
import numpy as np

# Avoid flooding the terminal with repetitive OpenCV backend internals.
try:
    cv2.utils.logging.setLogLevel(cv2.utils.logging.LOG_LEVEL_ERROR)
except (AttributeError, RuntimeError):
    pass

from BallBalancerPID import BallBalancerPID
from plate_calibration import CalibrationCancelled, run_interactive_calibration

DEFAULT_CAMERA_INDEX = os.environ.get("DROIDCAM_CAMERA_INDEX")
CAPTURE_READ_TIMEOUT_MS = 2500
CONNECT_TIMEOUT_SECONDS = 5
RETRY_DELAY_SECONDS = 2
MIN_CONTOUR_AREA = 30
MIN_BALL_RADIUS = 4
DISPLAY_SMOOTHING_ALPHA = 0.2


class StreamError(Exception):
    pass


def _trackbar_changed(position):
    _ = position


def set_status(status_queue, message):
    try:
        status_queue.put_nowait(message)
    except queue.Full:
        try:
            status_queue.get_nowait()
        except queue.Empty:
            pass
        status_queue.put_nowait(message)


def discover_droidcam_index():
    try:
        from pygrabber.dshow_graph import FilterGraph
        devices = FilterGraph().get_input_devices()
    except Exception as error:
        print(f"Could not enumerate DirectShow cameras: {error}", flush=True)
        return None

    print(f"DirectShow cameras: {devices}", flush=True)
    for index, name in enumerate(devices):
        if "droidcam" in name.lower():
            print(f"Selected DroidCam device at DirectShow index {index}: {name}", flush=True)
            return index
    return None


def fourcc_name(value):
    value = int(value) & 0xFFFFFFFF
    chars = []
    for shift in range(0, 32, 8):
        char = chr((value >> shift) & 0xFF)
        chars.append(char if char.isascii() and char.isprintable() else ".")
    return "".join(chars)


def open_usb_camera(index):
    # Re-opening and changing formats on one capture can corrupt the Media
    # Foundation stream. DroidCam's known good USB format is established first,
    # and each retry gets a fresh capture object.
    cv2.setNumThreads(4)
    # Try DirectShow first because changing Media Foundation stream state can
    # leave DroidCam alternating between real frames and invalid frames.
    for backend in (cv2.CAP_MSMF, cv2.CAP_DSHOW):
        capture = cv2.VideoCapture(index, backend)
        if not capture.isOpened():
            capture.release()
            continue

        # DroidCam's Media Foundation stream is stable when OpenCV preserves
        # the driver's negotiated format. Re-setting FOURCC, resolution, and FPS
        # here can produce valid startup frames followed by black frames.
        capture.set(cv2.CAP_PROP_CONVERT_RGB, 1)
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        valid_samples = 0
        started = time.perf_counter()
        try:
            for _ in range(60):
                ok, frame = capture.read()
                if ok and is_valid_video_frame(frame):
                    valid_samples += 1
                if valid_samples >= 1:
                    break
        except cv2.error as error:
            print(f"Camera {index} backend {backend} read error: {error}", flush=True)
            capture.release()
            continue

        elapsed = max(time.perf_counter() - started, 1e-3)
        if valid_samples >= 1:
            measured_fps = valid_samples / elapsed
            actual_width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
            actual_fps = float(capture.get(cv2.CAP_PROP_FPS))
            fourcc = fourcc_name(capture.get(cv2.CAP_PROP_FOURCC))
            print(
                f"Camera {index}, backend {backend}: {fourcc} "
                f"{actual_width}x{actual_height}, requested 30 FPS, "
                f"reported {actual_fps:.1f}, sampled {measured_fps:.1f} FPS",
                flush=True,
            )
            return capture
        capture.release()
    return None


def is_valid_video_frame(frame):
    if frame is None or frame.size == 0:
        return False
    if len(frame.shape) != 3 or frame.shape[2] < 3:
        return False
    if float(frame.std()) < 8.0:
        return False
    # Do not run np.unique over every full-resolution frame; that operation can
    # turn an otherwise healthy 30 FPS camera into a multi-second bottleneck.
    sample = frame[::8, ::8]
    unique_colors = np.unique(sample.reshape(-1, 3), axis=0).shape[0]
    return unique_colors >= 12


def read_valid_frame(capture):
    try:
        ok, frame = capture.read()
    except (cv2.error, RuntimeError, ValueError):
        return None
    if not ok or frame is None or frame.size == 0:
        return None
    return frame if is_valid_video_frame(frame) else None


def stream_worker(stop_event, frame_queue, status_queue, preferred_index):
    while not stop_event.is_set():
        capture = None
        active_index = None
        candidates = [] if preferred_index is None else [preferred_index]

        if not candidates:
            message = "DroidCam Video was not found. Refusing to open another camera."
            print(message, flush=True)
            set_status(status_queue, message)
            stop_event.wait(RETRY_DELAY_SECONDS)
            continue

        set_status(status_queue, "Opening named DroidCam Video device...")
        for index in candidates:
            candidate = open_usb_camera(index)
            if candidate is None:
                continue

            # Drain startup frames, then require a real image. Motion is deliberately
            # not required: a stationary plate or test pattern is a valid camera frame.
            frame_received = False
            for _ in range(3):
                if read_valid_frame(candidate) is not None:
                    frame_received = True
                    break
            if frame_received:
                capture = candidate
                active_index = index
                break
            candidate.release()

        if capture is None:
            message = (
                "No active USB camera found. Start DroidCam Desktop, connect the "
                "phone by USB, enable USB mode, then retry."
            )
            print(message, flush=True)
            set_status(status_queue, message)
            stop_event.wait(RETRY_DELAY_SECONDS)
            continue

        print(f"Connected to DroidCam USB index {active_index}.", flush=True)
        set_status(status_queue, f"DroidCam USB camera {active_index}")

        try:
            unusable_frames = 0
            while not stop_event.is_set():
                frame = read_valid_frame(capture)
                if frame is None:
                    unusable_frames += 1
                    if unusable_frames < 30:
                        continue
                    print("USB stream became unusable; reopening camera connection.", flush=True)
                    break
                unusable_frames = 0

                try:
                    frame_queue.put_nowait(frame)
                except queue.Full:
                    try:
                        frame_queue.get_nowait()
                    except queue.Empty:
                        pass
                    frame_queue.put_nowait(frame)
                set_status(status_queue, f"Live USB camera {active_index}")
        except StreamError as error:
            print(f"USB camera error: {error}", flush=True)
            set_status(status_queue, f"USB camera error: {error}")
        finally:
            capture.release()
            stop_event.wait(RETRY_DELAY_SECONDS) if not stop_event.is_set() else None


def create_hsv_controls():
    cv2.namedWindow("HSV Tuning", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("HSV Tuning", 420, 380)
    cv2.createTrackbar("H min", "HSV Tuning", 5, 179, _trackbar_changed)
    cv2.createTrackbar("H max", "HSV Tuning", 25, 179, _trackbar_changed)
    cv2.createTrackbar("S min", "HSV Tuning", 100, 255, _trackbar_changed)
    cv2.createTrackbar("S max", "HSV Tuning", 255, 255, _trackbar_changed)
    cv2.createTrackbar("V min", "HSV Tuning", 80, 255, _trackbar_changed)
    cv2.createTrackbar("V max", "HSV Tuning", 255, 255, _trackbar_changed)
    cv2.createTrackbar("Red H low max", "HSV Tuning", 10, 20, _trackbar_changed)
    cv2.createTrackbar("Red H high min", "HSV Tuning", 170, 179, _trackbar_changed)
    cv2.createTrackbar("Red S min", "HSV Tuning", 100, 255, _trackbar_changed)
    cv2.createTrackbar("Red V min", "HSV Tuning", 50, 255, _trackbar_changed)


def get_hsv_bounds():
    lower = np.array(
        [
            cv2.getTrackbarPos("H min", "HSV Tuning"),
            cv2.getTrackbarPos("S min", "HSV Tuning"),
            cv2.getTrackbarPos("V min", "HSV Tuning"),
        ],
        dtype=np.uint8,
    )
    upper = np.array(
        [
            cv2.getTrackbarPos("H max", "HSV Tuning"),
            cv2.getTrackbarPos("S max", "HSV Tuning"),
            cv2.getTrackbarPos("V max", "HSV Tuning"),
        ],
        dtype=np.uint8,
    )
    return lower, upper


def get_red_bounds():
    return (
        cv2.getTrackbarPos("Red H low max", "HSV Tuning"),
        cv2.getTrackbarPos("Red H high min", "HSV Tuning"),
        cv2.getTrackbarPos("Red S min", "HSV Tuning"),
        cv2.getTrackbarPos("Red V min", "HSV Tuning"),
    )


def find_orange_ball(frame, lower_hsv, upper_hsv):
    blurred = cv2.GaussianBlur(frame, (5, 5), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_hsv, upper_hsv)

    open_kernel = np.ones((3, 3), dtype=np.uint8)
    close_kernel = np.ones((5, 5), dtype=np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, open_kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    candidates = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < MIN_CONTOUR_AREA:
            continue

        (x, y), radius = cv2.minEnclosingCircle(contour)
        if radius < MIN_BALL_RADIUS:
            continue

        circularity = area / (np.pi * radius * radius)
        if circularity >= 0.45:
            candidates.append((area, x, y, radius))

    detection = max(candidates, default=None, key=lambda candidate: candidate[0])
    return mask, detection


def calculate_desired_orientation(
    pid, center_x, center_y, ball_x, ball_y, scale_px_to_cm, zero_azimuth_deg, dt
):
    """Return bounded pitch/roll tilt, magnitude, and azimuth clockwise from pitch."""
    current_x = (ball_x - center_x) * scale_px_to_cm
    current_y = (ball_y - center_y) * scale_px_to_cm
    pitch, roll = pid.compute(0.0, 0.0, current_x, current_y, dt)

    tilt_deg = math.hypot(pitch, roll)
    if tilt_deg > pid.max_tilt_deg:
        factor = pid.max_tilt_deg / tilt_deg
        pitch *= factor
        roll *= factor
        tilt_deg = pid.max_tilt_deg

    azimuth_deg = (
        math.degrees(math.atan2(roll, pitch)) + zero_azimuth_deg
    ) % 360.0
    return pitch, roll, tilt_deg, azimuth_deg


def normalized_center_distance(center_x, center_y, ball_x, ball_y, plate_radius_px):
    if plate_radius_px <= 0:
        raise ValueError("Plate radius from center to red tape must be positive.")
    distance_px = math.hypot(ball_x - center_x, ball_y - center_y)
    return min(1.0, distance_px / plate_radius_px)


def smooth_angle(previous_deg, current_deg, alpha):
    if previous_deg is None:
        return current_deg % 360.0
    delta = (current_deg - previous_deg + 180.0) % 360.0 - 180.0
    return (previous_deg + alpha * delta) % 360.0


def run_camera_check(camera_index):
    """Capture a named DroidCam device and save one frame for user verification."""
    capture = None
    for backend_name, backend in (("Media Foundation", cv2.CAP_MSMF), ("DirectShow", cv2.CAP_DSHOW)):
        candidate = cv2.VideoCapture(camera_index, backend)
        if candidate.isOpened():
            capture = candidate
            print(f"Testing named DroidCam Video through {backend_name}...", flush=True)
            break
        candidate.release()
    if capture is None:
        raise SystemExit("DroidCam Video could not be opened; refusing to try another camera.")

    capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    frame_count = 0
    started = time.perf_counter()
    preview = None
    deadline = started + 10.0

    while time.perf_counter() < deadline and frame_count < 120:
        frame = read_valid_frame(capture)
        if frame is None:
            time.sleep(0.01)
            continue
        preview = frame
        frame_count += 1

    capture.release()
    elapsed = max(time.perf_counter() - started, 1e-3)
    if preview is None:
        raise SystemExit("DroidCam Video did not return a valid frame.")

    preview_path = "droidcam_preview.jpg"
    if not cv2.imwrite(preview_path, preview):
        raise SystemExit(f"Could not write preview image: {os.path.abspath(preview_path)}")
    print(f"Valid frames: {frame_count} in {elapsed:.2f}s ({frame_count / elapsed:.1f} FPS)")
    print(f"Saved verified DroidCam preview: {preview_path}", flush=True)


def main(camera_index=None):
    stop_event = threading.Event()
    if camera_index is None:
        camera_index = discover_droidcam_index()
        if camera_index is None:
            raise SystemExit(
                "DroidCam Video was not found. Refusing to open a webcam or "
                "any unnamed camera. Start DroidCam Desktop and reconnect USB."
            )
    frame_queue = queue.Queue(maxsize=1)
    status_queue = queue.Queue(maxsize=1)
    reader = threading.Thread(
        target=stream_worker,
        args=(stop_event, frame_queue, status_queue, camera_index),
        name="usb-camera-reader",
        daemon=True,
    )

    create_hsv_controls()
    cv2.namedWindow("Ping Pong Tracking", cv2.WINDOW_NORMAL)
    cv2.namedWindow("Orange Mask", cv2.WINDOW_NORMAL)
    reader.start()

    print(
        f"Reading DroidCam through USB using Windows camera index {camera_index}. "
        "Place the ball at the plate center and press SPACE to calibrate; "
        "press q or Esc to quit.",
        flush=True,
    )

    try:
        (
            center_x,
            center_y,
            scale_px_to_cm,
            zero_azimuth_deg,
            plate_radius_px,
        ) = (
            run_interactive_calibration(
                frame_queue,
                get_hsv_bounds,
                get_red_bounds,
                find_orange_ball,
                stop_event,
            )
        )
        pid = BallBalancerPID()
        print(
            f"Calibration complete: center=({center_x:.1f}, {center_y:.1f}), "
            f"scale={scale_px_to_cm:.3f} cm/pixel, "
            f"plate radius={plate_radius_px:.1f} px, "
            f"zero azimuth={zero_azimuth_deg:.1f} degrees.",
            flush=True,
        )

        status = "Calibration complete"
        current_frame = None
        current_mask = None
        display = None
        tracking = False
        previous_detection_time = None
        smoothed_tilt_deg = None
        smoothed_azimuth_deg = None
        fps = 0.0
        previous_frame_time = time.perf_counter()

        while True:
            while True:
                try:
                    status = status_queue.get_nowait()
                except queue.Empty:
                    break

            new_frame = False
            try:
                current_frame = frame_queue.get_nowait()
                new_frame = True
            except queue.Empty:
                pass

            if new_frame:
                lower_hsv, upper_hsv = get_hsv_bounds()
                current_mask, detection = find_orange_ball(
                    current_frame, lower_hsv, upper_hsv
                )
                display = current_frame.copy()
                now = time.perf_counter()
                elapsed = now - previous_frame_time
                previous_frame_time = now
                if elapsed > 0:
                    instant_fps = 1.0 / elapsed
                    fps = instant_fps if fps == 0 else 0.9 * fps + 0.1 * instant_fps

                if detection is not None:
                    area, x, y, radius = detection
                    center = (int(x), int(y))
                    cv2.drawMarker(
                        display,
                        (int(center_x), int(center_y)),
                        (255, 0, 0),
                        cv2.MARKER_CROSS,
                        18,
                        2,
                    )
                    cv2.circle(display, center, int(radius), (0, 255, 0), 2)
                    cv2.circle(display, center, 3, (0, 0, 255), -1)

                    if not tracking:
                        pid.reset(
                            0.0,
                            0.0,
                            (x - center_x) * scale_px_to_cm,
                            (y - center_y) * scale_px_to_cm,
                        )
                        tracking = True
                    dt = (
                        1.0 / max(fps, 1.0)
                        if previous_detection_time is None
                        else max(now - previous_detection_time, 1e-3)
                    )
                    previous_detection_time = now
                    pitch, roll, tilt_deg, azimuth_deg = calculate_desired_orientation(
                        pid,
                        center_x,
                        center_y,
                        x,
                        y,
                        scale_px_to_cm,
                        zero_azimuth_deg,
                        dt,
                    )
                    if smoothed_tilt_deg is None:
                        smoothed_tilt_deg = tilt_deg
                    else:
                        smoothed_tilt_deg += DISPLAY_SMOOTHING_ALPHA * (
                            tilt_deg - smoothed_tilt_deg
                        )
                    smoothed_azimuth_deg = smooth_angle(
                        smoothed_azimuth_deg,
                        azimuth_deg,
                        DISPLAY_SMOOTHING_ALPHA,
                    )
                    center_distance = normalized_center_distance(
                        center_x,
                        center_y,
                        x,
                        y,
                        plate_radius_px,
                    )
                    cv2.putText(
                        display,
                        f"Ball: x={center[0]} y={center[1]} area={area:.0f}",
                        (10, 28),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 255, 0),
                        2,
                        cv2.LINE_AA,
                    )
                    cv2.putText(
                        display,
                        f"Center distance={center_distance:.2f} (0=center, 1=edge)",
                        (10, 55),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (255, 255, 255),
                        2,
                        cv2.LINE_AA,
                    )
                    cv2.putText(
                        display,
                        f"Smoothed tilt={smoothed_tilt_deg:.2f} deg  "
                        f"azimuth={smoothed_azimuth_deg:.1f} deg",
                        (10, 82),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (255, 255, 255),
                        2,
                        cv2.LINE_AA,
                    )
                    cv2.putText(
                        display,
                        f"Pitch={pitch:.2f} deg  Roll={roll:.2f} deg",
                        (10, 109),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (255, 255, 255),
                        2,
                        cv2.LINE_AA,
                    )
                else:
                    tracking = False
                    previous_detection_time = None
                    smoothed_tilt_deg = None
                    smoothed_azimuth_deg = None
                    cv2.putText(
                        display,
                        "Ball not detected - tune HSV sliders",
                        (10, 28),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.65,
                        (0, 255, 255),
                        2,
                        cv2.LINE_AA,
                    )
            if display is None:
                display = np.zeros((360, 640, 3), dtype=np.uint8)
                cv2.putText(
                    display,
                    status[:80],
                    (15, 180),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (255, 255, 255),
                    1,
                    cv2.LINE_AA,
                )
            elif new_frame:
                cv2.putText(
                    display,
                    f"{fps:.1f} FPS | {status[:55]}",
                    (10, 136 if tracking else 55),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (255, 255, 255),
                    1,
                    cv2.LINE_AA,
                )

            if current_mask is not None:
                cv2.imshow("Ping Pong Tracking", display)
                cv2.imshow("Orange Mask", current_mask)
            else:
                cv2.imshow("Ping Pong Tracking", display)

            if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                break
    except CalibrationCancelled as error:
        print(error, flush=True)
    finally:
        stop_event.set()
        reader.join(timeout=CONNECT_TIMEOUT_SECONDS + 1)
        cv2.destroyAllWindows()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="USB DroidCam ball tracker")
    parser.add_argument(
        "--camera-index",
        type=int,
        default=int(DEFAULT_CAMERA_INDEX) if DEFAULT_CAMERA_INDEX else None,
        help="DroidCam Windows DirectShow index (normally auto-detected; override with DROIDCAM_CAMERA_INDEX)",
    )
    parser.add_argument(
        "--camera-check",
        action="store_true",
        help="Capture only DroidCam Video and save droidcam_preview.jpg without starting tracking",
    )
    args = parser.parse_args()
    if args.camera_index is None:
        args.camera_index = discover_droidcam_index()
    if args.camera_index is None:
        raise SystemExit("DroidCam Video was not found; refusing to open any other camera.")
    if args.camera_check:
        run_camera_check(args.camera_index)
    else:
        main(args.camera_index)

