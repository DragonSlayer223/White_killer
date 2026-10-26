import html
import math
import queue
import re
import time
import threading
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

import cv2
import numpy as np

from BallBalancerPID import BallBalancerPID
from plate_calibration import CalibrationCancelled, run_interactive_calibration

STREAM_URL = "http://10.98.211.17:4747/video"
CONNECT_TIMEOUT_SECONDS = 4
RETRY_DELAY_SECONDS = 2
MAX_JPEG_BYTES = 8 * 1024 * 1024
MIN_CONTOUR_AREA = 30
MIN_BALL_RADIUS = 4
DISPLAY_SMOOTHING_ALPHA = 0.2


class StreamError(Exception):
    pass


def _trackbar_changed(position):
    _ = position


def iter_mjpeg_frames(response):
    buffer = bytearray()
    while True:
        chunk = response.read(16 * 1024)
        if not chunk:
            return
        buffer.extend(chunk)

        while True:
            start = buffer.find(b"\xff\xd8")
            if start < 0:
                buffer[:] = buffer[-1:] if buffer[-1:] == b"\xff" else b""
                break

            end = buffer.find(b"\xff\xd9", start + 2)
            if end < 0:
                if start:
                    del buffer[:start]
                if len(buffer) > MAX_JPEG_BYTES:
                    raise StreamError("JPEG frame exceeded the 8 MiB safety limit.")
                break

            encoded = bytes(buffer[start : end + 2])
            del buffer[: end + 2]
            frame = cv2.imdecode(np.frombuffer(encoded, dtype=np.uint8), cv2.IMREAD_COLOR)
            if frame is not None:
                yield frame


def describe_html_response(body):
    page = html.unescape(body.decode("utf-8", errors="replace"))
    message = " ".join(re.sub(r"<[^>]+>", " ", page).split())
    if "droidcam is busy" in message.lower():
        return "DroidCam reports Busy instead of returning a video stream."
    return f"Expected MJPEG but received HTML: {message[:160]}"


def set_status(status_queue, message):
    try:
        status_queue.put_nowait(message)
    except queue.Full:
        try:
            status_queue.get_nowait()
        except queue.Empty:
            pass
        status_queue.put_nowait(message)


def stream_worker(stop_event, frame_queue, status_queue):
    request = Request(
        STREAM_URL,
        headers={"Accept": "multipart/x-mixed-replace, image/jpeg"},
    )
    while not stop_event.is_set():
        try:
            set_status(status_queue, f"Connecting to {STREAM_URL}")
            with urlopen(request, timeout=CONNECT_TIMEOUT_SECONDS) as response:
                if response.headers.get_content_type() == "text/html":
                    raise StreamError(describe_html_response(response.read(8192)))

                for frame in iter_mjpeg_frames(response):
                    if stop_event.is_set():
                        return
                    try:
                        frame_queue.put_nowait(frame)
                    except queue.Full:
                        try:
                            frame_queue.get_nowait()
                        except queue.Empty:
                            pass
                        frame_queue.put_nowait(frame)
                    set_status(status_queue, f"Live MJPEG: {STREAM_URL}")

            if not stop_event.is_set():
                raise StreamError("The video stream ended.")
        except (HTTPError, URLError, TimeoutError, OSError, StreamError) as error:
            if stop_event.is_set():
                return
            message = f"Stream error: {error}"
            print(message, flush=True)
            set_status(status_queue, message)
            stop_event.wait(RETRY_DELAY_SECONDS)


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


def main():
    stop_event = threading.Event()
    frame_queue = queue.Queue(maxsize=1)
    status_queue = queue.Queue(maxsize=1)
    reader = threading.Thread(
        target=stream_worker,
        args=(stop_event, frame_queue, status_queue),
        name="mjpeg-reader",
        daemon=True,
    )

    create_hsv_controls()
    cv2.namedWindow("Ping Pong Tracking", cv2.WINDOW_NORMAL)
    cv2.namedWindow("Orange Mask", cv2.WINDOW_NORMAL)
    reader.start()

    print(
        f"Reading phone MJPEG stream at {STREAM_URL}. "
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
    main()
