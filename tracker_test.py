import html
import queue
import re
import time
import threading
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

import cv2
import numpy as np

STREAM_URL = "http://10.98.211.17:4747/video"
CONNECT_TIMEOUT_SECONDS = 4
RETRY_DELAY_SECONDS = 2
MAX_JPEG_BYTES = 8 * 1024 * 1024
MIN_CONTOUR_AREA = 30
MIN_BALL_RADIUS = 4


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
    cv2.resizeWindow("HSV Tuning", 420, 250)
    cv2.createTrackbar("H min", "HSV Tuning", 5, 179, _trackbar_changed)
    cv2.createTrackbar("H max", "HSV Tuning", 25, 179, _trackbar_changed)
    cv2.createTrackbar("S min", "HSV Tuning", 100, 255, _trackbar_changed)
    cv2.createTrackbar("S max", "HSV Tuning", 255, 255, _trackbar_changed)
    cv2.createTrackbar("V min", "HSV Tuning", 80, 255, _trackbar_changed)
    cv2.createTrackbar("V max", "HSV Tuning", 255, 255, _trackbar_changed)


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

    status = "Connecting to phone video stream..."
    frame = None
    fps = 0.0
    previous_time = time.perf_counter()
    print(
        f"Reading phone MJPEG stream at {STREAM_URL}. "
        "Adjust HSV sliders; press q or Esc to quit.",
        flush=True,
    )

    try:
        while True:
            while True:
                try:
                    status = status_queue.get_nowait()
                except queue.Empty:
                    break
            try:
                frame = frame_queue.get_nowait()
            except queue.Empty:
                pass

            if frame is None:
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
                cv2.imshow("Ping Pong Tracking", display)
            else:
                lower_hsv, upper_hsv = get_hsv_bounds()
                mask, detection = find_orange_ball(frame, lower_hsv, upper_hsv)
                display = frame.copy()

                if detection is not None:
                    area, x, y, radius = detection
                    center = (int(x), int(y))
                    cv2.circle(display, center, int(radius), (0, 255, 0), 2)
                    cv2.circle(display, center, 3, (0, 0, 255), -1)
                    cv2.putText(
                        display,
                        f"Ball: x={center[0]} y={center[1]} area={area:.0f}",
                        (10, 28),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.65,
                        (0, 255, 0),
                        2,
                        cv2.LINE_AA,
                    )
                else:
                    cv2.putText(
                        display,
                        "No orange ball detected - tune HSV sliders",
                        (10, 28),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.65,
                        (0, 255, 255),
                        2,
                        cv2.LINE_AA,
                    )

                current_time = time.perf_counter()
                elapsed = current_time - previous_time
                previous_time = current_time
                if elapsed > 0:
                    instant_fps = 1.0 / elapsed
                    fps = instant_fps if fps == 0 else 0.9 * fps + 0.1 * instant_fps
                cv2.putText(
                    display,
                    f"{fps:.1f} FPS | {status[:55]}",
                    (10, 55),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (255, 255, 255),
                    1,
                    cv2.LINE_AA,
                )

                cv2.imshow("Ping Pong Tracking", display)
                cv2.imshow("Orange Mask", mask)

            if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                break
    finally:
        stop_event.set()
        reader.join(timeout=CONNECT_TIMEOUT_SECONDS + 1)
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
