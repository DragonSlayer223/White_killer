
import argparse
import json
import math
import queue
import socket
import threading
import time
from pathlib import Path

import cv2
import numpy as np

# ============================================================
# STEWART PLATFORM - USB PHONE CAMERA BALL TRACKER
# ============================================================
# Camera: DroidCam USB (Windows DirectShow)
# Output: Ball position sent to MATLAB over localhost TCP
#
# Python: Camera -> HSV -> Ball position
# MATLAB: PD control -> Inverse Kinematics -> RUMBA
#
# No IP camera URL. No laptop webcam fallback.
# ============================================================

PORT = 5051
BALL_DIAMETER_CM = 4.0

MIN_AREA = 30
MIN_RADIUS = 4
MIN_CIRCULARITY = 0.45
# Separate circles, both centered on the calibrated center:
# CYAN (reference) = exactly the center-to-red-marker distance.
# MAGENTA (tracking) = larger detection boundary for ball and markers.
REFERENCE_RADIUS_SCALE = 1.00
TRACKING_RADIUS_SCALE = 1.50
PLATE_EDGE_MARGIN_PX = 2.0
if not (1.0 <= TRACKING_RADIUS_SCALE <= 3.0):
    raise ValueError("TRACKING_RADIUS_SCALE must be between 1.0 and 3.0")

SEND_HZ = 30
MAX_FRAME_GAP_S = 0.6
BALL_LOSS_TIMEOUT_S = 0.45

FOLDER = Path(__file__).resolve().parent
CAL_FILE = FOLDER / "calibration_config.json"
HSV_FILE = FOLDER / "usb_tracker_hsv.json"
CENTER_FILE = FOLDER / "platform_center_reference.json"
# Two BLUE markers on the moving plate and one GREEN marker on the base.
# HSV values are adjustable independently while the tracker is running.
# IMPORTANT: Blue AND green markers must lie COMPLETELY within the outer
# MAGENTA tracking circle. Markers outside this region are ignored.
MARKER_HSV_FILE = FOLDER / "usb_marker_hsv_blue_green.json"
DEFAULT_MARKER_HSV = {
    "blue": {"H min": 95, "H max": 130, "S min": 110,
             "S max": 255, "V min": 70, "V max": 255},
    "green": {"H min": 40, "H max": 90, "S min": 95,
               "S max": 255, "V min": 65, "V max": 255},
}
MARKER_MIN_AREA = 14
MARKER_MAX_AREA = 1500
MARKER_MIN_SEPARATION_PX = 30
PLATE_SEND_HZ = 30
# Brief obstructions do not invalidate the last observed platform reference.
# No fake PLATE packets: MATLAB freezes XY correction until markers return.
MARKER_OCCLUSION_GRACE_S = 1.20
# Lock the fixed green reference after three consistent frames.
GREEN_LOCK_FRAMES = 3
GREEN_LOCK_TOLERANCE_PX = 12.0



# ============================================================
# CAMERA DISCOVERY
# ============================================================

def list_cameras():
    from pygrabber.dshow_graph import FilterGraph

    devices = FilterGraph().get_input_devices()

    print("\nAvailable Windows cameras:")

    for i, name in enumerate(devices):
        print(f"[{i}] {name}")

    return devices


def find_phone_camera(devices, selected_index=None):
    matches = [
        i for i, name in enumerate(devices)
        if "droidcam" in name.lower()
    ]

    if not matches:
        raise RuntimeError(
            "DroidCam USB camera not found.\n"
            "Connect your phone through DroidCam Desktop USB mode.\n"
            "Refusing to use the laptop webcam."
        )

    if selected_index is not None:
        if selected_index not in matches:
            raise RuntimeError(
                f"Camera {selected_index} is not DroidCam."
            )
        index = selected_index
    else:
        index = next(
            (
                i for i in matches
                if "video" in devices[i].lower()
            ),
            matches[0]
        )

    print(f"\nSelected PHONE camera: {devices[index]}")
    print(f"Camera index: {index}")

    return index


def valid_frame(frame):
    if frame is None:
        return False

    if frame.size == 0 or frame.ndim != 3:
        return False

    if frame.shape[2] < 3:
        return False

    sample = frame[::8, ::8]

    return (
        sample.std() >= 8
        and len(np.unique(
            sample.reshape(-1, 3), axis=0
        )) >= 2
    )


# ============================================================
# CAMERA READER THREAD
# ============================================================


def camera_worker(index, frame_queue, stop_event):
    import comtypes
    from pygrabber.dshow_graph import FilterGraph

    comtypes.CoInitialize()

    try:
        while not stop_event.is_set():
            graph = None

            try:
                graph = FilterGraph()
                devices = graph.get_input_devices()

                if index >= len(devices) or "droidcam" not in devices[index].lower():
                    raise RuntimeError(
                        "Selected camera is not DroidCam. "
                        "Refusing to use laptop webcam."
                    )

                print(f"Opening PHONE camera: {devices[index]}")

                def on_frame(frame):
                    if not valid_frame(frame):
                        return

                    # Keep newest frame only
                    while not frame_queue.empty():
                        try:
                            frame_queue.get_nowait()
                        except queue.Empty:
                            break

                    try:
                        frame_queue.put_nowait(
                            (time.monotonic(), frame)
                        )
                    except queue.Full:
                        pass

                # Use DirectShow directly, not OpenCV VideoCapture.
                graph.add_video_input_device(index)
                graph.add_sample_grabber(on_frame)
                graph.add_null_render()
                graph.prepare_preview_graph()
                graph.run()

                print("DroidCam DirectShow capture started.")

                while not stop_event.is_set():
                    graph.grab_frame()
                    stop_event.wait(1 / 30)

            except Exception as error:
                print(f"DroidCam capture error: {error}")

            finally:
                if graph is not None:
                    try:
                        graph.stop()
                    except Exception:
                        pass

            if not stop_event.is_set():
                print("Retrying DroidCam connection...")
                stop_event.wait(1.0)

    finally:
        comtypes.CoUninitialize()



# ============================================================
# HSV BALL DETECTION
# ============================================================

HSV_SLIDER_LIMITS = {
    "H min": 179, "H max": 179,
    "S min": 255, "S max": 255,
    "V min": 255, "V max": 255,
}


def create_hsv_controls(settings, window="HSV Tuning"):
    """The same six OpenCV sliders for orange, blue, or green."""
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window, 450, 300)
    for name, maximum in HSV_SLIDER_LIMITS.items():
        cv2.createTrackbar(name, window, int(settings[name]),
                           maximum, lambda value: None)


def get_hsv_settings(window="HSV Tuning"):
    return {name: cv2.getTrackbarPos(name, window)
            for name in HSV_SLIDER_LIMITS}


def marker_hsv_bounds(settings):
    """Convert slider values to OpenCV lower/upper HSV triplets."""
    lower = (settings["H min"], settings["S min"], settings["V min"])
    upper = (settings["H max"], settings["S max"], settings["V max"])
    return lower, upper


def load_marker_hsv(path=MARKER_HSV_FILE):
    """Use persisted slider settings, with safe defaults for missing keys."""
    stored = load_json(path) or {}
    result = {}
    for color in ("blue", "green"):
        saved = stored.get(color, {}) if isinstance(stored, dict) else {}
        if not isinstance(saved, dict):
            saved = {}
        result[color] = {}
        for name, maximum in HSV_SLIDER_LIMITS.items():
            value = saved.get(name, DEFAULT_MARKER_HSV[color][name])
            try:
                value = int(value)
            except (TypeError, ValueError):
                value = DEFAULT_MARKER_HSV[color][name]
            result[color][name] = max(0, min(maximum, value))
    return result


def reference_radius_px(calibration):
    """Cyan reference radius: calibrated center to RED reference marker."""
    cx = float(calibration["center_x"])
    cy = float(calibration["center_y"])
    if "red_marker_x" in calibration and "red_marker_y" in calibration:
        return math.hypot(float(calibration["red_marker_x"]) - cx,
                          float(calibration["red_marker_y"]) - cy) * REFERENCE_RADIUS_SCALE
    return float(calibration["plate_radius_px"]) * REFERENCE_RADIUS_SCALE


def effective_plate_radius_px(calibration):
    """OUTER tracking radius used by orange ball and blue/green markers."""
    return reference_radius_px(calibration) * (TRACKING_RADIUS_SCALE / REFERENCE_RADIUS_SCALE)


def detect_ball(frame, settings, calibration=None):
    """Find the orange ball ONLY within the outer magenta tracking circle.

    During initial calibration (calibration=None), search the whole frame so
    the user can place the ball at the center before clicking the red marker.
    Once calibrated, ignore every contour outside the plate; no 15% overscan.
    """
    blurred = cv2.GaussianBlur(frame, (5, 5), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    lower = np.array([
        settings["H min"], settings["S min"], settings["V min"]
    ], dtype=np.uint8)
    upper = np.array([
        settings["H max"], settings["S max"], settings["V max"]
    ], dtype=np.uint8)

    mask = cv2.inRange(hsv, lower, upper)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,
                            np.ones((3, 3), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE,
                            np.ones((5, 5), np.uint8))

    # Analyze the ORIGINAL full contours, rather than clipping contours at
    # the plate edge; clipping could make an outside ball appear to be inside.
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)

    if calibration is not None:
        cx = float(calibration["center_x"])
        cy = float(calibration["center_y"])
        plate_radius = effective_plate_radius_px(calibration)
        valid_radius = plate_radius - PLATE_EDGE_MARGIN_PX
        if not (math.isfinite(valid_radius) and valid_radius > MIN_RADIUS):
            return None, np.zeros(mask.shape, dtype=np.uint8)

        plate_mask = np.zeros(mask.shape, dtype=np.uint8)
        cv2.circle(plate_mask, (round(cx), round(cy)),
                   max(1, int(valid_radius)), 255, -1)
        display_mask = cv2.bitwise_and(mask, plate_mask)
    else:
        cx = cy = valid_radius = None
        display_mask = mask

    candidates = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < MIN_AREA:
            continue
        (x, y), radius = cv2.minEnclosingCircle(contour)
        if radius < MIN_RADIUS:
            continue
        circularity = area / max(math.pi * radius * radius, 1)
        if circularity < MIN_CIRCULARITY:
            continue

        if calibration is not None:
            # Require the full detected ball to lie inside the tracking circle.
            if math.hypot(x-cx, y-cy) + radius > valid_radius:
                continue

        candidates.append((area, x, y, radius))

    return max(candidates, default=None), display_mask


def find_color_markers(frame, lower, upper, calibration=None, return_mask=False):
    """Return (area,x,y) only for complete colored blobs INSIDE magenta tracking circle.

    No calibration means no valid magenta tracking circle: never report plate/base markers.
    Keep complete contours intact, then test their enclosing circle so a
    partially visible outside blob cannot masquerade as an inside marker.
    """
    if calibration is None:
        blank = np.zeros(frame.shape[:2], dtype=np.uint8)
        return ([], blank) if return_mask else []

    cx = float(calibration["center_x"])
    cy = float(calibration["center_y"])
    allowed_r = effective_plate_radius_px(calibration) - PLATE_EDGE_MARGIN_PX
    if not (math.isfinite(allowed_r) and allowed_r > 0):
        blank = np.zeros(frame.shape[:2], dtype=np.uint8)
        return ([], blank) if return_mask else []

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(lower, dtype=np.uint8),
                      np.array(upper, dtype=np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,
                            np.ones((3, 3), dtype=np.uint8))
    # Preview: binary HSV result clipped to the magenta tracking circle.
    # White = HSV candidate pixel; accepted markers still have to satisfy
    # the original area and fully-inside-circle contour checks below.
    if return_mask:
        circle = np.zeros(mask.shape, dtype=np.uint8)
        cv2.circle(circle, (round(cx), round(cy)),
                   max(0, int(allowed_r)), 255, -1)
        preview_mask = cv2.bitwise_and(mask, circle)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    points = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if not MARKER_MIN_AREA <= area <= MARKER_MAX_AREA:
            continue
        moments = cv2.moments(contour)
        if moments["m00"] <= 0:
            continue
        x = moments["m10"] / moments["m00"]
        y = moments["m01"] / moments["m00"]
        (_, _), enclosing_r = cv2.minEnclosingCircle(contour)
        if math.hypot(x-cx, y-cy) + enclosing_r > allowed_r:
            continue
        points.append((area, x, y))
    points.sort(reverse=True)
    return (points, preview_mask) if return_mask else points


def marker_occlusion_expired(last_good_time, now):
    """True once the last COMPLETE three-marker fix is too old."""
    return now - last_good_time >= MARKER_OCCLUSION_GRACE_S


def detect_platform_markers(frame, calibration=None,
                            blue_settings=None, green_settings=None,
                            return_details=False, return_masks=False,
                            locked_base_xy=None):
    """Detect two BLUE plate markers against a fixed GREEN base reference.

    Default return is the original ((plate_x,plate_y),(base_x,base_y)) or None.
    Optional return_details gives the valid in-circle dots for drawing.
    Optional return_masks additionally gives blue/green binary preview images.
    Once locked_base_xy is set, live green visibility is no longer required.
    Missing blue markers still invalidate the plate position.
    """
    if blue_settings is None:
        blue_settings = DEFAULT_MARKER_HSV["blue"]
    if green_settings is None:
        green_settings = DEFAULT_MARKER_HSV["green"]
    if return_masks:
        plate, blue_mask = find_color_markers(
            frame, *marker_hsv_bounds(blue_settings),
            calibration=calibration, return_mask=True)
        base, green_mask = find_color_markers(
            frame, *marker_hsv_bounds(green_settings),
            calibration=calibration, return_mask=True)
    else:
        plate = find_color_markers(frame, *marker_hsv_bounds(blue_settings),
                                   calibration=calibration)
        base = find_color_markers(frame, *marker_hsv_bounds(green_settings),
                                  calibration=calibration)
    detection = None
    base_xy = locked_base_xy
    if base_xy is None and len(base) == 1:
        base_xy = (base[0][1], base[0][2])
    if len(plate) == 2 and base_xy is not None:
        separation = math.hypot(plate[0][1]-plate[1][1],
                                plate[0][2]-plate[1][2])
        if separation >= MARKER_MIN_SEPARATION_PX:
            detection = (((plate[0][1]+plate[1][1])/2,
                          (plate[0][2]+plate[1][2])/2),
                         (float(base_xy[0]), float(base_xy[1])))
    if return_masks:
        return detection, plate, base, blue_mask, green_mask
    if return_details:
        return detection, plate, base
    return detection


# ============================================================
# CALIBRATION STORAGE
# ============================================================

def load_json(path):
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None


def save_json(path, data):
    path.write_text(
        json.dumps(data, indent=2),
        encoding="utf-8"
    )


# ============================================================
# MAIN TRACKING PROGRAM
# ============================================================

def main():

    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--camera-index",
        type=int,
        default=None
    )

    parser.add_argument(
        "--list-cameras",
        action="store_true"
    )

    parser.add_argument(
        "--camera-check",
        action="store_true"
    )

    parser.add_argument(
        "--port",
        type=int,
        default=PORT
    )

    args = parser.parse_args()

    devices = list_cameras()

    if args.list_cameras:
        return

    index = find_phone_camera(
        devices, args.camera_index
    )

    camera_name = devices[index]

    # Camera thread
    stop_event = threading.Event()
    frame_queue = queue.Queue(maxsize=1)

    reader = threading.Thread(
        target=camera_worker,
        args=(index, frame_queue, stop_event),
        daemon=True
    )

    # HSV defaults for an orange ping pong ball
    hsv_settings = load_json(HSV_FILE) or {
        "H min": 5,
        "H max": 25,
        "S min": 100,
        "S max": 255,
        "V min": 80,
        "V max": 255,
    }

    create_hsv_controls(hsv_settings)

    # Independent live HSV panels for each centering marker color.
    saved_marker_hsv = load_marker_hsv()
    create_hsv_controls(saved_marker_hsv["blue"], "Blue Marker HSV")
    create_hsv_controls(saved_marker_hsv["green"], "Green Marker HSV")

    cv2.namedWindow(
        "Stewart USB Ball Tracker",
        cv2.WINDOW_NORMAL
    )

    cv2.namedWindow(
        "Orange Mask",
        cv2.WINDOW_NORMAL
    )
    cv2.namedWindow("Blue Mask", cv2.WINDOW_NORMAL)
    cv2.namedWindow("Green Mask", cv2.WINDOW_NORMAL)

    calibration = load_json(CAL_FILE)

    if calibration:
        if calibration.get("camera_device_name") != camera_name:
            calibration = None
        else:
            print(
                f"Loaded calibration: {CAL_FILE}\n"
                f"Reference radius: {reference_radius_px(calibration):.1f} px; "
                f"tracking radius: {effective_plate_radius_px(calibration):.1f} px "
                f"(scale={TRACKING_RADIUS_SCALE:.2f})."
            )
    print(f"Tracker source: {Path(__file__).resolve()}")

    marker_state = {
        "stage": None,
        "center": None,
        "radius": None,
        "shape": None,
    }

    new_calibration = False

    # This is independent of the existing ball center calibration.
    # Create it only with the upper plate physically centered and level.
    center_ref = load_json(CENTER_FILE)
    if center_ref is not None and center_ref.get("camera_device_name") != camera_name:
        center_ref = None
    if center_ref is not None:
        print("Plate-base centering reference loaded; press V to recapture at neutral.")
    # A base pixel coordinate saved during V calibration survives restarts,
    # provided the same camera and frame resolution are used.
    locked_base_xy = None
    if center_ref is not None and calibration is not None:
        saved_base = center_ref.get("base_marker_px")
        if (isinstance(saved_base, (list, tuple)) and len(saved_base) == 2
                and center_ref.get("image_width") == calibration.get("image_width")
                and center_ref.get("image_height") == calibration.get("image_height")):
            try:
                candidate_xy = (float(saved_base[0]), float(saved_base[1]))
                if all(math.isfinite(v) for v in candidate_xy):
                    locked_base_xy = candidate_xy
                    print("GREEN base position restored from saved V reference:", locked_base_xy)
            except (ValueError, TypeError):
                pass
    green_lock_candidate = None
    green_lock_count = 0
    detected_markers = None
    last_good_platform_center = None
    last_plate_sent = -float("inf")
    last_plate_good = -float("inf")
    plate_lost_sent = False

    # ========================================================
    # MOUSE CALIBRATION
    # ========================================================

    def mouse_callback(event, x, y, flags, param):
        nonlocal calibration, new_calibration

        if event != cv2.EVENT_LBUTTONDOWN:
            return

        if marker_state["stage"] != "marker":
            return

        cx, cy = marker_state["center"]
        radius = marker_state["radius"]

        plate_radius = math.hypot(x-cx, y-cy)

        if plate_radius <= 10:
            print("Click the RED marker away from center.")
            return

        heading = math.degrees(
            math.atan2(x-cx, cy-y)
        ) % 360

        calibration = {
            "center_x": cx,
            "center_y": cy,
            "scale_px_to_cm": (
                BALL_DIAMETER_CM / (2*radius)
            ),
            "zero_azimuth_deg": (-heading) % 360,
            "plate_radius_px": plate_radius,
            "red_marker_x": x,
            "red_marker_y": y,
            "ball_diameter_cm": BALL_DIAMETER_CM,
            "image_width": marker_state["shape"][1],
            "image_height": marker_state["shape"][0],
            "camera_device_name": camera_name
        }

        marker_state["stage"] = None

        save_json(CAL_FILE, calibration)

        new_calibration = True

        print(
            f"Calibration saved. Active plate radius: "
            f"{effective_plate_radius_px(calibration):.1f}px "
            f"(cyan 1.00x, tracking {TRACKING_RADIUS_SCALE:.2f}x)."
        )

    cv2.setMouseCallback(
        "Stewart USB Ball Tracker",
        mouse_callback
    )

    # ========================================================
    # LOCAL TCP SERVER FOR MATLAB
    # ========================================================

    server = None
    client = None

    if not args.camera_check:

        server = socket.socket(
            socket.AF_INET,
            socket.SOCK_STREAM
        )

        server.setsockopt(
            socket.SOL_SOCKET,
            socket.SO_REUSEADDR,
            1
        )

        server.bind(("127.0.0.1", args.port))
        server.listen(1)
        server.setblocking(False)

        print(
            f"MATLAB bridge: 127.0.0.1:{args.port}"
        )

    def send_line(message):
        nonlocal client

        if client is None:
            return

        try:
            client.sendall(
                (message + "\n").encode("ascii")
            )
        except (OSError, ConnectionError):
            try:
                client.close()
            except OSError:
                pass
            client = None

    def send_calibration():

        if calibration is None:
            return

        send_line(
            "CAL {:.8f} {:.8f} {:.3f} {:.3f} {:.4f}".format(
                calibration["scale_px_to_cm"],
                calibration["zero_azimuth_deg"],
                calibration["center_x"],
                calibration["center_y"],
                effective_plate_radius_px(calibration)
            )
        )
        # Physical R (cm) = CYAN center-to-red-marker radius in pixels
        # multiplied by the existing 4-cm-ball calibration scale.
        # Distinct from CAL's larger MAGENTA ball-detection radius.
        reference_r_cm = (reference_radius_px(calibration) *
                          calibration["scale_px_to_cm"])
        send_line(f"REF_R {reference_r_cm:.8f}")

    # ========================================================
    # START TRACKING
    # ========================================================

    print("\nUSB phone camera tracker running.")
    print("C = Calibrate (ball must be at plate center)")
    print("R = Recalibrate (changes require MATLAB manual re-arm)")
    print("V = Save PLATE/BASE centering reference when physically centered")
    print("Markers: 2 BLUE on upper plate, 1 GREEN on fixed base (locks after detection)")
    print("Blue Marker HSV and Green Marker HSV windows tune them separately.")
    print("Only markers COMPLETELY INSIDE the magenta tracking circle are accepted.")
    print("After GREEN is locked, only the two BLUE markers require continuous tracking.")
    print(f"BLUE occlusions: XY trim frozen for up to {MARKER_OCCLUSION_GRACE_S:.2f}s; longer gaps -> HOLD.")
    print("Q = Quit\n")

    reader.start()

    current_frame = None
    latest_ball = None

    last_frame_time = -float("inf")
    last_detection_time = -float("inf")
    last_send_time = -float("inf")

    lost_sent = False
    last_warning = 0

    try:
        while True:

            now = time.monotonic()

            # MATLAB connection
            if server is not None:
                try:
                    connection, address = server.accept()

                    connection.settimeout(0.01)

                    if client is not None:
                        client.close()

                    client = connection
                    print("MATLAB connected:", address)

                    send_calibration()
                    lost_sent = False

                except BlockingIOError:
                    pass

            # New camera frame
            new_frame = False

            try:
                frame_time, current_frame = (
                    frame_queue.get_nowait()
                )

                last_frame_time = frame_time
                new_frame = True

            except queue.Empty:
                pass

            if new_frame:

                height, width = current_frame.shape[:2]

                # Previous calibration invalid if size changes
                if calibration is not None:

                    if (
                        calibration.get("image_width") != width
                        or calibration.get("image_height") != height
                    ):
                        calibration = None
                        center_ref = None
                        locked_base_xy = None
                        green_lock_candidate = None
                        green_lock_count = 0
                        new_calibration = False
                        send_line("LOST")
                        send_line("PLATE_LOST")

                        print(
                            "Camera resolution changed. "
                            "Calibrate again."
                        )

                settings = get_hsv_settings()

                ball, mask = detect_ball(
                    current_frame, settings, calibration
                )

                latest_ball = ball
                display = current_frame.copy()

                # Detect the moving plate; green is only needed until locked.
                blue_settings = get_hsv_settings("Blue Marker HSV")
                green_settings = get_hsv_settings("Green Marker HSV")
                detected_markers, blue_dots, green_dots, blue_mask, green_mask = detect_platform_markers(
                    current_frame, calibration, blue_settings, green_settings,
                    return_masks=True, locked_base_xy=locked_base_xy)

                # Accept the green base location after three consecutive,
                # spatially consistent measurements with both blue markers.
                if locked_base_xy is None and calibration is not None:
                    if len(green_dots) == 1 and len(blue_dots) == 2:
                        candidate_xy = (green_dots[0][1], green_dots[0][2])
                        if (green_lock_candidate is not None and
                                math.hypot(candidate_xy[0] - green_lock_candidate[0],
                                           candidate_xy[1] - green_lock_candidate[1])
                                <= GREEN_LOCK_TOLERANCE_PX):
                            green_lock_count += 1
                        else:
                            green_lock_count = 1
                        green_lock_candidate = candidate_xy
                        if green_lock_count >= GREEN_LOCK_FRAMES:
                            locked_base_xy = candidate_xy
                            print("GREEN base marker LOCKED at "
                                  f"({candidate_xy[0]:.1f}, {candidate_xy[1]:.1f}) px")
                    else:
                        green_lock_candidate = None
                        green_lock_count = 0

                # No platform measurement until green has been locked.
                if locked_base_xy is None:
                    detected_markers = None
                elif len(blue_dots) == 2:
                    b0, b1 = blue_dots
                    if math.hypot(b0[1]-b1[1], b0[2]-b1[2]) >= MARKER_MIN_SEPARATION_PX:
                        detected_markers = (((b0[1]+b1[1])/2,
                                             (b0[2]+b1[2])/2), locked_base_xy)

                # Show accepted markers individually and make tuning visible.
                for _, mx, my in blue_dots:
                    cv2.circle(display, (round(mx), round(my)), 12,
                               (255, 0, 0), 2)
                    cv2.circle(display, (round(mx), round(my)), 3,
                               (255, 255, 255), -1)
                for _, mx, my in green_dots:
                    cv2.circle(display, (round(mx), round(my)), 12,
                               (0, 255, 0), 2)
                    cv2.circle(display, (round(mx), round(my)), 3,
                               (255, 255, 255), -1)

                # If the fixed GREEN base mark is hidden but both BLUE marks
                # are visible, the actual moving plate center remains known.
                # This helps BALL control, but NEVER authorizes new PLATE XY
                # feedback unless a green position was previously locked.
                visible_blue_center = None
                if len(blue_dots) == 2:
                    b0, b1 = blue_dots
                    if math.hypot(b0[1]-b1[1], b0[2]-b1[2]) >= MARKER_MIN_SEPARATION_PX:
                        visible_blue_center = ((b0[1]+b1[1])/2,
                                               (b0[2]+b1[2])/2)

                marker_valid = detected_markers is not None
                status = f"BLUE: {len(blue_dots)}/2 | GREEN: {'LOCKED' if locked_base_xy is not None else str(len(green_dots))+'/1'}"
                cv2.putText(display, status, (12, 137),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.67,
                            (0, 220, 0) if marker_valid else (0, 0, 255), 2)
                if locked_base_xy is not None:
                    cv2.drawMarker(display, tuple(map(round, locked_base_xy)),
                                   (0, 180, 0), cv2.MARKER_CROSS, 28, 2)
                if calibration is None:
                    cv2.putText(display, "CALIBRATE CYAN CIRCLE FIRST (C)",
                                (12, 166), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                                (0, 140, 255), 2)
                elif not marker_valid:
                    cv2.putText(display, "Need 2 BLUE + locked GREEN reference",
                                (12, 166), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                                (0, 140, 255), 2)

                if detected_markers is not None:
                    plate_xy, base_xy = detected_markers
                    last_good_platform_center = plate_xy
                    cv2.drawMarker(display, tuple(map(round,plate_xy)),
                                   (0,255,0), cv2.MARKER_CROSS, 20, 2)
                    cv2.drawMarker(display, tuple(map(round,base_xy)),
                                   (0,255,0), cv2.MARKER_CROSS, 20, 2)
                    if (center_ref is not None and calibration is not None and
                            center_ref.get("image_width") == width and
                            center_ref.get("image_height") == height):
                        base_offset = center_ref["plate_minus_base_px"]
                        dx = (plate_xy[0]-base_xy[0]-base_offset[0])
                        dy = (plate_xy[1]-base_xy[1]-base_offset[1])
                        error_px = math.hypot(dx,dy)
                        scale = calibration["scale_px_to_cm"]
                        plate_error_cm = (dx*scale, dy*scale)
                        last_plate_good = frame_time
                        if client is not None and frame_time-last_plate_sent >= 1/PLATE_SEND_HZ:
                            send_line(f"PLATE {plate_error_cm[0]:.6f} {plate_error_cm[1]:.6f}")
                            last_plate_sent = frame_time
                            plate_lost_sent = False
                        base_target = (round(base_xy[0]+base_offset[0]),
                                       round(base_xy[1]+base_offset[1]))
                        cv2.drawMarker(display, base_target, (0,255,255),
                                       cv2.MARKER_CROSS, 23, 2)
                        cv2.line(display, tuple(map(round,plate_xy)), base_target,
                                 (0,0,255), 2)
                        cv2.putText(display,
                                    f"Plate offset: {math.hypot(*plate_error_cm):.2f} cm",
                                    (12,105), cv2.FONT_HERSHEY_SIMPLEX,
                                    0.6, (255,255,0), 2)
                elif center_ref is not None:
                    age = frame_time - last_plate_good
                    if last_good_platform_center is not None and 0 <= age < MARKER_OCCLUSION_GRACE_S:
                        px, py = map(round, last_good_platform_center)
                        cv2.drawMarker(display, (px, py), (180,180,180),
                                       cv2.MARKER_CROSS, 20, 2)
                        cv2.putText(display, f"MARKER OCCLUDED {age:.2f}s - XY FROZEN",
                                    (12,105), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                                    (0,200,255),2)
                    else:
                        cv2.putText(display,"MARKERS LOST - MATLAB HOLD",(12,105),
                                    cv2.FONT_HERSHEY_SIMPLEX,0.6,(0,0,255),2)


                # Show the actual plate ROI on every frame. The red marker
                # defines its radius from the stored plate center.
                if calibration is not None:
                    cx = calibration["center_x"]
                    cy = calibration["center_y"]
                    # Cyan = exact reference radius (1.00x).
                    # Magenta = larger tracking boundary (default 1.50x).
                    cv2.circle(display, (round(cx), round(cy)),
                               round(reference_radius_px(calibration)), (255, 255, 0), 2)
                    cv2.circle(display, (round(cx), round(cy)),
                               round(effective_plate_radius_px(calibration)), (255, 0, 255), 2)
                    # Green + follows the measured moving upper-plate center.
                    # If plate markers are absent, show the saved ball center.
                    if detected_markers is None:
                        cv2.drawMarker(display, (round(cx), round(cy)),
                                       (0, 255, 0), cv2.MARKER_CROSS, 20, 2)
                    if "red_marker_x" in calibration and "red_marker_y" in calibration:
                        cv2.drawMarker(display,
                                       (round(calibration["red_marker_x"]),
                                        round(calibration["red_marker_y"])),
                                       (0, 0, 255), cv2.MARKER_CROSS, 18, 2)

                if ball is not None:
                    area, bx, by, radius = ball
                    cv2.circle(display, (round(bx), round(by)),
                               round(radius), (0, 255, 0), 2)
                    cv2.circle(display, (round(bx), round(by)),
                               3, (0, 0, 255), -1)

                    if calibration is not None:
                        scale = calibration["scale_px_to_cm"]
                        if detected_markers is not None:
                            center_ball_x, center_ball_y = detected_markers[0]
                        elif visible_blue_center is not None:
                            center_ball_x, center_ball_y = visible_blue_center
                        elif (last_good_platform_center is not None and
                              0 <= frame_time-last_plate_good < MARKER_OCCLUSION_GRACE_S):
                            # Visual estimate only; freeze until genuine markers return.
                            center_ball_x, center_ball_y = last_good_platform_center
                        else:
                            # No trustworthy moving-plate center: do not send
                            # misleading BALL coordinates to the motors.
                            center_ball_x, center_ball_y = cx, cy
                        error_x = (bx-center_ball_x) * scale
                        error_y = (by-center_ball_y) * scale

                        # detect_ball already established that the entire
                        # orange ball is inside the center/marker circle.
                        last_detection_time = frame_time
                        moving_center_fresh = (detected_markers is not None or
                            (last_good_platform_center is not None and
                             0 <= frame_time-last_plate_good < MARKER_OCCLUSION_GRACE_S))
                        # If a calibrated centering reference exists, require a
                        # recently measured moving-plate center to send BALL.
                        may_send_ball = center_ref is None or moving_center_fresh
                        if (may_send_ball and client is not None and
                                frame_time-last_send_time >= 1/SEND_HZ):
                            send_line(f"BALL {error_x:.6f} {error_y:.6f}")
                            last_send_time = frame_time
                            lost_sent = False

                        cv2.putText(display,
                                    f"Ball X={error_x:+.2f} cm Y={error_y:+.2f} cm",
                                    (12, 32), cv2.FONT_HERSHEY_SIMPLEX,
                                    0.7, (0, 255, 0), 2)

                # Display instructions
                if marker_state["stage"] == "marker":

                    message = "Click RED reference marker"

                elif calibration is None:

                    message = (
                        "Ball at CENTER: press C"
                    )

                else:

                    message = (
                        f"CYAN {REFERENCE_RADIUS_SCALE:.2f}x | TRACK (magenta) {TRACKING_RADIUS_SCALE:.2f}x"
                    )

                cv2.putText(
                    display,
                    message,
                    (12, 70),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    (0, 255, 255),
                    2
                )

                cv2.imshow(
                    "Stewart USB Ball Tracker",
                    display
                )

                cv2.imshow(
                    "Orange Mask",
                    mask
                )
                cv2.imshow("Blue Mask", blue_mask)
                cv2.imshow("Green Mask", green_mask)

            # Send new calibration to MATLAB
            if new_calibration:
                new_calibration = False
                send_calibration()

            # Ball loss handling
            if (
                calibration is not None
                and now-last_detection_time
                > BALL_LOSS_TIMEOUT_S
                and not lost_sent
            ):
                send_line("LOST")
                lost_sent = True

            # Independent loss signal: MATLAB must not continue applying
            # centering correction using stale plate-marker measurements.
            if (center_ref is not None and
                    marker_occlusion_expired(last_plate_good, now) and not plate_lost_sent):
                send_line("PLATE_LOST")
                plate_lost_sent = True

            # Camera disconnect handling
            if (
                now-last_frame_time > MAX_FRAME_GAP_S
                and now-last_warning > 2
            ):
                print(
                    "USB camera frame gap. "
                    "MATLAB should HOLD."
                )

                last_warning = now

            # Keyboard controls
            key = cv2.waitKey(1) & 0xFF

            if key in (27, ord("q")):
                break

            if key == ord("v"):
                if detected_markers is None or now-last_frame_time >= MAX_FRAME_GAP_S:
                    print("Cannot calibrate center: lock GREEN and detect both BLUE markers first.")
                elif calibration is None:
                    print("Calibrate the ball center first (C), then center reference (V).")
                elif len(green_dots) != 1:
                    print("GREEN must be visible when saving/recalibrating with V.")
                else:
                    # Use the freshly visible green location for V calibration.
                    # Only press V with the actual upper plate centered and level.
                    plate_xy, _ = detected_markers
                    base_xy = (green_dots[0][1], green_dots[0][2])
                    locked_base_xy = base_xy
                    green_lock_candidate = base_xy
                    green_lock_count = GREEN_LOCK_FRAMES
                    center_ref = {
                        "camera_device_name": camera_name,
                        "image_width": int(current_frame.shape[1]),
                        "image_height": int(current_frame.shape[0]),
                        "plate_minus_base_px": [plate_xy[0]-base_xy[0],
                                                 plate_xy[1]-base_xy[1]],
                        "base_marker_px": [base_xy[0], base_xy[1]]
                    }
                    save_json(CENTER_FILE, center_ref)
                    print("Plate/base reference saved, GREEN position locked across restarts.")
                    send_line("PLATE_CAL_CHANGED")
                    last_plate_good = -float("inf")
                    last_good_platform_center = None
                    plate_lost_sent = True

            if key in (ord("c"), ord("r")):

                if (
                    current_frame is not None
                    and latest_ball is not None
                    and now-last_frame_time < MAX_FRAME_GAP_S
                ):

                    area, bx, by, radius = latest_ball

                    marker_state.update({
                        "stage": "marker",
                        "center": (bx, by),
                        "radius": radius,
                        "shape": current_frame.shape
                    })

                    calibration = None
                    center_ref = None
                    locked_base_xy = None
                    green_lock_candidate = None
                    green_lock_count = 0
                    last_good_platform_center = None
                    last_plate_good = -float("inf")

                    send_line("LOST")
                    send_line("PLATE_CAL_CHANGED")

                    print(
                        "Ball center stored. "
                        "Click the RED reference marker."
                    )

                else:
                    print(
                        "Ball not detected. "
                        "Adjust HSV sliders."
                    )

            if current_frame is None:
                time.sleep(0.01)

    finally:

        stop_event.set()
        reader.join(timeout=1.0)

        save_json(HSV_FILE, get_hsv_settings())
        save_json(MARKER_HSV_FILE, {
            "blue": get_hsv_settings("Blue Marker HSV"),
            "green": get_hsv_settings("Green Marker HSV")
        })

        if client is not None:
            client.close()

        if server is not None:
            server.close()

        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
