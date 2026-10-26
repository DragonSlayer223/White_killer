import json
import os
import queue
import cv2
import numpy as np

CONFIG_FILE = "calibration_config.json"
BALL_DIAMETER_CM = 4.0


class CalibrationCancelled(Exception):
    pass


def detect_red_reference(
    frame, center, hue_low_max, hue_high_min, saturation_min, value_min
):
    """Find the visible red marker farthest from the calibrated plate center."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    low_red = cv2.inRange(
        hsv,
        np.array([0, saturation_min, value_min], dtype=np.uint8),
        np.array([hue_low_max, 255, 255], dtype=np.uint8),
    )
    high_red = cv2.inRange(
        hsv,
        np.array([hue_high_min, saturation_min, value_min], dtype=np.uint8),
        np.array([179, 255, 255], dtype=np.uint8),
    )
    mask = cv2.bitwise_or(low_red, high_red)
    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_CLOSE,
        np.ones((3, 3), dtype=np.uint8),
    )

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    candidates = []
    center_x, center_y = center
    for contour in contours:
        area = cv2.contourArea(contour)
        _, _, width, height = cv2.boundingRect(contour)
        if area < 2 or width * height < 6:
            continue

        moments = cv2.moments(contour)
        if moments["m00"] == 0:
            continue
        marker_x = moments["m10"] / moments["m00"]
        marker_y = moments["m01"] / moments["m00"]
        distance_squared = (marker_x - center_x) ** 2 + (marker_y - center_y) ** 2
        candidates.append(
            (distance_squared, area, marker_x, marker_y, contour)
        )

    if not candidates:
        return mask, None

    _, area, marker_x, marker_y, contour = max(
        candidates, key=lambda candidate: candidate[0]
    )
    heading_deg = np.degrees(
        np.arctan2(marker_x - center_x, center_y - marker_y)
    ) % 360.0
    return mask, (area, marker_x, marker_y, heading_deg, contour)


def load_calibration():
    """Load calibration data from JSON file if it exists."""
    if os.path.exists(CONFIG_FILE):
        try:
            with open(CONFIG_FILE, "r") as f:
                config = json.load(f)
                print(f"Loaded existing calibration from {CONFIG_FILE}")
                return config
        except Exception as e:
            print(f"Error loading calibration file: {e}")
    return None

def save_calibration(
    center_x, center_y, scale_px_to_cm, zero_azimuth_deg, plate_radius_px
):
    """Save calibration parameters to a JSON file."""
    config = {
        "center_x": float(center_x),
        "center_y": float(center_y),
        "scale_px_to_cm": float(scale_px_to_cm),
        "zero_azimuth_deg": float(zero_azimuth_deg),
        "plate_radius_px": float(plate_radius_px),
    }
    with open(CONFIG_FILE, "w") as f:
        json.dump(config, f, indent=4)
    print(f"Calibration successfully saved to {CONFIG_FILE}!")

def run_interactive_calibration(
    frame_queue, get_hsv_bounds, get_red_bounds, detector, stop_event
):
    """Lock center, then confirm a visible red azimuth reference."""
    print("=== STARTING INTERACTIVE CALIBRATION ===")
    print("1. Place the ball in the EXACT CENTER of the plate.")
    print("2. Press SPACE to lock the center, then press SPACE to lock red tape.")
    
    center_pos = None
    scale_px_to_cm = None
    zero_azimuth_deg = 0.0
    plate_radius_px = None
    cv2.namedWindow("Calibration", cv2.WINDOW_NORMAL)
    cv2.namedWindow("Red Tape Mask", cv2.WINDOW_NORMAL)
    
    while not stop_event.is_set():
        try:
            frame = frame_queue.get(timeout=0.1)
        except queue.Empty:
            display = np.zeros((360, 640, 3), dtype=np.uint8)
            cv2.putText(
                display,
                "Waiting for phone video stream...",
                (15, 180),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )
            cv2.imshow("Calibration", display)
            if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                raise CalibrationCancelled("Calibration cancelled by user.")
            continue
            
        display = frame.copy()
        detection = None
        red_detection = None
        red_mask = np.zeros(frame.shape[:2], dtype=np.uint8)
        if center_pos is None:
            lower_hsv, upper_hsv = get_hsv_bounds()
            _, detection = detector(frame, lower_hsv, upper_hsv)

            if detection is not None:
                _, x, y, r = detection
                cv2.circle(display, (int(x), int(y)), int(r), (255, 0, 0), 2)
                cv2.putText(
                    display,
                    "Ball detected! Press SPACE to lock center.",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2,
                )
            else:
                cv2.putText(
                    display,
                    "No ball found. Place ball in center.",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 0, 255),
                    2,
                )

        else:
            hue_low_max, hue_high_min, saturation_min, value_min = get_red_bounds()
            red_mask, red_detection = detect_red_reference(
                frame,
                center_pos,
                hue_low_max,
                hue_high_min,
                saturation_min,
                value_min,
            )
            cv2.drawMarker(
                display,
                (int(center_pos[0]), int(center_pos[1])),
                (255, 0, 0),
                cv2.MARKER_CROSS,
                20,
                2,
            )
            if red_detection is not None:
                area, marker_x, marker_y, heading_deg, contour = red_detection
                cv2.drawContours(display, [contour], -1, (0, 0, 255), 2)
                cv2.circle(display, (int(marker_x), int(marker_y)), 5, (0, 0, 255), -1)
                cv2.line(
                    display,
                    (int(center_pos[0]), int(center_pos[1])),
                    (int(marker_x), int(marker_y)),
                    (255, 255, 0),
                    2,
                )
                instruction = (
                    f"Red marker detected (area {area:.0f}, "
                    f"heading {heading_deg:.1f} deg). SPACE to confirm."
                )
                cv2.putText(
                    display,
                    instruction,
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (0, 255, 0),
                    2,
                )
            else:
                heading_deg = None
                cv2.putText(
                    display,
                    "Red tape not found. Adjust red HSV sliders.",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 0, 255),
                    2,
                )

        cv2.imshow("Calibration", display)
        cv2.imshow("Red Tape Mask", red_mask)
        key = cv2.waitKey(1) & 0xFF

        if key == ord(" ") and center_pos is None:
            if detection is not None:
                _, x, y, r = detection
                center_pos = (x, y)
                scale_px_to_cm = BALL_DIAMETER_CM / (2.0 * r)
                print(f"Center locked at: X={x:.1f}, Y={y:.1f}")
                print("Now locate the red tape and press SPACE to confirm its direction.")
        elif key == ord(" ") and center_pos is not None:
            if red_detection is not None:
                _, marker_x, marker_y, heading_deg, _ = red_detection
                plate_radius_px = float(
                    np.hypot(marker_x - center_pos[0], marker_y - center_pos[1])
                )
                if plate_radius_px <= 0:
                    continue
                zero_azimuth_deg = (-heading_deg) % 360.0
                print(
                    f"Red tape confirmed at ({marker_x:.1f}, {marker_y:.1f}); "
                    f"reference heading={heading_deg:.1f} deg, "
                    f"azimuth offset={zero_azimuth_deg:.1f} deg."
                )
                break
        if key in (ord("q"), 27):
            raise CalibrationCancelled("Calibration cancelled by user.")
    else:
        raise CalibrationCancelled("Calibration stopped.")
            
    cv2.destroyWindow("Calibration")
    cv2.destroyWindow("Red Tape Mask")
    
    # Save values to JSON
    save_calibration(
        center_pos[0],
        center_pos[1],
        scale_px_to_cm,
        zero_azimuth_deg,
        plate_radius_px,
    )
    return (
        center_pos[0],
        center_pos[1],
        scale_px_to_cm,
        zero_azimuth_deg,
        plate_radius_px,
    )
