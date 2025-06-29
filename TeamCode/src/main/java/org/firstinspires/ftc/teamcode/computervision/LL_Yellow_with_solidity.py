import cv2
import numpy as np
import math
import time
from collections import defaultdict

# --- TUNABLE CONSTANTS ---
SMALL_CONTOUR_AREA = 300
MIN_BRIGHTNESS_THRESHOLD = 50
MIN_ASPECT_RATIO = 1.5
MAX_ASPECT_RATIO = 6.0
VERTICAL_THRESHOLD = 200
MIN_SOLIDITY = 0.90

# How far (in pixels) from the robot's true centerline a sample can be to be "straight ahead".
CENTER_TOLERANCE_PX = 40

# Color detection ranges for yellow in HSV
HSV_YELLOW_RANGE = ([15, 60, 100], [80, 255, 255])


def calculate_angle(contour):
    if len(contour) < 5: return 0
    (x, y), (MA, ma), angle = cv2.fitEllipse(contour)
    return angle

# <<< HIGHLIGHT: Modified draw_info to include solidity for debugging
def draw_info(image, color, angle, center, index, area, solidity):
    cv2.putText(image, f"#{index}: {color}", (center[0] - 50, center[1] - 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    cv2.putText(image, f"Angle: {angle:.2f}", (center[0] - 50, center[1] - 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    cv2.putText(image, f"Area: {area:.2f}", (center[0] - 50, center[1] - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    cv2.putText(image, f"Solid: {solidity:.2f}", (center[0] - 50, center[1] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    cv2.circle(image, center, 5, (0, 255, 0), -1)

def separate_touching_contours(contour, min_area_ratio=0.15):
    x, y, w, h = cv2.boundingRect(contour)
    mask = np.zeros((h, w), dtype=np.uint8)
    shifted_contour = contour - [x, y]
    cv2.drawContours(mask, [shifted_contour], -1, 255, -1)
    original_area = cv2.contourArea(contour)
    max_contours = []
    max_count = 1
    dist_transform = cv2.distanceTransform(mask, cv2.DIST_L2, 3)
    for threshold in np.linspace(0.1, 0.9, 9):
        _, thresh = cv2.threshold(dist_transform, threshold * dist_transform.max(), 255, 0)
        thresh = np.uint8(thresh)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        valid_contours = [c for c in contours if cv2.contourArea(c) > original_area * min_area_ratio]
        if len(valid_contours) > max_count:
            max_count = len(valid_contours)
            max_contours = valid_contours
    if max_contours:
        return [c + [x, y] for c in max_contours]
    return [contour]


def process_color(frame, mask):
    kernel = np.ones((5, 5), np.uint8)
    masked_frame = cv2.bitwise_and(frame, frame, mask=mask)
    gray_masked = cv2.cvtColor(masked_frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray_masked, (3, 3), 0)
    sobelx = cv2.Sobel(blurred, cv2.CV_32F, 1, 0, ksize=1)
    sobely = cv2.Sobel(blurred, cv2.CV_32F, 0, 1, ksize=1)
    magnitude = np.sqrt(sobelx**2 + sobely**2)
    magnitude = np.uint8(magnitude * 255 / np.max(magnitude))
    _, edges = cv2.threshold(magnitude, 50, 255, cv2.THRESH_BINARY)
    edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
    edges = cv2.dilate(edges, np.ones((3, 3), np.uint8), iterations=1)
    edges = cv2.bitwise_not(edges)
    edges = cv2.bitwise_and(edges, edges, mask=mask)
    edges = cv2.GaussianBlur(edges, (3, 3), 0)
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return contours, gray_masked


def runPipeline(frame, llrobot):
    try:
        llpython = [0, 0, 0, 0, 0, 0, 0, 0]
        
        frame_height, frame_width, _ = frame.shape
        image_center_x = frame_width / 2

        # Calculate robot's true center in the image using offset from Java
        horizontal_offset_cm = llrobot[1] if llrobot and len(llrobot) > 1 else 0
        fov_deg = llrobot[2] if llrobot and len(llrobot) > 2 else 55.0

        # A very rough approximation for pixels per cm at a typical intake distance
        # A more accurate method would involve `ty` but this is sufficient for prioritization
        px_per_cm_at_distance = 8 
        offset_in_px = horizontal_offset_cm * px_per_cm_at_distance
        
        # If camera is on the RIGHT (positive offset), its center is LEFT of the image center
        robot_center_x = image_center_x - offset_in_px

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        yellow_mask = cv2.inRange(hsv, np.array(HSV_YELLOW_RANGE[0]), np.array(HSV_YELLOW_RANGE[1]))
        yellow_mask = cv2.erode(yellow_mask, np.ones((3, 3), np.uint8))

        yellow_contours, yellow_gray = process_color(frame, yellow_mask)
        
        valid_contours_info = []
        for i, contour in enumerate(yellow_contours):
            if cv2.contourArea(contour) < SMALL_CONTOUR_AREA: continue
            
            rect = cv2.minAreaRect(contour)
            width, height = rect[1]
            if width == 0 or height == 0: continue
            aspect_ratio = max(width, height) / min(width, height)
            if not (MIN_ASPECT_RATIO <= aspect_ratio <= MAX_ASPECT_RATIO): continue

            for sep_contour in separate_touching_contours(contour):
                area = cv2.contourArea(sep_contour)
                if area < SMALL_CONTOUR_AREA: continue

                # <<< HIGHLIGHT: New Solidity Filter
                hull = cv2.convexHull(sep_contour)
                if cv2.contourArea(hull) == 0: continue
                solidity = float(area) / cv2.contourArea(hull)
                if solidity < MIN_SOLIDITY: continue
                # End of new filter

                mask = np.zeros(yellow_gray.shape, dtype=np.uint8)
                cv2.drawContours(mask, [sep_contour], -1, 255, -1)
                if cv2.mean(yellow_gray, mask=mask)[0] < MIN_BRIGHTNESS_THRESHOLD: continue

                M = cv2.moments(sep_contour)
                if M["m00"] == 0: continue
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                if center[1] < frame_height - VERTICAL_THRESHOLD: continue
                
                # <<< HIGHLIGHT: Categorize target based on robot's true center
                position_category = ""
                if abs(center[0] - robot_center_x) < CENTER_TOLERANCE_PX:
                    position_category = "Center"
                elif center[0] < robot_center_x:
                    position_category = "Left"
                else:
                    position_category = "Right"

                valid_contours_info.append({
                    'contour': sep_contour, 'center': center, 'angle': calculate_angle(sep_contour), 'area': area,
                    'solidity': solidity, 'index': i, 'category': position_category
                })
        
        # Prioritization Logic
        best_target = None
        center_targets = [c for c in valid_contours_info if c['category'] == "Center"]
        left_targets = [c for c in valid_contours_info if c['category'] == "Left"]
        right_targets = [c for c in valid_contours_info if c['category'] == "Right"]

        if center_targets: best_target = max(center_targets, key=lambda c: c['area'])
        elif left_targets: best_target = max(left_targets, key=lambda c: c['area'])
        elif right_targets: best_target = max(right_targets, key=lambda c: c['area'])

        # Output section
        if best_target:
            center, angle = best_target['center'], best_target['angle']
            llpython = [1, center[0], center[1], angle, len(valid_contours_info), 0, 0, 0]
            
            # Draw all valid targets in blue
            for c in valid_contours_info:
                cv2.drawContours(frame, [c['contour']], -1, (255, 0, 0), 2)
                draw_info(frame, "Yellow", c['angle'], c['center'], c['index'] + 1, c['area'], c['solidity'])
            
            # Highlight the chosen best target in green
            cv2.drawContours(frame, [best_target['contour']], -1, (0, 255, 0), 2)
            cv2.circle(frame, best_target['center'], 10, (0, 255, 0), 2)

        # Draw the robot's true center for debugging
        cv2.line(frame, (int(robot_center_x), 0), (int(robot_center_x), frame_height), (0, 0, 255), 2)
            
        return best_target['contour'] if best_target else np.array([[]]), frame, llpython

    except Exception as e:
        print(f"Error: {str(e)}")
        return np.array([[]]), frame, [0, 0, 0, 0, 0, 0, 0, 0]