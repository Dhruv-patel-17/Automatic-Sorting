"""
VisionSort - Contour-based Lid/Base Sorting Pipeline (Direct Window Capture)
========================================================================
Detects objects using CONTOUR DETECTION (finding any distinct blob
against the belt background), combined with CONTOUR SHAPE ANALYSIS to
distinguish "Lid" vs "Base" - based on each object's silhouette:
  - Lid: a simple rounded square - nearly fully convex outline
  - Base: four L-shaped corner brackets with concave notches between
    them (a cross/bracket cutout shape) - noticeably less convex

WHY NOT YOLO FOR DETECTION (changed from an earlier version of this
script): a pretrained YOLO model (trained on COCO) has never seen this
synthetic game object and doesn't resemble any of COCO's ~80 trained
categories (person, bottle, cup, etc.) closely enough to detect it at
all - testing confirmed YOLO produced zero detections regardless of
confidence threshold. Since the actual need here is just "find any
object-shaped blob on the belt" (not genuine semantic classification),
plain contour detection is the correct, reliable tool - it doesn't
depend on recognizing what the object "is," only that it's a distinct
shape against the background.

This approach was chosen after three other methods failed for this
specific pair of parts:
  - YOLO DETECTION: doesn't recognize this custom synthetic object at all.
  - COLOR: both parts are the same green color family (and color varies
    across game instances), so it's not a reliable signal.
  - ORB FEATURE MATCHING: both parts have some rotational symmetry (the
    Lid's concentric-ring pattern especially), which produces spurious
    "matches" regardless of the real object.

CAPTURE METHOD: captures the Factory I/O window DIRECTLY - no webcam,
no OBS Virtual Camera needed. Finds the Factory I/O window on your
screen and grabs its live pixels every frame, using PrintWindow (avoids
the recursive feedback-loop problem screen-region capture has).

Prerequisites:
  pip install opencv-python numpy pymodbus pygetwindow pywin32

Controls while running:
  q  - quit
"""

import cv2
import csv
import time
import os
import numpy as np
import pygetwindow as gw
import win32gui
import win32ui
import win32con
from ctypes import windll
from pymodbus.client import ModbusTcpClient

# =================================================================
# CONFIGURATION
# =================================================================

# Exact or partial title of the Factory I/O window, as it appears in
# your taskbar. Usually just "Factory IO" - adjust if yours differs.
FACTORY_IO_WINDOW_TITLE = "Factory IO"

# Minimum contour area (pixels) to count as a real object, not background
# noise. Tune based on your actual object size in the captured frame.
MIN_CONTOUR_AREA = 3000

# --- Region of interest within the captured Factory I/O window ---
# Restricts detection to JUST the belt area, cropping out background
# scenery, structural beams, UI panels, etc. that would otherwise be
# picked up as false "objects" by contour detection.
# Expressed as FRACTIONS of the captured frame (0.0-1.0), so this stays
# correct even if the window is resized. Tune these by running with
# SHOW_ROI_DEBUG = True first (draws the crop rectangle on the full
# frame so you can see exactly what it currently covers).
ROI_TOP    = 0.10   # fraction from top of frame where the belt starts
ROI_BOTTOM = 0.95   # fraction from top of frame where the belt ends
ROI_LEFT   = 0.45   # fraction from left of frame where the belt starts
ROI_RIGHT  = 0.73   # fraction from left of frame where the belt ends

SHOW_ROI_DEBUG = False   # set False once the crop region looks correct

# --- Morphological closing ---
# The Base part's design has real physical gaps between its four corner
# brackets - when thresholded, these gaps can split ONE physical object
# into several disconnected blobs, each getting its own tracker ID and
# being classified independently (explaining both "multiple IDs for one
# object" and "conflicting answers" for the same object). Morphological
# CLOSING bridges small gaps by dilating then eroding, merging nearby
# fragments back into a single blob before contour detection runs.
# Increase MORPH_KERNEL_SIZE if fragments still aren't merging; decrease
# if it starts merging genuinely separate/adjacent objects together.
MORPH_KERNEL_SIZE = 21

# IMPORTANT: this is a SEPARATE, much smaller kernel used only inside
# shape classification (classify_design). The large kernel above is
# meant to merge an object's own disconnected fragments into one blob
# for tracking - but Base's real concave notches (the gaps between its
# corner brackets) are the actual FEATURE the shape classifier depends
# on to tell Base apart from Lid. Using the large kernel there too would
# smooth those notches away and break the classifier. This small kernel
# only cleans minor thresholding noise, not the real design gaps.
SHAPE_MORPH_KERNEL_SIZE = 3

# --- Shape classification: Lid (simple convex outline) vs Base (concave bracket cross) ---
# Given repeated difficulty with color (both parts are green, and color
# varies across instances) and feature matching (both parts have
# symmetric patterns that confuse ORB/homography), this uses CONTOUR
# SHAPE ANALYSIS instead - a fundamentally different, rotation-invariant
# signal based on the actual silhouettes:
#   - Lid: a simple rounded square - nearly fully convex outline
#   - Base: four L-shaped corner brackets with CONCAVE notches between
#     them (the cross/bracket cutout) - noticeably less convex
#
# SOLIDITY = contour_area / convex_hull_area. A simple convex shape is
# close to 1.0; a shape with concave notches is meaningfully lower.
LID_MIN_SOLIDITY = 0.99   # Lid should be close to fully convex
BASE_MAX_SOLIDITY = 0.92  # Base's concave notches should pull this down

# A significant concave notch (convexity defect), in pixels, relative to
# the object's own size - used as a second confirming signal alongside
# solidity.
DEFECT_DEPTH_RATIO = 0.04
BASE_MIN_DEFECTS = 2       # Base should show several notches; Lid should show ~0

STABILITY_FRAMES = 3
STABILITY_TOLERANCE_PX = 40

# --- Sorting decision ---
# This is the actual sorting logic: whichever class you set here gets
# diverted by REJECT_SOL (pusher fires -> pushed off the belt).
# The other class is left alone and continues straight through to the
# accept end. Set to "LID" or "BASE".
DIVERT_CLASS = "LID"

# --- OpenPLC / Modbus ---
PLC_IP = "127.0.0.1"
PLC_PORT = 503
COIL_VISION_REJECT_FLAG = 3        # written TRUE if the divert-class (or unrecognized) object appears
COIL_VISION_HEARTBEAT   = 4
COIL_VISION_FAULT_LAMP  = 5
HEARTBEAT_INTERVAL = 0.5

LOG_FILE = "classification_log.csv"


def find_factory_io_hwnd():
    """Locates the Factory I/O window and returns its Windows handle (HWND)."""
    matches = [w for w in gw.getAllWindows() if FACTORY_IO_WINDOW_TITLE.lower() in w.title.lower()]
    if not matches:
        return None
    return matches[0]._hWnd


def capture_window(hwnd):
    """
    Captures the window's actual content directly from its own buffer,
    using PrintWindow - this works regardless of what's on top of it on
    screen (avoids the video-feedback-loop problem that screen-region
    capture has when another window overlaps it, especially fullscreen).
    """
    left, top, right, bottom = win32gui.GetClientRect(hwnd)
    width, height = right - left, bottom - top
    if width <= 0 or height <= 0:
        return None

    hwnd_dc = win32gui.GetWindowDC(hwnd)
    mfc_dc = win32ui.CreateDCFromHandle(hwnd_dc)
    save_dc = mfc_dc.CreateCompatibleDC()

    save_bitmap = win32ui.CreateBitmap()
    save_bitmap.CreateCompatibleBitmap(mfc_dc, width, height)
    save_dc.SelectObject(save_bitmap)

    # PW_RENDERFULLCONTENT (value 2) captures GPU-rendered content too,
    # needed for 3D applications like Factory I/O
    result = windll.user32.PrintWindow(hwnd, save_dc.GetSafeHdc(), 2)

    bmp_info = save_bitmap.GetInfo()
    bmp_bits = save_bitmap.GetBitmapBits(True)
    img = np.frombuffer(bmp_bits, dtype=np.uint8)
    img.shape = (bmp_info["bmHeight"], bmp_info["bmWidth"], 4)

    win32gui.DeleteObject(save_bitmap.GetHandle())
    save_dc.DeleteDC()
    mfc_dc.DeleteDC()
    win32gui.ReleaseDC(hwnd, hwnd_dc)

    if not result:
        return None

    return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)


# =================================================================
# Simple tracker (position-based, works alongside YOLO's own boxes)
# =================================================================
# Number of consecutive frames an object can go undetected before the
# tracker gives up on it and frees its ID. Without this grace period, a
# single missed frame (motion blur, a brief contour flicker) causes the
# same physical object to be re-assigned a NEW id when it reappears,
# triggering a duplicate classification and duplicate PLC signal for
# what is really one object.
MAX_MISSED_FRAMES = 8


class CentroidTracker:
    def __init__(self, max_distance=150, buffer_size=STABILITY_FRAMES):
        self.next_id = 0
        self.objects = {}
        self.missed_frames = {}
        self.position_history = {}
        self.classified = {}
        self.max_distance = max_distance
        self.buffer_size = buffer_size

    def update(self, centroids):
        used_ids = set()
        matched_this_frame = set()

        for c in centroids:
            best_id, best_dist = None, self.max_distance
            for oid, prev_c in self.objects.items():
                if oid in used_ids:
                    continue
                dist = ((c[0]-prev_c[0])**2 + (c[1]-prev_c[1])**2) ** 0.5
                if dist < best_dist:
                    best_dist, best_id = dist, oid
            if best_id is not None:
                self.objects[best_id] = c
                self.missed_frames[best_id] = 0
                used_ids.add(best_id)
                matched_this_frame.add(best_id)
            else:
                new_id = self.next_id
                self.objects[new_id] = c
                self.missed_frames[new_id] = 0
                self.position_history[new_id] = []
                used_ids.add(new_id)
                matched_this_frame.add(new_id)
                self.next_id += 1

        # For tracks that existed but weren't matched this frame, keep
        # them alive (at their last known position) for a grace period
        # instead of immediately forgetting them - this is what prevents
        # ID fragmentation from a brief detection miss.
        for oid in list(self.objects.keys()):
            if oid not in matched_this_frame:
                self.missed_frames[oid] = self.missed_frames.get(oid, 0) + 1
                if self.missed_frames[oid] > MAX_MISSED_FRAMES:
                    del self.objects[oid]
                    del self.missed_frames[oid]

        # Only return tracks that were actually detected THIS frame -
        # callers shouldn't draw/classify a box for a track that's
        # currently in its grace period with no real detection.
        return {oid: self.objects[oid] for oid in matched_this_frame}

    def record_position(self, oid, c):
        hist = self.position_history.setdefault(oid, [])
        hist.append(c)
        if len(hist) > self.buffer_size:
            hist.pop(0)

    def is_stable(self, oid):
        hist = self.position_history.get(oid, [])
        if len(hist) < self.buffer_size:
            return False
        xs = [p[0] for p in hist]
        ys = [p[1] for p in hist]
        return (max(xs)-min(xs) <= STABILITY_TOLERANCE_PX and
                max(ys)-min(ys) <= STABILITY_TOLERANCE_PX)


def connect_plc():
    client = ModbusTcpClient(PLC_IP, port=PLC_PORT)
    if client.connect():
        print(f"[OK] Connected to OpenPLC at {PLC_IP}:{PLC_PORT}")
        return client
    print(f"[ERROR] Could not connect to OpenPLC at {PLC_IP}:{PLC_PORT}")
    return None


def init_log():
    new_file = not os.path.exists(LOG_FILE)
    f = open(LOG_FILE, "a", newline="")
    writer = csv.writer(f)
    if new_file:
        writer.writerow(["timestamp", "object_id", "confidence", "classification", "result"])
    return f, writer


def load_templates_and_detector():
    """Loads the Lid/Base reference images and precomputes ORB features."""
    orb = cv2.ORB_create(nfeatures=500)

    lid_img = cv2.imread(LID_TEMPLATE_PATH, cv2.IMREAD_GRAYSCALE)
    base_img = cv2.imread(BASE_TEMPLATE_PATH, cv2.IMREAD_GRAYSCALE)

    if lid_img is None:
        raise FileNotFoundError(f"Could not load '{LID_TEMPLATE_PATH}' - make sure it exists in this folder.")
    if base_img is None:
        raise FileNotFoundError(f"Could not load '{BASE_TEMPLATE_PATH}' - make sure it exists in this folder.")

    lid_kp, lid_des = orb.detectAndCompute(lid_img, None)
    base_kp, base_des = orb.detectAndCompute(base_img, None)

    return orb, (lid_kp, lid_des), (base_kp, base_des)


def classify_design(roi_bgr):
    """
    Returns 'LID', 'BASE', or 'UNKNOWN' based on the object's silhouette
    shape (solidity + convexity defects) rather than color or texture -
    robust to both parts being the same color family and to the
    rotational symmetry that confused feature matching.
    """
    if roi_bgr.size == 0 or roi_bgr.shape[0] < 10 or roi_bgr.shape[1] < 10:
        return "UNKNOWN", 0, 0

    gray = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    _, thresh = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    # Use the SAME merge kernel as main detection - the goal is one
    # unified contour representing the object's true outer silhouette.
    # The real design notches (Base's cross/bracket shape) are a larger-
    # scale feature than the fine fragmentation gaps this bridges, so
    # they should survive; only shadow/lighting-driven disconnection
    # gets merged away. Verify this with the printed debug numbers below
    # and adjust MORPH_KERNEL_SIZE if solidity numbers don't separate
    # Lid from Base clearly.
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (MORPH_KERNEL_SIZE, MORPH_KERNEL_SIZE))
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return "UNKNOWN", 0, 0

    largest = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(largest)
    if area < 100:
        return "UNKNOWN", 0, 0

    hull = cv2.convexHull(largest)
    hull_area = cv2.contourArea(hull)
    solidity = (area / hull_area) if hull_area > 0 else 0

    # Count significant concave notches (convexity defects)
    defect_count = 0
    hull_indices = cv2.convexHull(largest, returnPoints=False)
    obj_size = max(roi_bgr.shape[0], roi_bgr.shape[1])
    if len(hull_indices) > 3:
        defects = cv2.convexityDefects(largest, hull_indices)
        if defects is not None:
            for i in range(defects.shape[0]):
                _, _, _, d = defects[i, 0]
                depth_px = d / 256.0
                if depth_px > DEFECT_DEPTH_RATIO * obj_size:
                    defect_count += 1

    if solidity >= LID_MIN_SOLIDITY and defect_count < BASE_MIN_DEFECTS:
        return "LID", solidity, defect_count
    elif solidity <= BASE_MAX_SOLIDITY or defect_count >= BASE_MIN_DEFECTS:
        return "BASE", solidity, defect_count
    return "UNKNOWN", solidity, defect_count


def main():
    print(f"[INFO] Looking for Factory I/O window (title contains '{FACTORY_IO_WINDOW_TITLE}')...")
    hwnd = find_factory_io_hwnd()
    if hwnd is None:
        print(f"[ERROR] Could not find a window with '{FACTORY_IO_WINDOW_TITLE}' in its title.")
        print("        Make sure Factory I/O is open (not minimized) and check FACTORY_IO_WINDOW_TITLE.")
        return
    print(f"[OK] Found Factory I/O window (hwnd={hwnd})")

    tracker = CentroidTracker()
    counts = {"LID": 0, "BASE": 0, "UNKNOWN": 0}
    log_file, log_writer = init_log()

    plc_client = connect_plc()
    heartbeat_state = False
    last_heartbeat_time = 0

    while True:
        hwnd = find_factory_io_hwnd()
        if hwnd is None:
            print("[ERROR] Factory I/O window no longer found (closed or minimized?)")
            break

        frame = capture_window(hwnd)
        if frame is None:
            print("[WARNING] Failed to capture a frame this cycle, retrying...")
            continue

        frame_h, frame_w = frame.shape[:2]
        roi_x1 = int(ROI_LEFT * frame_w)
        roi_y1 = int(ROI_TOP * frame_h)
        roi_x2 = int(ROI_RIGHT * frame_w)
        roi_y2 = int(ROI_BOTTOM * frame_h)

        if SHOW_ROI_DEBUG:
            cv2.rectangle(frame, (roi_x1, roi_y1), (roi_x2, roi_y2), (0, 255, 255), 3)
            cv2.putText(frame, "ROI - adjust ROI_TOP/BOTTOM/LEFT/RIGHT until this box tightly fits the belt only",
                        (10, frame_h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

        # Crop to the belt-only region BEFORE detection, so background
        # scenery/UI never reaches contour detection at all
        cropped = frame[roi_y1:roi_y2, roi_x1:roi_x2]

        gray = cv2.cvtColor(cropped, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        _, thresh = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

        # Merge fragmented pieces of the same physical object (e.g. the
        # Base's separate corner brackets) into single connected blobs
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (MORPH_KERNEL_SIZE, MORPH_KERNEL_SIZE))
        closed = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        centroids, boxes, confidences = [], [], []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < MIN_CONTOUR_AREA:
                continue
            x, y, w, h = cv2.boundingRect(cnt)
            # Offset coordinates back into full-frame space, since we
            # cropped before detecting - everything downstream (drawing,
            # ROI cropping for classification) expects full-frame coords
            x += roi_x1
            y += roi_y1
            centroids.append((x + w // 2, y + h // 2))
            boxes.append((x, y, w, h))
            confidences.append(1.0)  # contour detection has no confidence score - always 1.0

        tracked = tracker.update(centroids)
        frame_should_divert = False

        for oid, (cx, cy) in tracked.items():
            match_idx = min(
                range(len(boxes)),
                key=lambda i: (boxes[i][0]+boxes[i][2]//2-cx)**2 + (boxes[i][1]+boxes[i][3]//2-cy)**2,
                default=None
            )
            if match_idx is None:
                continue
            x, y, w, h = boxes[match_idx]
            conf = confidences[match_idx]
            tracker.record_position(oid, (cx, cy))

            if oid not in tracker.classified:
                roi_bgr = frame[y:y+h, x:x+w]
                classification, solidity, defect_count = classify_design(roi_bgr)

                tracker.classified[oid] = classification
                counts[classification] = counts.get(classification, 0) + 1
                print(f"[CLASSIFIED] ID{oid} -> {classification} "
                      f"(solidity={solidity:.2f}, defects={defect_count}, conf={conf:.2f})")

                # Sorting decision: divert the configured class, plus
                # anything unrecognized (safety default - don't let
                # unknown objects pass through as if they were fine)
                if classification == DIVERT_CLASS:
                    sort_result = "DIVERTED"
                elif classification == "UNKNOWN":
                    sort_result = "DIVERTED (unrecognized)"
                else:
                    sort_result = "PASSED THROUGH"

                log_writer.writerow([time.strftime("%Y-%m-%d %H:%M:%S"), oid,
                                      f"{conf:.2f}", classification, sort_result])
                log_file.flush()
            else:
                classification = tracker.classified[oid]  # already locked in from a previous frame

            will_divert = (classification == DIVERT_CLASS or classification == "UNKNOWN")
            if will_divert:
                frame_should_divert = True

            color = (0, 0, 255) if will_divert else (0, 255, 0)
            cv2.rectangle(frame, (x, y), (x+w, y+h), color, 2)
            sort_tag = "DIVERT" if will_divert else "PASS"
            cv2.putText(frame, f"ID{oid} {classification} [{sort_tag}] ({conf:.2f})", (x, y-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        vision_fault = False
        if plc_client is not None:
            try:
                plc_client.write_coil(COIL_VISION_REJECT_FLAG, frame_should_divert)
            except Exception as e:
                print(f"[MODBUS WRITE ERROR - REJECT_FLAG] {e}")

            now = time.time()
            if now - last_heartbeat_time > HEARTBEAT_INTERVAL:
                heartbeat_state = not heartbeat_state
                try:
                    plc_client.write_coil(COIL_VISION_HEARTBEAT, heartbeat_state)
                except Exception as e:
                    print(f"[MODBUS WRITE ERROR - HEARTBEAT] {e}")
                last_heartbeat_time = now

            try:
                result = plc_client.read_coils(COIL_VISION_FAULT_LAMP, count=1)
                if not result.isError():
                    vision_fault = result.bits[0]
            except Exception as e:
                print(f"[MODBUS READ ERROR - FAULT_LAMP] {e}")

        status = f"LID: {counts.get('LID',0)}  BASE: {counts.get('BASE',0)}  UNKNOWN: {counts.get('UNKNOWN',0)}  |  Diverting: {DIVERT_CLASS}"
        plc_status = "PLC: CONNECTED" if plc_client else "PLC: NOT CONNECTED"
        fault_status = "VISION FAULT (PLC-DETECTED)" if vision_fault else "VISION OK"

        cv2.putText(frame, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(frame, plc_status, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0, 255, 0) if plc_client else (0, 0, 255), 2)
        cv2.putText(frame, fault_status, (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0, 0, 255) if vision_fault else (0, 255, 0), 2)

        cv2.imshow("VisionSort - Lid/Base Classification", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()
    log_file.close()
    if plc_client is not None:
        plc_client.close()


if __name__ == "__main__":
    main()

