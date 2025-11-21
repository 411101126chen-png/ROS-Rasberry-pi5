# -*- coding: utf-8 -*-
"""ROS2 node wrapper for the follow.py vision implementation.

This file adapts the standalone follow code into a rclpy Node that
publishes geometry_msgs/Twist on /cmd_vel.
"""
import os
import math
import faulthandler
from collections import deque

faulthandler.enable()
os.environ["MEDIAPIPE_DISABLE_GPU"] = "1"
os.environ["GLOG_minloglevel"] = "2"
os.environ["TF_CPP_MIN_LOG_LEVEL"] = "2"

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import cv2
import numpy as np
try:
    import mediapipe as mp
    _HAS_MEDIAPIPE = True
except Exception:
    mp = None
    _HAS_MEDIAPIPE = False

# ======= parameters (kept from original follow.py) =======
FRAME_W, FRAME_H = 640, 480
MIN_CONTOUR_AREA_PX = 3000
SMOOTH_N = 7
OPEN_HAND_CONSEC = 3
MIN_LOCK_HOLD = 45
LOCK_TIMEOUT = 25
LOCK_RADIUS_RATIO = 0.18
POS_DIST_THR_RATIO = 0.25
HAND_LINK_MAX = 0.50
SEG_THRESH = 0.30
COLOR_HIGH = 0.72
COLOR_LOW = 0.58
COLOR_UPDATE_ALPHA = 0.08
H_BINS, S_BINS = 32, 32
TOGGLE_COOLDOWN = 20
RESUME_BOOST_FRAMES = 10
FONT = cv2.FONT_HERSHEY_SIMPLEX

if _HAS_MEDIAPIPE:
    mp_selfie = mp.solutions.selfie_segmentation
    mp_hands = mp.solutions.hands
else:
    mp_selfie = None
    mp_hands = None

def dist(p1, p2):
    return math.hypot(float(p1[0]-p2[0]), float(p1[1]-p2[1]))

def contour_center(cnt):
    M = cv2.moments(cnt)
    if M.get("m00", 0) == 0:
        return None
    return (int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"]))

def safe_resize_mask(mask, target_shape):
    H,W = target_shape[:2]
    if mask is None or mask.size == 0:
        return np.zeros((H,W), np.uint8)
    if mask.shape[:2] != (H,W):
        mask = cv2.resize(mask, (W,H), interpolation=cv2.INTER_NEAREST)
    return mask

def extract_person_color(frame, contour):
    try:
        if contour is None or len(contour) < 3:
            return None
        mask = np.zeros(frame.shape[:2], np.uint8)
        cv2.drawContours(mask, [contour.astype(np.int32)], -1, 255, -1)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        h,s,v = cv2.split(hsv)
        valid = (v > 40) & (v < 220)
        sel = (mask==255) & valid
        if np.count_nonzero(sel) < 80:
            return None
        mean_h = float(np.mean(h[sel]))
        mean_s = float(np.mean(s[sel]))
        hist = cv2.calcHist([hsv],[0,1],mask,[H_BINS,S_BINS],[0,180,0,256])
        if hist is None:
            return None
        hist = cv2.normalize(hist, hist).flatten()
        return (mean_h, mean_s, hist)
    except Exception:
        return None

def color_similarity(h1,s1,hs1,h2,s2,hs2):
    try:
        if hs1 is None or hs2 is None:
            return 0.0
        dh = min(abs(h1-h2), 180-abs(h1-h2)) / 180.0
        ds = abs(s1-s2) / 256.0
        hist_sim = cv2.compareHist(hs1, hs2, cv2.HISTCMP_BHATTACHARYYA)
        hist_sim = 1.0 - min(hist_sim, 1.0)
        sim = 0.3*(1-dh) + 0.2*(1-ds) + 0.5*hist_sim
        return max(0.0, min(1.0, sim))
    except Exception:
        return 0.0

def is_open_hand(lm):
    try:
        wrist = np.array([lm[0].x, lm[0].y])
        mid_mcp = np.array([lm[9].x, lm[9].y])
        hand_size = np.linalg.norm(mid_mcp - wrist) + 1e-6
        fingers = [(8,5), (12,9), (16,13), (20,17)]
        count = 0
        for tip_i, mcp_i in fingers:
            tip = np.array([lm[tip_i].x, lm[tip_i].y])
            mcp = np.array([lm[mcp_i].x, lm[mcp_i].y])
            if np.linalg.norm(tip - mcp) > 0.45 * hand_size:
                count += 1
        return count >= 3
    except Exception:
        return False

def is_fist(lm):
    try:
        wrist = np.array([lm[0].x, lm[0].y])
        mid_mcp = np.array([lm[9].x, lm[9].y])
        hand_size = np.linalg.norm(mid_mcp - wrist) + 1e-6
        fingers = [(8,5), (12,9), (16,13), (20,17)]
        extended = 0
        for tip_i, mcp_i in fingers:
            tip = np.array([lm[tip_i].x, lm[tip_i].y])
            mcp = np.array([lm[mcp_i].x, lm[mcp_i].y])
            if np.linalg.norm(tip - mcp) > 0.45 * hand_size:
                extended += 1
        return extended < 2
    except Exception:
        return False

class FollowProcessor:
    def __init__(self, camera_idx=0, width=FRAME_W, height=FRAME_H, no_gui=True, logger=None, debug=False, auto_lock=False):
        self.cap = None
        self.camera_idx = camera_idx
        self.width = width
        self.height = height
        self.no_gui = no_gui
        # optional logger passed from the ROS node and debug flag
        self.logger = logger
        self.debug = bool(debug)
        # if True, when mediapipe is present we will auto-lock to the largest
        # contour if no open-hand toggle is seen. This makes interactive tests
        # easier when you can't or don't want to use the hand toggle gesture.
        self.auto_lock = bool(auto_lock)
        # If mediapipe is not available, we will run in degraded mode: the
        # processor will still open the camera but won't run segmentation or
        # hand detection. This allows the ROS node to start on systems where
        # mediapipe cannot be installed.
        if _HAS_MEDIAPIPE:
            self.segmenter = mp_selfie.SelfieSegmentation(model_selection=1)
            self.hands = mp_hands.Hands(max_num_hands=1, min_detection_confidence=0.6, min_tracking_confidence=0.5)
        else:
            self.segmenter = None
            self.hands = None
            # fallback background subtractor for systems without mediapipe
            try:
                self.bg_sub = cv2.createBackgroundSubtractorMOG2(history=500, varThreshold=25, detectShadows=False)
            except Exception:
                self.bg_sub = None
        self.cmd_hist = deque(maxlen=SMOOTH_N)
        # state
        self.state = "STANDBY"
        self.lock_contour = None
        self.lock_color = (None,None,None)
        self.lock_age = 0
        self.miss = 0
        self.follow_active = False
        self.open_hand_prev = False
        self.open_hand_streak = 0
        self.toggle_cooldown = 0
        self.resume_boost = 0
        self.last_hand_lm = None

        self.open_camera()
        try:
            if self.logger is not None:
                self.logger.info(f"FollowProcessor: auto_lock={self.auto_lock}")
        except Exception:
            pass

    def open_camera(self):
        for idx in range(self.camera_idx, self.camera_idx+5):
            cap = cv2.VideoCapture(idx)
            if not cap.isOpened():
                try:
                    cap.release()
                except Exception:
                    pass
                continue
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self.cap = cap
            self.camera_idx = idx
            break
        if self.cap is None:
            # Do not raise here; allow the processor to run in degraded/no-camera mode.
            # This makes the ROS node usable on systems without an available camera
            # (it will publish zero velocities). The node/user can still supply a
            # different camera index via the 'camera' parameter.
            self.no_camera = True
            print("[WARN] Cannot open any camera. Running in no-camera degraded mode.")
        else:
            self.no_camera = False

    def close(self):
        try:
            if self.cap is not None:
                self.cap.release()
            cv2.destroyAllWindows()
        except Exception:
            pass

    def step(self):
        """讀一張影格並回傳一個字典 {'linear':..., 'angular':..., 'cmd': 'FORWARD'|'STOP'}"""
        # If no camera is available (self.cap is None) we are running in
        # degraded/no-camera mode — return a zero-velocity command instead
        # of raising an exception. This keeps the ROS node alive so other
        # components can still interact with it (and /cmd_vel stays published).
        if getattr(self, 'cap', None) is None or getattr(self, 'no_camera', False):
            return {'linear': 0.0, 'angular': 0.0, 'cmd': 'STOP'}

        ok, frame = self.cap.read()
        if not ok or frame is None:
            return {'linear':0.0, 'angular':0.0, 'cmd':'STOP'}
        frame = cv2.flip(frame, 1)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        H,W = frame.shape[:2]

        # segmentation
        try:
            if self.segmenter is not None:
                seg_result = self.segmenter.process(rgb)
                mask = seg_result.segmentation_mask
                mask = (mask > SEG_THRESH).astype(np.uint8) * 255
                mask = safe_resize_mask(mask, frame.shape)
            else:
                # fallback: background subtraction -> morphological cleanup
                if self.bg_sub is not None:
                    fg = self.bg_sub.apply(frame)
                    # remove shadows/noise
                    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
                    fg = cv2.morphologyEx(fg, cv2.MORPH_OPEN, k, iterations=1)
                    fg = cv2.morphologyEx(fg, cv2.MORPH_DILATE, k, iterations=2)
                    mask = safe_resize_mask(fg, frame.shape)
                else:
                    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                    _, mask = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY)
                    mask = safe_resize_mask(mask, frame.shape)
        except Exception:
            mask = np.zeros((H,W), np.uint8)

        try:
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            contours = [c for c in contours if cv2.contourArea(c) >= MIN_CONTOUR_AREA_PX]
        except Exception:
            contours = []

        # hands
        open_hand = False
        hand_center = None
        try:
            if self.hands is not None:
                hand_result = self.hands.process(rgb)
                if hand_result.multi_hand_landmarks and len(hand_result.multi_hand_landmarks) > 0:
                    lm = hand_result.multi_hand_landmarks[0].landmark
                    open_hand = is_open_hand(lm)
                    self.last_hand_lm = lm
                    wrist = lm[0]
                    hand_center = (int(wrist.x * W), int(wrist.y * H))
                else:
                    self.last_hand_lm = None
            else:
                # no mediapipe: we won't detect hands. use None and rely on contour-based auto-lock
                self.last_hand_lm = None
                open_hand = False
                hand_center = None
        except Exception:
            self.last_hand_lm = None

        self.open_hand_streak = self.open_hand_streak + 1 if open_hand else 0
        toggle_edge = ((not self.open_hand_prev) and open_hand and (self.open_hand_streak >= OPEN_HAND_CONSEC) and (self.toggle_cooldown==0))
        self.open_hand_prev = open_hand
        if self.toggle_cooldown > 0:
            self.toggle_cooldown -= 1
        toggle_ready = (self.open_hand_streak >= OPEN_HAND_CONSEC) and (self.toggle_cooldown == 0)

        cmd = "STOP"

        if self.state == "STANDBY":
            # Normal behavior with mediapipe: require toggle_ready and a nearby hand center
            if self.segmenter is not None:
                if toggle_ready and hand_center is not None and contours:
                    centers = []
                    dists_list = []
                    for cc_cnt in contours:
                        cc = contour_center(cc_cnt)
                        if cc is None:
                            continue
                        centers.append(cc)
                        dists_list.append(dist(hand_center, cc))
                    if centers and min(dists_list) <= HAND_LINK_MAX * W:
                        best_idx = int(np.argmin(dists_list))
                        self.lock_contour = contours[best_idx]
                        self.lock_color = extract_person_color(frame, self.lock_contour)
                        self.lock_age = 0
                        self.miss = 0
                        self.state = "LOCKED"
                        self.follow_active = True
                        self.toggle_cooldown = TOGGLE_COOLDOWN
                        self.resume_boost = RESUME_BOOST_FRAMES
                        cmd = "FORWARD"
                # optional auto-lock (when enabled via node parameter) — if
                # mediapipe is available but the user didn't perform the
                # open-hand toggle, allow locking to the largest contour so
                # testing/standalone follow still works.
                elif self.auto_lock and contours:
                    # auto-lock: pick largest contour regardless of area so
                    # tests work even when MIN_CONTOUR_AREA_PX is too strict
                    areas = [cv2.contourArea(c) for c in contours]
                    best_idx = int(np.argmax(areas))
                    self.lock_contour = contours[best_idx]
                    self.lock_color = extract_person_color(frame, self.lock_contour)
                    self.lock_age = 0
                    self.miss = 0
                    self.state = "LOCKED"
                    self.follow_active = True
                    self.toggle_cooldown = TOGGLE_COOLDOWN
                    self.resume_boost = RESUME_BOOST_FRAMES
                    cmd = "FORWARD"
                    try:
                        if self.logger is not None:
                            self.logger.info("auto_lock: locked to largest contour")
                    except Exception:
                        pass
            else:
                # Fallback auto-lock: lock to the largest contour (if any)
                if contours:
                    areas = [cv2.contourArea(c) for c in contours]
                    best_idx = int(np.argmax(areas))
                    if areas[best_idx] >= MIN_CONTOUR_AREA_PX:
                        self.lock_contour = contours[best_idx]
                        self.lock_color = (None, None, None)
                        self.lock_age = 0
                        self.miss = 0
                        self.state = "LOCKED"
                        self.follow_active = True
                        cmd = "FORWARD"
        else:
            if self.lock_contour is not None:
                if self.lock_age < MIN_LOCK_HOLD:
                    search_radius = LOCK_RADIUS_RATIO * W
                else:
                    search_radius = POS_DIST_THR_RATIO * W
                best_cnt = None
                best_sim = -1.0
                lock_center = contour_center(self.lock_contour)
                if lock_center is not None:
                    for cnt in contours:
                        cc = contour_center(cnt)
                        if cc is None:
                            continue
                        d = dist(lock_center, cc)
                        if d > search_radius:
                            continue
                        color_data = extract_person_color(frame, cnt)
                        if color_data is None:
                            sim = 0.0
                        else:
                            h2,s2,hs2 = color_data
                            lh,ls,lhs = self.lock_color
                            sim = color_similarity(lh, ls, lhs, h2, s2, hs2)
                        threshold = COLOR_HIGH if self.lock_age < 5 else COLOR_LOW
                        if sim >= threshold and sim > best_sim:
                            best_sim = sim
                            best_cnt = cnt
                if best_cnt is not None:
                    self.lock_contour = best_cnt
                    self.lock_color = extract_person_color(frame, best_cnt)
                    self.lock_age += 1
                    self.miss = 0
                else:
                    self.miss += 1
                    self.lock_age += 1
                    if self.miss >= LOCK_TIMEOUT:
                        self.state = "STANDBY"
                        self.lock_contour = None
                        self.lock_color = (None,None,None)
                        self.lock_age = 0
                        self.follow_active = False
                        cmd = "STOP"
            if self.follow_active and self.lock_contour is not None:
                if self.last_hand_lm is not None and is_fist(self.last_hand_lm):
                    self.state = "STANDBY"
                    self.lock_contour = None
                    self.lock_color = (None,None,None)
                    self.lock_age = 0
                    self.miss = 0
                    self.follow_active = False
                    cmd = "STOP"
                elif open_hand:
                    cmd = "FORWARD"
                else:
                    cmd = "STOP"

        # optional GUI for debugging
        if not self.no_gui:
            try:
                if self.lock_contour is not None:
                    cv2.drawContours(frame, [self.lock_contour.astype(np.int32)], 0, (0,255,0), 2)
                    cc = contour_center(self.lock_contour)
                    if cc is not None:
                        cv2.circle(frame, cc, 5, (0,255,0), -1)
                        cv2.putText(frame, f"STATE:{self.state} AGE:{self.lock_age} MISS:{self.miss}", (10,40), FONT, 0.6, (0,255,0), 1)
                if hand_center is not None:
                    cv2.circle(frame, hand_center, 8, (255,0,0), 2)
                cx_mid = W//2
                cv2.line(frame,(cx_mid,0),(cx_mid,H),(120,120,120),1)
                cv2.putText(frame, f"CMD: {cmd}", (10,80), FONT, 0.9, (255,255,255), 2)
                cv2.putText(frame, f"FOLLOW: {'ON' if self.follow_active else 'PAUSED'}", (10,120), FONT, 0.7, ((0,255,0) if self.follow_active else (0,255,255)), 2)
                cv2.imshow("Follow", frame)
                cv2.waitKey(1)
            except Exception:
                pass

        # map cmd to velocities (adjust to your robot)
        linear = 0.2 if cmd == "FORWARD" else 0.0
        angular = 0.0
        # optional debug logging (via ROS logger if available)
        if getattr(self, 'debug', False):
            msg = f"DEBUG: cmd={cmd} follow_active={self.follow_active} lock_age={self.lock_age} contours={len(contours)} miss={self.miss}"
            try:
                if self.logger is not None:
                    self.logger.info(msg)
                else:
                    print(msg)
            except Exception:
                # be robust to logging failures
                pass

        return {'linear': linear, 'angular': angular, 'cmd': cmd}

class FollowNode(Node):
    def __init__(self):
        super().__init__('follow_node')
        self.declare_parameter('camera', 0)
        self.declare_parameter('no_gui', True)
        self.declare_parameter('debug', False)
        self.declare_parameter('auto_lock', False)
        self.declare_parameter('rate', 20.0)
        cam = int(self.get_parameter('camera').value)
        no_gui = bool(self.get_parameter('no_gui').value)
        debug = bool(self.get_parameter('debug').value)
        auto_lock = bool(self.get_parameter('auto_lock').value)
        rate = float(self.get_parameter('rate').value)

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.processor = FollowProcessor(camera_idx=cam, no_gui=no_gui, logger=self.get_logger(), debug=debug, auto_lock=auto_lock)
        self.timer = self.create_timer(1.0/float(rate), self.timer_cb)
        self.get_logger().info(f"follow_node started (camera={self.processor.camera_idx}, no_gui={no_gui}, rate={rate})")

    def timer_cb(self):
        try:
            out = self.processor.step()
            twist = Twist()
            twist.linear.x = float(out.get('linear', 0.0))
            twist.angular.z = float(out.get('angular', 0.0))
            try:
                self.pub.publish(twist)
            except Exception as e:
                self.get_logger().error(f"publish failed in timer_cb: {e}")
        except Exception as e:
            import traceback
            self.get_logger().error(f"Exception in timer_cb: {e}")
            tb = traceback.format_exc()
            for line in tb.splitlines():
                self.get_logger().error(line)

    def destroy_node(self):
        try:
            self.processor.close()
        except Exception:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = FollowNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
