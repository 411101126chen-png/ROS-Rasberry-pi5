# -*- coding: utf-8 -*-
# follow.py — Toggle Follow (gesture) + Person mask contour + Color match
# Hardened + HUD + Fallback + Level-trigger lock in STANDBY
import argparse
import sys
import cv2
import math
import os
import faulthandler

# 對外部套件做友善提示（在某些電腦上 pip install mediapipe 可能會失敗）
try:
    import mediapipe as mp
except Exception as e:
    print("[ERROR] Failed to import 'mediapipe'.", file=sys.stderr)
    print("        Please install it in your environment, :", file=sys.stderr)
    print("        pip install mediapipe", file=sys.stderr)
    print("        If installation fails on Windows, try a compatible Python version or see Mediapipe docs.", file=sys.stderr)
    raise

try:
    import numpy as np
except Exception as e:
    print("[ERROR] Failed to import 'numpy'. Please install: pip install numpy", file=sys.stderr)
    raise

faulthandler.enable()
os.environ["MEDIAPIPE_DISABLE_GPU"] = "1"
os.environ["GLOG_minloglevel"] = "2"
os.environ["TF_CPP_MIN_LOG_LEVEL"] = "2"
from collections import deque

print("⚙️ opencv:", cv2.__version__)

# ========================= MediaPipe =========================
mp_selfie = mp.solutions.selfie_segmentation
mp_hands  = mp.solutions.hands
segmenter = mp_selfie.SelfieSegmentation(model_selection=1)
hands = mp_hands.Hands(max_num_hands=1, min_detection_confidence=0.6, min_tracking_confidence=0.5)
draw  = mp.solutions.drawing_utils

# ========================= 參數（可調） =========================
FRAME_W, FRAME_H = 640, 480
MIN_CONTOUR_AREA_PX = 3000

# 指令輸出平滑
SMOOTH_N    = 7

# 鎖定/追蹤穩定化
OPEN_HAND_CONSEC   = 3     # 連續幀「張手」才觸發（STANDBY 使用等級觸發）
MIN_LOCK_HOLD      = 45    # 鎖定後至少維持幾幀不換人
LOCK_TIMEOUT       = 25    # 連續失配幀數 → 解鎖
LOCK_RADIUS_RATIO  = 0.18  # 鎖定初期搜尋半徑（相對畫面寬）
POS_DIST_THR_RATIO = 0.25  # 穩定後搜尋半徑（相對畫面寬）
HAND_LINK_MAX      = 0.50  # 啟動時：手中心到人輪廓中心距離上限（相對畫面寬，先放寬）

# 人形分割閥值（越小越容易被判為人）
SEG_THRESH = 0.30

# 顏色相似度遲滯 + 緩慢更新
COLOR_HIGH         = 0.72  # 新進/切換需要 >= 這個值
COLOR_LOW          = 0.58  # 已鎖定時只要 >= 這個值就算同人
COLOR_UPDATE_ALPHA = 0.08  # 顏色模型低速自適應
H_BINS, S_BINS     = 32, 32
FONT = cv2.FONT_HERSHEY_SIMPLEX

# 手勢切換（跟隨/暫停）參數
TOGGLE_COOLDOWN = 20      # 開掌切換後的冷卻幀數（避免一次觸發多次）
RESUME_BOOST_FRAMES = 10  # 從暫停恢復時，強制 FORWARD 幾幀（幫設備啟動）

# ========================= 小工具 =========================
def dist(p1, p2): 
    return math.hypot(float(p1[0]-p2[0]), float(p1[1]-p2[1]))

def majority(v):
    if not v: return "STOP"
    return max(set(v), key=v.count)

def is_extended(lm, tip, pip): 
    return lm[tip].y < lm[pip].y  # y 小在上 → 伸直

def is_open_hand(lm):
    """
    更穩定的開掌判斷：
    - 以手腕到中指 MCP 的距離作為手部尺度
    - 若指尖到對應 MCP 的距離超過尺度的比例則視為該指伸直
    這樣對手部旋轉（例如手背朝鏡頭、手朝下）比較穩定，不會誤判
    """
    try:
        # 使用 normalized (x,y) 座標
        wrist = np.array([lm[0].x, lm[0].y])
        mid_mcp = np.array([lm[9].x, lm[9].y])
        hand_size = np.linalg.norm(mid_mcp - wrist) + 1e-6

        fingers = [(8,5), (12,9), (16,13), (20,17)]  # (tip, mcp)
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
    """
    Detect a fist: count how many fingers are extended (tip-to-mcp distance relative to hand size).
    If fewer than 2 fingers extended, consider it a fist.
    """
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

def safe_resize_mask(mask, target_shape):
    H,W = target_shape[:2]
    if mask is None or mask.size == 0: 
        return np.zeros((H,W), np.uint8)
    if mask.shape[:2] != (H,W):
        mask = cv2.resize(mask, (W,H), interpolation=cv2.INTER_NEAREST)
    return mask

def contour_center(cnt):
    M = cv2.moments(cnt)
    if M["m00"] == 0: 
        return None
    return (int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"]))

def extract_person_color(frame, contour):
    try:
        if contour is None or len(contour) < 3: 
            return None
        mask = np.zeros(frame.shape[:2], np.uint8)
        cv2.drawContours(mask, [contour.astype(np.int32)], -1, 255, -1)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        h,s,v = cv2.split(hsv)
        valid = (v > 40) & (v < 220)  # 排除過暗/過亮
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
        # Hue 循環距離
        dh = min(abs(h1-h2), 180-abs(h1-h2)) / 180.0
        ds = abs(s1-s2) / 256.0
        # 直方圖相似度 (Bhattacharyya)
        hist_sim = cv2.compareHist(hs1, hs2, cv2.HISTCMP_BHATTACHARYYA)
        hist_sim = 1.0 - min(hist_sim, 1.0)
        # 加權組合
        sim = 0.3*(1-dh) + 0.2*(1-ds) + 0.5*hist_sim
        return max(0.0, min(1.0, sim))
    except Exception:
        return 0.0

# ========================= 主程式 =========================
def parse_args():
    p = argparse.ArgumentParser(description="Follow gesture vision node (standalone)")
    p.add_argument("--camera", type=int, default=0, help="camera index (default: 0)")
    p.add_argument("--no-gui", action="store_true", help="run without opening OpenCV windows")
    p.add_argument("--log", type=str, default=None, help="write combined stdout/stderr to this file")
    return p.parse_args()


def open_camera_with_fallback(start_idx=0, max_try=4, width=FRAME_W, height=FRAME_H):
    # 嘗試多個 camera index，回傳已開啟的 VideoCapture 與實際 index
    for idx in range(start_idx, start_idx + max_try + 1):
        cap = cv2.VideoCapture(idx)
        if not cap.isOpened():
            cap.release()
            continue
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        return cap, idx
    return None, None


def main():
    args = parse_args()
    if args.log:
        try:
            logf = open(args.log, "a", encoding="utf-8")
            # 將 stderr/stdout 同時導向檔案（但保留原有 terminal 輸出）
            class TeeStream:
                def __init__(self, orig, f):
                    self.orig = orig
                    self.f = f
                def write(self, data):
                    try:
                        self.orig.write(data)
                    except Exception:
                        pass
                    try:
                        self.f.write(data)
                        self.f.flush()
                    except Exception:
                        pass
                def flush(self):
                    try:
                        self.orig.flush()
                    except Exception:
                        pass
                    try:
                        self.f.flush()
                    except Exception:
                        pass
            sys.stdout = TeeStream(sys.stdout, logf)
            sys.stderr = TeeStream(sys.stderr, logf)
        except Exception as e:
            print(f"[WARN] cannot open log file {args.log}: {e}", file=sys.stderr)

    print("✅ Loaded: Toggle Follow (gesture) + Mask + Color (Hardened + HUD)")
    cap, real_idx = open_camera_with_fallback(args.camera, max_try=4, width=FRAME_W, height=FRAME_H)
    if cap is None or not cap.isOpened():
        raise RuntimeError("Camera open failed. 請關閉會佔用相機的程式（相機App/Zoom/Teams/Discord/OBS）再試，或指定 --camera INDEX。")
    else:
        print(f"Camera opened (index={real_idx})")
    # 將 cap 與 NO_GUI 設為全域以供主迴圈使用
    globals()['cap'] = cap
    globals()['NO_GUI'] = args.no_gui


# 初始化（設定 camera, 日誌檔等）
if __name__ == '__main__':
    try:
        args = parse_args()
        if args.log:
            try:
                logf = open(args.log, "a", encoding="utf-8")
                class TeeStream:
                    def __init__(self, orig, f):
                        self.orig = orig
                        self.f = f
                    def write(self, data):
                        try:
                            self.orig.write(data)
                        except Exception:
                            pass
                        try:
                            self.f.write(data)
                            self.f.flush()
                        except Exception:
                            pass
                    def flush(self):
                        try:
                            self.orig.flush()
                        except Exception:
                            pass
                        try:
                            self.f.flush()
                        except Exception:
                            pass
                sys.stdout = TeeStream(sys.stdout, logf)
                sys.stderr = TeeStream(sys.stderr, logf)
            except Exception as e:
                print(f"[WARN] cannot open log file {args.log}: {e}", file=sys.stderr)

        print("✅ Loaded: Toggle Follow (gesture) + Mask + Color (Hardened + HUD)")
        cap, real_idx = open_camera_with_fallback(args.camera, max_try=4, width=FRAME_W, height=FRAME_H)
        if cap is None or not cap.isOpened():
            raise RuntimeError("Camera open failed. 請關閉會佔用相機的程式（相機App/Zoom/Teams/Discord/OBS）再試，或指定 --camera INDEX。")
        else:
            print(f"Camera opened (index={real_idx})")
        
        NO_GUI = args.no_gui

        # ========================= 狀態初始化 =========================
        state = "STANDBY"
        lock_contour = None
        lock_color   = (None,None,None)
        lock_age     = 0
        miss         = 0
        cmd_hist = deque(maxlen=SMOOTH_N)

        follow_active = False
        open_hand_prev = False
        open_hand_streak = 0
        toggle_cooldown = 0
        resume_boost = 0
        last_hand_lm = None

        # 增加狀態穩定機制
        STABLE_STATE_DELAY = 10  # 狀態穩定延遲幀數
        stable_state_counter = 0

        # ========================= 主迴圈 =========================
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            frame = cv2.flip(frame, 1)
            rgb   = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            H, W  = frame.shape[:2]

            # ---- segmentation → mask
            try:
                seg_result = segmenter.process(rgb)
                mask = seg_result.segmentation_mask
                mask = (mask > SEG_THRESH).astype(np.uint8) * 255
                mask = safe_resize_mask(mask, frame.shape)
            except Exception:
                mask = np.zeros((H,W), np.uint8)

            # ---- contours
            try:
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                contours = [c for c in contours if cv2.contourArea(c) >= MIN_CONTOUR_AREA_PX]
            except Exception:
                contours = []

            # ---- hands (only for lock/toggle)
            open_hand = False
            hand_center = None
            try:
                hand_result = hands.process(rgb)
                if hand_result.multi_hand_landmarks and len(hand_result.multi_hand_landmarks) > 0:
                    lm = hand_result.multi_hand_landmarks[0].landmark
                    open_hand = is_open_hand(lm)
                    last_hand_lm = lm
                    # 手掌中心 (鎖定點)
                    wrist = lm[0]
                    hand_center = (int(wrist.x * W), int(wrist.y * H))
                else:
                    last_hand_lm = None
            except Exception:
                last_hand_lm = None

            # ---- 開掌連續幀數 + 冷卻 + 觸發旗標
            open_hand_streak = open_hand_streak + 1 if open_hand else 0
            toggle_edge = ((not open_hand_prev) and open_hand and
                           (open_hand_streak >= OPEN_HAND_CONSEC) and
                           (toggle_cooldown == 0))          # 邊緣觸發（LOCKED 用）
            open_hand_prev = open_hand
            if toggle_cooldown > 0:
                toggle_cooldown -= 1
            toggle_ready = (open_hand_streak >= OPEN_HAND_CONSEC) and (toggle_cooldown == 0)  # 等級觸發（STANDBY 用）

            # 手勢偵測 HUD
            cv2.putText(frame, f"HAND_OPEN:{int(open_hand)}  STREAK:{open_hand_streak}  COOLDOWN:{toggle_cooldown}",
                        (10, 160), FONT, 0.6, (0,255,255), 2)

            cmd = "STOP"

            if state == "STANDBY":
                # 等級觸發模式：只要持續開掌就嘗試鎖定
                if toggle_ready and hand_center is not None and contours:
                    # 找離手最近的輪廓，且距離不超過 HAND_LINK_MAX
                    centers = []
                    dists_list = []
                    for cc_cnt in contours:
                        cc = contour_center(cc_cnt)
                        if cc is None: 
                            continue
                        centers.append(cc)
                        dists_list.append(dist(hand_center, cc))
                    
                    if centers and min(dists_list) <= HAND_LINK_MAX * W:
                        best_idx = np.argmin(dists_list)
                        lock_contour = contours[best_idx]
                        lock_color = extract_person_color(frame, lock_contour)
                        lock_age = 0
                        miss = 0
                        state = "LOCKED"
                        follow_active = True
                        toggle_cooldown = TOGGLE_COOLDOWN
                        resume_boost = RESUME_BOOST_FRAMES
                        cmd = "FORWARD"
                        stable_state_counter = 0
                else:
                    stable_state_counter = 0
            else:  # LOCKED 模式
                if lock_contour is not None:
                    # 計算搜尋半徑
                    if lock_age < MIN_LOCK_HOLD:
                        search_radius = LOCK_RADIUS_RATIO * W
                    else:
                        search_radius = POS_DIST_THR_RATIO * W
                    
                    # 在搜尋半徑內尋找最匹配的輪廓
                    best_cnt = None
                    best_sim = -1.0
                    lock_center = contour_center(lock_contour)
                    
                    if lock_center is not None:
                        for cnt in contours:
                            cc = contour_center(cnt)
                            if cc is None: 
                                continue
                            d = dist(lock_center, cc)
                            if d > search_radius: 
                                continue
                            
                            # 顏色比對
                            color_data = extract_person_color(frame, cnt)
                            if color_data is None:
                                sim = 0.0
                            else:
                                h2, s2, hs2 = color_data
                                lh, ls, lhs = lock_color
                                sim = color_similarity(lh, ls, lhs, h2, s2, hs2)
                            
                            # 新進/切換需要 COLOR_HIGH，已鎖定只要 COLOR_LOW
                            threshold = COLOR_HIGH if lock_age < 5 else COLOR_LOW
                            if sim >= threshold and sim > best_sim:
                                best_sim = sim
                                best_cnt = cnt
                    
                    # 更新或解鎖
                    if best_cnt is not None:
                        lock_contour = best_cnt
                        lock_color = extract_person_color(frame, best_cnt)
                        lock_age += 1
                        miss = 0
                    else:
                        miss += 1
                        lock_age += 1
                        if miss >= LOCK_TIMEOUT:
                            state = "STANDBY"
                            lock_contour = None
                            lock_color = (None, None, None)
                            lock_age = 0
                            follow_active = False
                            cmd = "STOP"
                
                # 生成命令：基於手勢狀態
                # 開掌 → FORWARD，握拳 → STANDBY（解鎖）
                if follow_active and lock_contour is not None:
                    # 若偵測到完整的握拳 (fist)，則轉回 STANDBY（解鎖）
                    if last_hand_lm is not None and is_fist(last_hand_lm):
                        state = "STANDBY"
                        lock_contour = None
                        lock_color = (None, None, None)
                        lock_age = 0
                        miss = 0
                        follow_active = False
                        cmd = "STOP"
                    elif open_hand:
                        # 開掌持續往前
                        cmd = "FORWARD"
                    else:
                        cmd = "STOP"

            # 繪製輪廓和鎖定指示器
            try:
                if lock_contour is not None:
                    cv2.drawContours(frame, [lock_contour.astype(np.int32)], 0, (0,255,0), 2)
                    cc = contour_center(lock_contour)
                    if cc is not None:
                        cv2.circle(frame, cc, 5, (0,255,0), -1)
                        cv2.putText(frame, f"STATE:{state} AGE:{lock_age} MISS:{miss}", 
                                    (10,40), FONT, 0.6, (0,255,0), 1)
                
                if hand_center is not None:
                    cv2.circle(frame, hand_center, 8, (255,0,0), 2)
            except Exception:
                pass

            # 參考線 & CMD 顯示
            cx_mid = W//2
            cv2.line(frame,(cx_mid,0),(cx_mid,H),(120,120,120),1)
            cv2.putText(frame, f"CMD: {cmd}", (10,80), FONT, 0.9, (255,255,255), 2)
            cv2.putText(frame, f"FOLLOW: {'ON' if follow_active else 'PAUSED'}", (10,120),
                        FONT, 0.7, ((0,255,0) if follow_active else (0,255,255)), 2)
            cv2.putText(frame, "ESC=quit  R=unlock  G=toggle/lock (fallback)", (10,106), FONT, 0.5, (200,200,200), 1)

            try:
                if not NO_GUI:
                    cv2.imshow("Follow", frame)
            except Exception:
                pass

            key = cv2.waitKey(1) & 0xFF
            if key == 27: 
                break
            if key in (ord('r'), ord('R')):
                state = "STANDBY"
                lock_contour = None
                lock_color = (None, None, None)
                lock_age = 0
                miss = 0
                follow_active = False
            if key in (ord('g'), ord('G')):
                follow_active = not follow_active
                toggle_cooldown = TOGGLE_COOLDOWN

        cap.release()
        cv2.destroyAllWindows()
        print("✅ Clean exit.")
    
    except Exception as e:
        print(f"[FATAL] Error during execution: {e}", file=sys.stderr)
        import traceback
        traceback.print_exc()
        if cap is not None:
            cap.release()
        cv2.destroyAllWindows()
        raise
