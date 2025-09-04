
# projectPT_all_in_one_gui.py
import PySimpleGUI as sg
import serial
import threading
import time
import sys
import os
import math
# WITH this (must be *before* importing pyplot):
import matplotlib
# Force a GUI backend so plt.show() opens a window on Windows/PySimpleGUI
try:
    matplotlib.use("TkAgg")
except Exception:
    pass
import matplotlib.pyplot as plt
import numpy as np

# ── Modern theme (colors only; no logic changes) ────────────────────────────────
# Palette inspired by Tailwind slate/indigo
sg.theme_add_new("NeoRadar", {
    "BACKGROUND":            "#0b1220",
    "TEXT":                  "#e5e9f0",
    "INPUT":                 "#111827",
    "TEXT_INPUT":            "#e5e9f0",
    "SCROLL":                "#64748b",
    "BUTTON":                ("#ffffff", "#2563eb"),  # fg, bg
    "PROGRESS":              ("#2563eb", "#111827"),
    "BORDER":                0,
    "SLIDER_DEPTH":          0,
    "PROGRESS_DEPTH":        0,
})
sg.theme("NeoRadar")

# Global UI options (apply to all windows)
try:
    sg.set_options(
        font=("Segoe UI", 11),
        element_padding=(8, 8),
        margins=(14, 14),
        input_elements_background_color="#111827",
        text_element_background_color="#0b1220",
        input_text_color="#e5e9f0",
        button_color=("#ffffff", "#2563eb"),
        border_width=0,
    )
    # ttk buttons look a bit cleaner on Tk
    sg.set_options(use_ttk_buttons=True)
except Exception:
    pass

# Quick helpers for consistent accents (optional)
ACCENT_BG   = "#2563eb"  # primary
ACCENT_TXT  = "#ffffff"
SECOND_BG   = "#1f2937"  # secondary/ghost
WARN_BG     = "#ef4444"  # danger
def primary_btn(text, key=None, **kw):
    return sg.Button(text, key=key, button_color=(ACCENT_TXT, ACCENT_BG), border_width=0, **kw)
def secondary_btn(text, key=None, **kw):
    return sg.Button(text, key=key, button_color=("#e5e9f0", SECOND_BG), border_width=0, **kw)
def danger_btn(text, key=None, **kw):
    return sg.Button(text, key=key, button_color=("#ffffff", WARN_BG), border_width=0, **kw)

PORT = "COM7"
BAUD = 9600
# ---- MCU commands for LDR mode ----
LDR_ENTER_CMD = b"3"   # enter LDR / light-detector state (matches your state3)
LDR_START_CMD = b"l"   # optional start trigger; ignored by MCU if not needed

ANG_OFFSET_DEG = +7.0
# --- LDR distance calibration (PC-side) ---
LDR_DIST_SCALE  = 0.952   # ~ 40/42  → מכווץ ~5% כדי להפוך 42 ס"מ ל-~40
LDR_DIST_OFFSET = 0.0     # השאר 0 אלא אם תראה היסט קבוע

# ---- Calibration knobs (NEW) ----
BEAM_DEG_COMP  = 3.0    # מפחית מרוחב זוויתי לפני חישוב רוחב
WIDTH_SCALE    = 0.85   # כיווץ/הרחבת הרוחב הסופי
SECTOR_LEFT_MAX = 80.0
SECTOR_RIGHT_MIN  = 100.0
Dist_Samp = []
Angle_Samp = []
# ---- Light-picking params (NEW) ----
LIGHT_MAX_SOURCES     = 3     # נגלה עד 3 מקורות אור
LIGHT_MIN_SEP_DEG     = 12.0  # מרחק זוויתי מינימלי בין מקורות שנשאיר
LIGHT_SIDE_GUARD_DEG  = 6.0   # מרחק "שוליים" מהאמצע כדי להפריד שמאל/ימין
LIGHT_CENTER_BAND_DEG = 12.0  # רוחב "אמצע" שבו מותר לזהות מקור שלישי
LIGHT_BASE_PCT        = 82    # פרצנטיל בסיס לאמנה (רגיש יותר מאשר 85)
LIGHT_MIN_DROP_CM     = 4.0   # ירידה מינימלית מהבסיס כדי להיחשב כמקור
LIGHT_SMOOTH_W        = 7     # החלקה קלה לפני גילוי


LDR_Samp = []
LDR_Angle = []
# ---- Angle mapping for half-scan (NEW) ----
ANG_DST0_DEG = 90.0   # map min raw angle to 90°
ANG_DST1_DEG = 180.0  # map max raw angle to 180°
ANG_MIRROR   = False  # set True אם שמאל/ימין הפוכים פיזית
#ANG_OFFSET_DEG =+ 7.0

# ---- Angle mapping (NEW) ----
#ANG_DST0_DEG = 90.0     # לאן למפות את הזווית הגולמית המינימלית (אצלך ≈0° → 90°)
#ANG_DST1_DEG = 180.0    # לאן למפות את הזווית הגולמית המקסימלית (אצלך ≈88° → 180°)
#ANG_MIRROR   = False    # הפוך צדדים אם צריך (מראה סביב 90°)
# חשוב: עדכן היסט קבוע כך שהפיק יישב על המציאות שלך:
#ANG_OFFSET_DEG = -12.0  # 56° → ≈135° (ניתן לכיול דק)

stop_event = threading.Event()
current_mode = None

print_pending_dist = False
print_pending_ldr = False

SCRIPT_CACHE = os.path.join(os.path.expanduser("~"), ".mcu_scripts_cache.txt")
Local_Scripts = []

def load_local_scripts():
    try:
        with open(SCRIPT_CACHE, "r", encoding="utf-8") as f:
            return [ln.strip() for ln in f if ln.strip()]
    except Exception:
        return []

def save_local_scripts():
    try:
        with open(SCRIPT_CACHE, "w", encoding="utf-8") as f:
            for n in Local_Scripts:
                f.write(n + "\n")
    except Exception:
        pass

Local_Scripts = load_local_scripts()

def to_cm(t_diff):
    return (t_diff // 58) if t_diff < 400 * 58 else 0

def read_exact(ser, n, max_wait_s=1.0):
    deadline = time.time() + max_wait_s
    buf = bytearray()
    while len(buf) < n and not stop_event.is_set():
        if time.time() > deadline and len(buf) > 0:
            break
        chunk = ser.read(n - len(buf))
        if chunk:
            buf.extend(chunk)
        else:
            time.sleep(0.001)
    return bytes(buf)

def read_scan_header(ser, timeout_s=3.0, log_cb=print, dbg_tag=""):
    """
    Wait for 0xFF then read u16 little-endian sample count.
    Returns int count, or None on timeout/error.
    """
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        b = ser.read(1)
        if not b:
            continue
        if b == b"\xFF":
            cnt_b = read_exact(ser, 2, max_wait_s=1.0)
            if len(cnt_b) != 2:
                log_cb(f"[{dbg_tag}] Missing sample-count bytes after 0xFF.")
                return None
            return (cnt_b[0] | (cnt_b[1] << 8))
    log_cb(f"[{dbg_tag}] Timeout waiting for 0xFF header.")
    return None
# --- NEW: 9_23 30_8 ---
def send_post_mode(ser):
    """
    Send '0' immediately so the MCU returns to its idle post-mode.
    If you REALLY meant a NUL byte instead, change b"0" to b"\\x00".
    """
    try:
        if ser and getattr(ser, "is_open", False):
            ser.reset_input_buffer()
            ser.write(b"0")   # <-- ASCII '0' (0x30). Change to b"\x00" if your MCU expects NUL.
            ser.flush()
    except Exception as e:
        try:
            print(f"[TX post-mode error] {e}")
        except Exception:
            pass

#  single place to cleanly exit any MCU state and local flags ---
def ensure_idle(ser, settle_s=0.05):
    """
    Puts MCU back to idle (ASCII '0'), flushes buffers, and clears local state.
    """
    global current_mode, print_pending_dist, print_pending_ldr
    try:
        if ser and getattr(ser, "is_open", False):
            # flush anything we might still be transmitting
            ser.reset_output_buffer()
            # send '0' to MCU (this already resets input before TX)
            send_post_mode(ser)
            # let the MCU consume it, then drop anything left from the old state
            time.sleep(settle_s)
            ser.reset_input_buffer()
    except Exception:
        pass

    # local state reset
    current_mode = None
    print_pending_dist = False
    print_pending_ldr = False
    try:
        Dist_Samp.clear(); Angle_Samp.clear(); LDR_Samp.clear(); LDR_Angle.clear()
    except Exception:
        pass

def parse_frame3(frame_bytes):
    if len(frame_bytes) != 3:
        return None, None
    a = frame_bytes[0]
    v = frame_bytes[1] | (frame_bytes[2] << 8)
    if a > 180:
        a = 180
    return v, a

# ===== Calibration (PC-side LUTs) =====
CAL_POINTS = 10
LUT_SIZE = 50
cal1 = [0]*CAL_POINTS
cal2 = [0]*CAL_POINTS
lut1 = [0]*LUT_SIZE
lut2 = [0]*LUT_SIZE
lut_ready = False
use_lut_flag = True
lut_sensor_sel = 2

def build_lut(cal):
    lut = [0]*LUT_SIZE
    for d in range(LUT_SIZE):
        if d < 5:
            lut[d] = cal[0]
        elif d >= 45:
            # אקסטרפולציה לינארית לפי המקטע האחרון (40→45 ס"מ)
            #a = int(cal[-2])      # 40 ס"מ
            #b = int(cal[-1])      # 45 ס"מ
            #r = d - 45            # 0..4
            #v = (b - a) * r
            #q = (v + 2)//5 if v >= 0 else (v - 2)//5
            #lut[d] = b + q
            lut[d] = int(cal[-1])
        else:
            k = d // 5
            r = d % 5
            a = int(cal[k])
            b = int(cal[k+1])
            v = (b - a) * r
            q = (v + 2)//5 if v >= 0 else (v - 2)//5
            lut[d] = a + q
    return lut


def adc_to_cm(adc, lut):
    best_d = 0
    best_err = 1 << 30
    for d in range(LUT_SIZE):
        e = abs(int(lut[d]) - int(adc))
        if e < best_err:
            best_err = e
            best_d = d
    return best_d

def fetch_calibration_from_mcu(ser, log_print=None, verbose=False):
    global cal1, cal2, lut1, lut2, lut_ready, use_lut_flag
    if not ser:
        if verbose and log_print: log_print("[CAL] Not connected.")
        return False
    try:
        ser.reset_input_buffer()
        ser.write(b"K"); ser.flush()
        hdr = read_exact(ser, 1, 1.0)
        if hdr != b"K":
            if verbose and log_print: log_print("[CAL] No 'K' echo from MCU.")
            return False

        cnt_b = read_exact(ser, 1, 1.0)
        if len(cnt_b) != 1:
            if verbose and log_print: log_print("[CAL] Missing count byte.")
            return False
        cnt = cnt_b[0]
        if cnt != CAL_POINTS and verbose and log_print:
            log_print(f"[CAL] Unexpected count {cnt}, expecting {CAL_POINTS}.")

        b1 = read_exact(ser, 2*CAL_POINTS, 1.0)
        b2 = read_exact(ser, 2*CAL_POINTS, 1.0)
        if len(b1) != 2*CAL_POINTS or len(b2) != 2*CAL_POINTS:
            if verbose and log_print: log_print("[CAL] Incomplete calibration payload.")
            return False

        cal1 = [ (b1[2*i] | (b1[2*i+1] << 8)) for i in range(CAL_POINTS) ]
        cal2 = [ (b2[2*i] | (b2[2*i+1] << 8)) for i in range(CAL_POINTS) ]

        lut1 = build_lut(cal1)
        lut2 = build_lut(cal2)
        lut_ready = True
        use_lut_flag = True

        if verbose and log_print:
            log_print(f"[CAL] Received {CAL_POINTS} points per sensor. LUTs ready.")
        return True
    except Exception as e:
        if verbose and log_print: log_print(f"[CAL error] {e}")
        return False

# ===== helpers =====
def percentile(arr, p):
    if not arr:
        return None
    xs = sorted(arr)
    k = max(0, min(len(xs)-1, int(round((p/100.0)*(len(xs)-1)))))
    return xs[k]

def _moving_avg(xs, w=5):
    """Simple moving average with edge clamping."""
    if not xs:
        return []
    w = max(1, int(w))
    half = w // 2
    out = []
    n = len(xs)
    for i in range(n):
        s = 0.0
        c = 0
        for j in range(i - half, i + half + 1):
            jj = min(max(j, 0), n - 1)
            s += xs[jj]
            c += 1
        out.append(s / c)
    return out



def _median_filter(xs, w=3):
    """Odd-window median filter; clamps at edges."""
    if not xs:
        return []
    w = max(1, int(w) | 1)  # force odd
    r = w // 2
    out = []
    n = len(xs)
    for i in range(n):
        s = []
        for j in range(i - r, i + r + 1):
            jj = min(max(j, 0), n - 1)
            s.append(xs[jj])
        s.sort()
        out.append(s[len(s)//2])
    return out

def _despike_median(xs, max_cm=400, win=2, spike_up=24.0, spike_dn=18.0):
    """Replace isolated spikes/holes with neighborhood median (robust to 0→max_cm)."""
    n = len(xs)
    out = list(xs)
    for i in range(n):
        L = max(0, i - win); R = min(n, i + win + 1)
        neigh = [xs[j] for j in range(L, R) if j != i and xs[j] < max_cm]
        if len(neigh) < 2:
            continue
        m = _median(neigh)
        # huge/high spike (incl. 0 mapped to max_cm)
        if xs[i] >= max_cm or xs[i] > m + spike_up:
            out[i] = m
            continue
        # single-sample deep hole
        if xs[i] < m - spike_dn:
            left_ok  = (i > 0     and xs[i-1] >= m - spike_dn)
            right_ok = (i < n - 1 and xs[i+1] >= m - spike_dn)
            if left_ok and right_ok:
                out[i] = m
    return out

def _close_mask(mask, max_gap=2):
    """Morphological closing on boolean mask: fill tiny False runs between True runs."""
    out = list(mask)
    n = len(out); i = 0
    while i < n:
        if out[i]:
            i += 1; continue
        j = i
        while j < n and not out[j]:
            j += 1
        left_true  = (i - 1 >= 0 and out[i - 1])
        right_true = (j < n and out[j])
        if left_true and right_true and (j - i) <= max_gap:
            for k in range(i, j):
                out[k] = True
        i = j
    return out


def _split_on_internal_peaks(ds, start, end, jump_cm=8.0, min_sep=2):
    """
    Return a list of (s,e) subsegments inside [start,end] by splitting
    anywhere the signal climbs by >= jump_cm for >= min_sep consecutive points
    and then descends again. This prevents merging nearby boxes.
    """
    if end - start + 1 < 2*min_sep + 1:
        return [(start, end)]

    segments = []
    s = start
    i = start + 1
    while i <= end - min_sep:
        # consecutive climb?
        climb = all(ds[i+k] - ds[i-1+k] >= 0 for k in range(min_sep))
        big_jump = (ds[i+min_sep-1] - ds[s]) >= jump_cm
        if climb and big_jump:
            # close current segment before the hill
            e = i - 1
            if e >= s:
                segments.append((s, e))
            # skip the hill; start a new segment after it
            s = i + min_sep
            # reset local minimum reference
        i += 1
    if s <= end:
        segments.append((s, end))
    # keep only non-empty pieces
    segments = [(a,b) for (a,b) in segments if b - a + 1 >= min_sep]
    return segments if segments else [(start, end)]

def _median(lst):
    if not lst:
        return None
    s = sorted(lst)
    m = len(s)//2
    return (s[m] if len(s)%2==1 else 0.5*(s[m-1]+s[m]))

def _interp_edge(theta0, v0, theta1, v1, target):
    if v1 == v0:
        return theta0
    t = (target - v0) / (v1 - v0)
    t = max(0.0, min(1.0, t))
    return theta0 + t * (theta1 - theta0)

def map_angle_half(a_raw, a_min, a_max):
    """Stretch [a_min..a_max] to [ANG_DST0_DEG..ANG_DST1_DEG], then apply ANG_OFFSET_DEG.
       אם הטווח כבר גדול (≈סריקה מלאה), לא מותחים – רק מוסיפים אופסט."""
    span = float(a_max - a_min)
    a = float(a_raw)
    if span < 120.0 and span > 1e-6:   # חצי-סריקה => למתוח
        t = (a - float(a_min)) / span
        a = ANG_DST0_DEG + t * (ANG_DST1_DEG - ANG_DST0_DEG)
        if ANG_MIRROR:
            a = 180.0 - a
    a += ANG_OFFSET_DEG                  # אופסט גלובלי
    return max(0.0, min(180.0, a))


def map_angle_raw(a_raw, a_min, a_max):
    # מתיחה לינארית של טווח [a_min..a_max] ל-[ANG_DST0_DEG..ANG_DST1_DEG]
    if a_max <= a_min:
        a = float(a_raw)
    else:
        t = (float(a_raw) - float(a_min)) / float(a_max - a_min)
        a = ANG_DST0_DEG + t * (ANG_DST1_DEG - ANG_DST0_DEG)
    if ANG_MIRROR:
        a = 180.0 - a
    a += ANG_OFFSET_DEG
    return max(0.0, min(180.0, a))


def _merge_close_peaks(peaks, min_sep_deg=8.0):
    """
    מאחד פסגות (זיהויי אור) שמרוחקות פחות מ-min_sep_deg — משאיר את הקרובה יותר (מרחק קטן יותר).
    peaks: [(angle_deg, distance_cm), ...]
    """
    if not peaks:
        return []
    peaks = sorted(peaks, key=lambda p: p[0])
    groups = [[peaks[0]]]
    for p in peaks[1:]:
        if abs(p[0] - groups[-1][-1][0]) <= min_sep_deg:
            groups[-1].append(p)
        else:
            groups.append([p])
    return [min(g, key=lambda q: q[1]) for g in groups]

# ===== DIST object detection (robust) =====
def detect_objects_dist_cm(dist_list, ang_list,
                           max_cm=400,
                           base_pct=85,
                           min_drop_cm=10,
                           min_span=3,
                           gap_allow=0,
                           edge_keep_frac=0.35,
                           smooth_w=5,
                           side_guard_cm=3.0,
                           split_jump_cm=6.0,
                           split_consec=2,
                           min_obj_dist=8.0,
                           min_ang_span_deg=1.4,
                           max_valid_cm=400,
                           min_valid_cm=5,
                           beam_deg=3.0,
                           use_tan=True,
                           plateau_add=True,
                           plateau_slope_eps=0.3,
                           plateau_min_len=4,
                           plateau_drop_cm=10.0,
                           endpoint_relax_deg=5.0,
                           endpoint_min_span_deg=0.5,
                           edge_refine_slope=True,
                           min_width_cm=2.0):

    n = min(len(dist_list), len(ang_list))
    if n == 0:
        return []

    ds_raw = dist_list[:n]
    ds_work = [v if (min_valid_cm <= v <= max_valid_cm) else max_cm for v in ds_raw]
    ang = ang_list[:n]

    valid_for_base = [v for v in ds_work if v < max_cm]
    if not valid_for_base:
        return []

    base_glob = percentile(valid_for_base, base_pct)
    eff_thr = max(min_valid_cm + 1, base_glob - min_drop_cm)

    ds_work = _despike_median(ds_work, max_cm=max_cm, win=2, spike_up=24.0, spike_dn=18.0)
    ds_fill_for_smooth = [(v if v < max_cm else base_glob) for v in ds_work]
    ds_med = _median_filter(ds_fill_for_smooth, w=3)
    ds_sm = _moving_avg(ds_med, w=smooth_w)

    mask = [ds_sm[i] <= eff_thr for i in range(n)]
    mask = _close_mask(mask, max_gap=2)

    objs = []
    i = 0
    while i < n:
        if not mask[i]:
            i += 1
            continue

        start = i
        last = i
        j = i + 1
        gaps = 0
        while j < n:
            if mask[j]:
                last = j
                gaps = 0
                j += 1
            else:
                gaps += 1
                if gaps <= gap_allow:
                    last = j
                    j += 1
                else:
                    break

        span = last - start + 1
        true_pts = sum(1 for k in range(start, last + 1) if mask[k])

        if span >= min_span and true_pts >= min_span:
            objs_run = []
            for (seg_s, seg_e) in _split_on_internal_peaks(ds_sm, start, last,
                                                           jump_cm=split_jump_cm,
                                                           min_sep=split_consec):
                min_idx = min(range(seg_s, seg_e + 1), key=lambda k: ds_sm[k])
                d_min = ds_sm[min_idx]

                L_win = ds_sm[max(0, seg_s - 8):seg_s]
                R_win = ds_sm[seg_e + 1:min(n, seg_e + 9)]
                base_local = _median(L_win + R_win) or base_glob
                if (base_local - d_min) < min_drop_cm:
                    continue

                depth = max(0.0, base_local - d_min)
                t_depth = d_min + edge_keep_frac * depth
                t_guard = d_min + side_guard_cm
                t_edge = min(t_guard, t_depth)
                iL = min_idx

                while iL > seg_s and ds_sm[iL] <= t_edge:
                    iL -= 1
                theta_L = ang[iL] if iL == min_idx else _interp_edge(ang[iL], ds_sm[iL],
                                                                     ang[iL + 1], ds_sm[iL + 1], t_edge)

                iR = min_idx
                while iR < seg_e and ds_sm[iR] <= t_edge:
                    iR += 1
                theta_R = ang[iR] if iR == min_idx else _interp_edge(ang[iR - 1], ds_sm[iR - 1],
                                                                     ang[iR], ds_sm[iR], t_edge)

                # --- NEW: trim to the steepest shoulders (max rising slope) ---
                if edge_refine_slope:
                    # Left side: search [iL, min_idx) for maximum positive slope
                    if iL < min_idx - 1:
                        g_best, k_best = -1e9, None
                        for k in range(iL, min_idx):
                            da = max(1e-6, abs(ang[k + 1] - ang[k]))
                            g = (ds_sm[k + 1] - ds_sm[k]) / da
                            if g > g_best:
                                g_best, k_best = g, k
                        if k_best is not None:
                            theta_L = 0.5 * (ang[k_best] + ang[k_best + 1])

                    # Right side: search [min_idx, iR) for maximum positive slope
                    if min_idx < iR - 1:
                        g_best, k_best = -1e9, None
                        for k in range(min_idx, iR):
                            da = max(1e-6, abs(ang[k + 1] - ang[k]))
                            g = (ds_sm[k + 1] - ds_sm[k]) / da
                            if g > g_best:
                                g_best, k_best = g, k
                        if k_best is not None:
                            theta_R = 0.5 * (ang[k_best] + ang[k_best + 1])

                ang_center = 0.5 * (theta_L + theta_R)
                d_center = _median(ds_sm[max(seg_s, min_idx - 2):min(seg_e + 1, min_idx + 3)]) or d_min

                dtheta_deg  = max(0.0, (theta_R - theta_L) - beam_deg)
                at_endpoint = (ang[min_idx] <= endpoint_relax_deg) or (ang[min_idx] >= 180.0 - endpoint_relax_deg)
                span_ok     = (dtheta_deg >= (endpoint_min_span_deg if at_endpoint else min_ang_span_deg))
                if d_center >= min_obj_dist and span_ok:
                    dtheta_rad = math.radians(max(dtheta_deg, endpoint_min_span_deg))
                    width_geom = 2.0 * d_center * (math.tan(dtheta_rad / 2.0) if use_tan else math.sin(dtheta_rad / 2.0))
                    width_cm   = max(min_width_cm, width_geom * WIDTH_SCALE)
                    objs_run.append((ang_center, d_center, width_cm))

            if plateau_add:
                slopes = [ds_sm[k+1] - ds_sm[k] for k in range(start, last)]
                idx = 0
                while idx < len(slopes):
                    if abs(slopes[idx]) <= plateau_slope_eps:
                        s = idx
                        while idx < len(slopes) and abs(slopes[idx]) <= plateau_slope_eps:
                            idx += 1
                        e = idx
                        p_s = start + s
                        p_e = start + e
                        if (p_e - p_s + 1) >= plateau_min_len:
                            d_center   = _median(ds_sm[p_s:p_e+1])
                            L_win      = ds_sm[max(0, start - 8):start]
                            R_win      = ds_sm[last + 1:min(n, last + 9)]
                            base_local = _median(L_win + R_win) or base_glob
                            drop       = base_local - d_center
                            if drop >= plateau_drop_cm:
                                ang_center = _median(ang[p_s:p_e+1])
                                # be more conservative to avoid a 2nd hit on the same box
                                dup = any(abs(ang_center - oc) <= 10.0 for (oc, _, _) in objs_run)
                                if not dup:
                                    # estimate width for plateaus instead of forcing 0
                                    depth   = max(0.0, (base_local - d_center))
                                    t_depth = d_center + edge_keep_frac * depth
                                    t_guard = d_center + side_guard_cm
                                    t_edge  = min(t_guard, t_depth)

                                    # search left edge from plateau start
                                    iL = p_s
                                    while iL > start and ds_sm[iL] <= t_edge:
                                        iL -= 1
                                    theta_L = ang[iL] if iL == p_s else _interp_edge(
                                        ang[iL], ds_sm[iL], ang[iL+1], ds_sm[iL+1], t_edge)

                                    # search right edge from plateau end
                                    iR = p_e
                                    while iR < last and ds_sm[iR] <= t_edge:
                                        iR += 1
                                    theta_R = ang[iR] if iR == p_e else _interp_edge(
                                        ang[iR-1], ds_sm[iR-1], ang[iR], ds_sm[iR], t_edge)

                                    # compute a non-zero width from those edges
                                    dtheta_deg = max(0.0, (theta_R - theta_L) - beam_deg)
                                    dtheta_rad = math.radians(max(dtheta_deg, endpoint_min_span_deg))
                                    width_geom = 2.0 * d_center * (math.tan(dtheta_rad/2.0) if use_tan else math.sin(dtheta_rad/2.0))
                                    width_cm   = max(min_width_cm, width_geom * WIDTH_SCALE)

                                    objs_run.append((float(ang_center), float(d_center), float(width_cm)))
                    else:
                        idx += 1

            objs.extend(objs_run)

        i = last + 1
    return objs

def detect_lights_cm(dist_list, ang_list,
                     max_cm=400,
                     base_pct=85,
                     min_drop_cm=8,
                     min_span=3,
                     gap_allow=1,
                     smooth_w=5,
                     max_valid_cm=400,
                     min_valid_cm=2):
    """
    Find local 'dips' (contiguous runs below an adaptive baseline).
    Returns a list of (angle_center_deg, distance_cm) with no width.
    """
    n = min(len(dist_list), len(ang_list))
    if n == 0:
        return []

    ds_raw = dist_list[:n]
    ang = ang_list[:n]

    # cap to valid; invalid -> max_cm
    ds_work = [v if (min_valid_cm <= v <= max_valid_cm) else max_cm for v in ds_raw]

    valid_vals = [v for v in ds_work if v < max_cm]
    if not valid_vals:
        return []

    base_glob = percentile(valid_vals, base_pct)
    eff_thr = max(min_valid_cm + 1, base_glob - min_drop_cm)

    # light smoothing (edge-preserving enough for our purpose)
    fill = [(v if v < max_cm else base_glob) for v in ds_work]
    ds_sm = _moving_avg(fill, w=smooth_w)

    # mark candidates below threshold
    mask = [(ds_work[i] < max_cm and ds_sm[i] <= eff_thr) for i in range(n)]
    mask = _close_mask(mask, max_gap=1)  # סגירה מורפולוגית של פערים קטנים

    lights = []
    i = 0
    while i < n:
        if not mask[i]:
            i += 1
            continue
        start = i
        last = i
        j = i + 1
        gaps = 0
        while j < n:
            if mask[j]:
                last = j
                gaps = 0
            else:
                gaps += 1
                if gaps > gap_allow:
                    break
            j += 1

        span = last - start + 1
        true_pts = sum(1 for k in range(start, last + 1) if mask[k])

        if span >= min_span and true_pts >= min_span:
            # pick the minimum point in this run
            min_idx = min(range(start, last + 1), key=lambda k: ds_sm[k])
            lights.append((float(ang[min_idx]), float(ds_sm[min_idx])))

        i = last + 1

    lights = _merge_close_peaks(lights, min_sep_deg=8.0)
    return lights

# --- Script encoder (for file upload path) ---
def encode_script_text(txt: str) -> bytes:
    opmap = {"inc_lcd": 1, "dec_lcd": 2, "rra_lcd": 3, "set_delay": 4,
             "clear_lcd": 5, "servo_deg": 6, "servo_scan": 7, "sleep": 8}
    out_lines = []
    for raw in txt.splitlines():
        line = raw.strip()
        if not line:
            continue
        parts = line.split(" ", 1)
        cmd = parts[0]
        args = parts[1] if len(parts) > 1 else ""
        op = opmap.get(cmd)
        if op is None:
            continue
        s = f"{op:02d}"
        if args:
            if cmd == "servo_scan":
                a1, a2 = [x.strip() for x in args.split(",", 1)]
                s += f"{int(a1):02X}{int(a2):02X}"
            else:
                s += f"{int(args):02X}"
        out_lines.append(s)
    return ("\n".join(out_lines) + "\n").encode("ascii")

# ===== Background listener =====
def uart_listener(ser):
    global print_pending_dist, print_pending_ldr
    prev_val = None
    while not stop_event.is_set():
        try:
            if ser is None:
                time.sleep(0.02)
                continue
            mode = current_mode

            if mode == "dist":
                while not stop_event.is_set() and current_mode == "dist":
                    b0 = ser.read(1)
                    if not b0:
                        continue
                    if b0 == b"$":
                        if current_mode == "dist" and print_pending_dist:
                            print(f"\n[DIST] Batch complete ({len(Dist_Samp)} samples).")
                            print_pending_dist = False
                        break
                    else:
                        rest = read_exact(ser, 2, max_wait_s=0.2)
                        if len(rest) != 2:
                            continue
                        frame = b0 + rest
                        raw_val, angle = parse_frame3(frame)
                        if raw_val is None:
                            continue
                        val_cm = to_cm(raw_val)
                        Dist_Samp.append(val_cm)
                        Angle_Samp.append(angle)



            # ====================== uart_listener(...) ======================

            elif mode == "ldr":

                # LDR is handled synchronously by ldr_scan_once().

                # Keep the branch so current_mode == "ldr" still means something,

                # but do NOT read from the port here.

                while not stop_event.is_set() and current_mode == "ldr":
                    time.sleep(0.01)

                # no serial reads, no parsing


            # ==================== end LDR branch replacement ====================

            elif mode == "Telemeter":
                frame = read_exact(ser, 3, max_wait_s=0.5)
                if len(frame) != 3:
                    continue
                raw_val, angle = parse_frame3(frame)
                if raw_val is None:
                    continue
                val_cm = to_cm(raw_val)
                if prev_val is None or abs(prev_val - val_cm) > 2:
                    prev_val = val_cm
                    print(f"[RX Tele] {val_cm} cm @ angle {angle}")

            else:
                time.sleep(0.02)

        except Exception:
            time.sleep(0.05)

class StdoutTee:
    def __init__(self, element):
        self.element = element
        self._orig = sys.__stdout__
    def write(self, data):
        try:
            self._orig.write(data)
        except Exception:
            pass
        try:
            if data:
                self.element.update(value=self.element.get() + str(data))
                self.element.set_vscroll_position(1.0)
        except Exception:
            pass
    def flush(self):
        try:
            self._orig.flush()
        except Exception:
            pass


def pad16_name(name: str) -> bytes:
    b = (name or "").encode("ascii", "ignore")
    if len(b) > 16:
        b = b[:16]
    if len(b) < 16:
        b = b + b" " * (16 - len(b))
    return b

def ascii_name_from_path(path: str) -> str:
    base = os.path.basename(path) if path else ""
    base_ascii = base.encode("ascii", "ignore").decode("ascii", "ignore")
    return base_ascii

def fetch_script_names(_ser=None, wait_s=0.0):
    return list(Local_Scripts)

# ===== Synchronous DIST scan =====
def dist_scan_once(ser, mask_cm, log_cb=print, debug=False, header_timeout_s=3.0):
    """
    New synchronous DIST scan (matches MCU protocol):
      PC -> MCU: '1' then 's'
      MCU -> PC: 0xFF, u16(nsamp), then nsamp * (angle, LSB, MSB)
    Returns (ds_cm, ang_deg, objs_visible)
    """
    ds, ang = [], []
    try:
        if ser is None:
            log_cb("[DIST] Not connected.")
            return ds, ang, []

        # Enter DIST mode and start
        ser.reset_input_buffer()
        ser.write(b"1"); ser.flush()
        time.sleep(0.05)
        ser.write(b"s"); ser.flush()

        # Header
        nsamp = read_scan_header(ser, timeout_s=header_timeout_s, log_cb=log_cb, dbg_tag="DIST")
        if nsamp is None or nsamp <= 0:
            return ds, ang, []
        if debug:
            log_cb(f"[DIST RX] Header OK. nsamp={nsamp}")

        # Samples
        missed = 0
        for i in range(nsamp):
            trip = read_exact(ser, 3, max_wait_s=0.7)
            if len(trip) != 3:
                missed += 1
                continue
            a = min(180, trip[0])
            tof = trip[1] | (trip[2] << 8)  # LSB, MSB
            cm = to_cm(tof)

            ds.append(cm)
            ang.append(a)

            if debug and i < 12:
                log_cb(f"[DIST RX] i={i:03d} θ={a:3d}  raw=0x{tof:04X}  d≈{cm}cm")

        if missed and debug:
            log_cb(f"[DIST RX] Missed {missed} / {nsamp} frames (timeouts).")

        if not ds:
            log_cb("[DIST] No samples captured.")
            return ds, ang, []

        log_cb("[DIST] Running detection...")

        objs_all = detect_objects_dist_cm(
            ds, ang,
            base_pct=80,  # was 85
            min_drop_cm=2.0,  # was 6
            min_span=3,
            gap_allow=0,
            edge_keep_frac=0.35,
            smooth_w=5,
            side_guard_cm=2.0,  # was 3
            split_jump_cm=4.0,  # was 6
            split_consec=2,
            min_obj_dist=8.0,
            min_ang_span_deg=0.6,  # was 1.4
            max_valid_cm=400,
            min_valid_cm=5,
            beam_deg=BEAM_DEG_COMP,
            use_tan=True,
            plateau_add=True,
            plateau_slope_eps=0.6,
            plateau_min_len=3,
            plateau_drop_cm=2.5,  # was 6
            endpoint_relax_deg=5.0,
            endpoint_min_span_deg=0.4,
            edge_refine_slope=True,
            min_width_cm=1.5  # was 2
        )

        # Display corrections + mask
        objs_corr = []
        for (ang_c, d_cm, w_cm) in objs_all:
            ang_adj = max(0.0, min(180.0, ang_c + ANG_OFFSET_DEG))
            # width already scaled inside detect_objects_dist_cm → don't scale again
            w_adj = max(0.0, float(w_cm))
            objs_corr.append((ang_adj, d_cm, w_adj))

        try:
            mask_val = float(mask_cm)
        except Exception:
            mask_val = 50.0
        objs_visible = [o for o in objs_corr if o[1] <= mask_val]

        if not objs_visible:
            log_cb("[OBJ] לא זוהו עצמים בתוך טווח ה-MASK.")
        else:
            for idx, (ang_c, d_cm, w_cm) in enumerate(objs_visible, start=1):
                log_cb(f"[OBJ {idx}] θ={ang_c:.1f}°, d={d_cm:.1f} cm, w≈{w_cm:.1f} cm")

            # Make the grey background samples use the same angle offset
        ang_disp = [max(0.0, min(180.0, a + ANG_OFFSET_DEG)) for a in ang]

        ser.reset_input_buffer()
        return ds, ang_disp, objs_visible

    except Exception as e:
        log_cb(f"[DIST error] {e}")
        return ds, ang, []




def show_radar(ds, ang, objs, title="Scanner Map (0–180°)", rmax=None,
               extra_sets=None, detected_color="yellow", legend_label="Detected"):

    """
    Polar 'radar':
      • Grey dots  = all samples (angle, distance)
      • Yellow dots = detected objects (with labels)
      • extra_sets  = optional [(angles, distances, label), ...] overlays (e.g., LDR1/LDR2)
    """
    # close any old figure
    try:
        plt.close('all')
    except Exception:
        pass

    # figure & polar axes
    fig = plt.figure(title)
    ax  = plt.subplot(111, polar=True)
    ax.set_theta_zero_location("E")
    ax.set_thetamin(0); ax.set_thetamax(180)
    try:
        ax.set_thetagrids(range(0, 181, 30))  # 0°,30°,...,180°
    except Exception:
        pass

    # radius range (auto unless rmax provided)
    if rmax is None:
        positive = [v for v in ds if isinstance(v, (int, float)) and v > 0]
        rmax_calc = max(positive + [50])               # at least 50 cm
        rmax_calc = min(max(50, int((rmax_calc+9)//10)*10), 400)
        rmax = rmax_calc
    else:
        rmax = max(10, min(400, rmax))
    ax.set_rlim(0, rmax)
    ax.grid(True)
    ax.set_title(title, va='bottom')

    # --- background samples (grey) ---
    try:
        th_all = [math.radians(max(0, min(180, float(a)))) for a in ang]
        r_all = [max(0.0, float(d)) for d in ds]
        sc_all = ax.scatter(th_all, r_all, s=14, color='0.7', alpha=0.9, label="Samples")
    except Exception:
        sc_all = None

    # --- optional extra sets (e.g., LDR1 / LDR2) ---
    if extra_sets:
        markers = ['x', '^', 's', 'd']
        for idx, (ang_ex, dist_ex, lbl) in enumerate(extra_sets):
            try:
                th_e = [math.radians(max(0, min(180, float(a)))) for a in ang_ex]
                r_e = [max(0.0, float(d)) for d in dist_ex]
                m = markers[idx % len(markers)]
                # Color "Light ..." overlays in yellow 'x'; others keep the existing scheme
                if isinstance(lbl, str) and lbl.lower().startswith("light"):
                    ax.scatter(th_e, r_e, s=38, marker='x', color='yellow',
                               linewidths=1.0, label=str(lbl))
                elif m == 'x':
                    ax.scatter(th_e, r_e, s=22, marker=m, color='black',
                               linewidths=0.8, label=str(lbl))
                else:
                    ax.scatter(th_e, r_e, s=22, marker=m, facecolors='none',
                               edgecolors='black', linewidths=0.6, label=str(lbl))

            except Exception:
                pass

    # --- detected objects (yellow with labels) ---
    first_obj = True

    if objs:
        for ang_c, d_cm, w_cm in objs:
            th = math.radians(max(0, min(180, float(ang_c))))
            rr = max(0.0, float(d_cm))
            ax.scatter([th], [rr], s=120, marker='o',
                       facecolors=detected_color, edgecolors='black', linewidths=1.0,
                       label=(legend_label if first_obj else None))
            first_obj = False

            # Label format: [distance, angle°, width]
            w_show = 0 if (w_cm is None or (isinstance(w_cm, (int, float)) and abs(w_cm) < 1e-6)) else int(round(w_cm))
            label = f"[{int(round(rr))}, {ang_c:.1f}°, {w_show}]"
            ax.annotate(label, xy=(th, rr), xytext=(8, 8), textcoords="offset points")

    try:
        ax.legend(loc="upper right", bbox_to_anchor=(1.18, 1.12))
    except Exception:
        pass

    plt.show()
# === NEW: helpers to draw onto existing axes, and a 3-in-1 window ===
def _polar_common(ax, title, rmax):
    ax.set_theta_zero_location("E")
    ax.set_thetamin(0); ax.set_thetamax(180)
    try: ax.set_thetagrids(range(0, 181, 30))
    except Exception: pass
    ax.set_rlim(0, max(10, min(400, (rmax if rmax is not None else 200))))
    ax.grid(True)
    ax.set_title(title, va='bottom')

def draw_radar_on(ax, ds, ang, points=None, *,
                  title="", rmax=None, detected_color="yellow",
                  legend_label="Detected", extra_sets=None,
                  samples_label="Samples"):
    _polar_common(ax, title, rmax)
    # background
    try:
        th_all = [math.radians(max(0, min(180, float(a)))) for a in ang]
        r_all  = [max(0.0, float(d)) for d in ds]
        ax.scatter(th_all, r_all, s=14, color='0.7', alpha=0.9, label=samples_label)
    except Exception:
        pass
    # extras (e.g., LDR1/LDR2 curves as hollow markers)
    if extra_sets:
        markers = ['x', '^', 's', 'd']
        for idx, (ang_ex, dist_ex, lbl) in enumerate(extra_sets):
            try:
                th_e = [math.radians(max(0, min(180, float(a)))) for a in ang_ex]
                r_e  = [max(0.0, float(d)) for d in dist_ex]
                m = markers[idx % len(markers)]
                if m == 'x':
                    ax.scatter(th_e, r_e, s=22, marker=m, color='black', linewidths=0.8, label=str(lbl))
                else:
                    ax.scatter(th_e, r_e, s=22, marker=m, facecolors='none', edgecolors='black',
                               linewidths=0.6, label=str(lbl))
            except Exception:
                pass
    # detected points
    if points:
        first = True
        for ang_c, d_cm, w_cm in points:
            th = math.radians(max(0, min(180, float(ang_c))))
            rr = max(0.0, float(d_cm))
            ax.scatter([th], [rr], s=120, marker='o',
                       facecolors=detected_color, edgecolors='black', linewidths=1.0,
                       label=(legend_label if first else None))
            first = False
            w_show = 0 if (w_cm is None or (isinstance(w_cm, (int, float)) and abs(w_cm) < 1e-6)) else int(round(w_cm))
            ax.annotate(f"[{int(round(rr))}, {ang_c:.1f}°, {w_show}]",
                        xy=(th, rr), xytext=(8, 8), textcoords="offset points")
    try: ax.legend(loc="upper right", bbox_to_anchor=(1.18, 1.12))
    except Exception: pass

def show_radars_triple(ang, obj_ds, objs_pts, ldr_comb, ldr1, ldr2, lights_pts, rmax=None):
    """
    Draws three polar radars in a single window:
      Top   = Objects (blue)
      Middle= LDR lights (yellow) + LDR1/LDR2 as extras
      Bottom= Combined (blue objects + yellow lights) over fused LDR samples
    """
    try: plt.close('all')
    except Exception: pass

    fig = plt.figure("Light & Object – Triple Radar", figsize=(9, 16))
    ax_top    = plt.subplot(3,1,1, polar=True)
    ax_mid    = plt.subplot(3,1,2, polar=True)
    ax_bottom = plt.subplot(3,1,3, polar=True)

    # TOP: Objects (blue)
    draw_radar_on(ax_top, obj_ds, ang, objs_pts,
                  title="Objects (Ultrasonic) — blue",
                  rmax=rmax, detected_color="blue",
                  legend_label="Objects")

    # MIDDLE: LDR lights (yellow) + LDR1/LDR2 samples
    draw_radar_on(ax_mid, ldr_comb, ang, [(a,d,0.0) for (a,d) in lights_pts],
                  title="Light Sources (LDR) — yellow",
                  rmax=rmax, detected_color="yellow",
                  legend_label="Light sources",
                  extra_sets=[(ang, ldr1, "LDR1 samples"),
                              (ang, ldr2, "LDR2 samples")],
                  samples_label="LDR fused samples")

    # BOTTOM: Combined view — baseline=fused LDR samples, overlay both sets
    draw_radar_on(ax_bottom, ldr_comb, ang, None,
                  title="Combined view (Objects + Lights)",
                  rmax=rmax, samples_label="LDR fused samples")
    # overlay objects (blue)
    if objs_pts:
        th = [math.radians(max(0,min(180,float(a)))) for a,_,_ in objs_pts]
        rr = [max(0.0,float(d)) for _,d,_ in objs_pts]
        ax_bottom.scatter(th, rr, s=120, marker='o', facecolors='blue', edgecolors='black',
                          linewidths=1.0, label="Objects")
    # overlay lights (yellow)
    if lights_pts:
        th = [math.radians(max(0,min(180,float(a)))) for a,_ in lights_pts]
        rr = [max(0.0,float(d)) for _,d in lights_pts]
        ax_bottom.scatter(th, rr, s=120, marker='o', facecolors='yellow', edgecolors='black',
                          linewidths=1.0, label="Light sources")
    try: ax_bottom.legend(loc="upper right", bbox_to_anchor=(1.18, 1.12))
    except Exception: pass

    # global legend for protocol
    fig.suptitle("Top: Objects (blue)  |  Middle: LDR lights (yellow)  |  Bottom: Combined\n"
                 "Packet format (COMBO): angle, dist_LSB, dist_MSB, angle(dup), LDR1, LDR2",
                 y=0.98, fontsize=11)
    plt.tight_layout(rect=[0, 0, 1, 0.95])
    plt.show()

def show_results_table(angles, distances, title="Scan results"):
    """
    Popup table of raw samples: Angle (deg) vs Distance (cm).
    Uses explicit light colors so text is visible on all rows.
    Adds Copy buttons for quick pasting into chat.
    """
    if not angles or not distances:
        sg.popup("No samples to display.")
        return

    rows = [[f"{float(a):.1f}", f"{float(d):.1f}"] for a, d in zip(angles, distances)]
    headings = ["Angle (°)", "Distance (cm)"]

    # consistent light scheme (white text only in header)
    TABLE_BG          = "#ffffff"   # normal rows
    TABLE_ALT_BG      = "#eef3f9"   # alternating rows
    TABLE_TXT         = "#111111"   # cell text
    HEAD_BG           = "#334155"   # header background
    HEAD_TXT          = "#ffffff"   # header text

    layout = [
        [sg.Text(title, font=("Segoe UI", 12, "bold"))],
        [sg.Table(
            values=rows,
            headings=headings,
            key="-TAB-",
            justification="center",
            auto_size_columns=False,
            col_widths=[12, 14],
            num_rows=min(20, len(rows)),
            expand_x=True, expand_y=True,
            alternating_row_color=TABLE_ALT_BG,
            background_color=TABLE_BG,
            text_color=TABLE_TXT,
            header_background_color=HEAD_BG,
            header_text_color=HEAD_TXT,
            header_font=("Segoe UI", 10, "bold"),
            font=("Segoe UI", 10),
        )],
        [
            sg.Button("Copy CSV", key="-COPYCSV-"),
            sg.Button("Copy (tab-separated)", key="-COPYTSV-"),
            sg.Button("Export CSV", key="-CSV-"),
            sg.Button("Close", key="-CLOSE-"),
        ],
    ]

    w = sg.Window(title, layout, finalize=True, modal=True, resizable=True)

    def _copy_to_clipboard(sep=","):
        header = sep.join(["Angle (deg)", "Distance (cm)"])
        lines  = [f"{float(a):.1f}{sep}{float(d):.1f}" for a, d in zip(angles, distances)]
        text   = header + "\n" + "\n".join(lines)
        sg.clipboard_set(text)
        sg.popup_quick_message("Copied to clipboard", auto_close=True, auto_close_duration=1)

    while True:
        ev, _ = w.read()
        if ev in (sg.WIN_CLOSED, "-CLOSE-"):
            break
        if ev == "-COPYCSV-":
            _copy_to_clipboard(",")
        elif ev == "-COPYTSV-":
            _copy_to_clipboard("\t")
        elif ev == "-CSV-":
            path = sg.popup_get_file(
                "Save results as CSV",
                save_as=True, no_window=True,
                default_extension=".csv",
                file_types=(("CSV", "*.csv"),),
            )
            if path:
                try:
                    with open(path, "w", encoding="utf-8") as f:
                        f.write("Angle (deg),Distance (cm)\n")
                        for a, d in zip(angles, distances):
                            f.write(f"{float(a):.1f},{float(d):.1f}\n")
                    sg.popup_quick_message("Saved.", auto_close=True, auto_close_duration=1)
                except Exception as e:
                    sg.popup_error(f"Failed to save:\n{e}")
    w.close()


def show_results_table_dual(angles, d1_list, d2_list, title="LDR scan – LDR1 & LDR2 arrays"):
    if not angles or not d1_list or not d2_list:
        sg.popup("No LDR samples to display."); return

    def _mk_rows(vals):
        return [[f"{float(a):.1f}", f"{float(v):.1f}"] for a, v in zip(angles, vals)]

    HEAD_BG, HEAD_TXT = "#334155", "#ffffff"
    TABLE_BG, TABLE_ALT_BG, TABLE_TXT = "#ffffff", "#eef3f9", "#111111"

    layout = [
        [sg.Text(title, font=("Segoe UI", 12, "bold"))],
        [sg.Text("LDR1", font=("Segoe UI", 10, "bold"))],
        [sg.Table(values=_mk_rows(d1_list), headings=["Angle (°)", "LDR1 distance (cm)"],
                  key="-TAB1-", justification="center", auto_size_columns=False,
                  col_widths=[12, 18], num_rows=min(12, len(angles)), expand_x=True, expand_y=False,
                  alternating_row_color=TABLE_ALT_BG, background_color=TABLE_BG, text_color=TABLE_TXT,
                  header_background_color=HEAD_BG, header_text_color=HEAD_TXT,
                  header_font=("Segoe UI", 10, "bold"), font=("Segoe UI", 10))],
        [sg.Text("LDR2", font=("Segoe UI", 10, "bold"))],
        [sg.Table(values=_mk_rows(d2_list), headings=["Angle (°)", "LDR2 distance (cm)"],
                  key="-TAB2-", justification="center", auto_size_columns=False,
                  col_widths=[12, 18], num_rows=min(12, len(angles)), expand_x=True, expand_y=False,
                  alternating_row_color=TABLE_ALT_BG, background_color=TABLE_BG, text_color=TABLE_TXT,
                  header_background_color=HEAD_BG, header_text_color=HEAD_TXT,
                  header_font=("Segoe UI", 10, "bold"), font=("Segoe UI", 10))],
        [sg.Button("Export CSV (LDR1)", key="-CSV1-"),
         sg.Button("Export CSV (LDR2)", key="-CSV2-"),
         sg.Button("Close", key="-CLOSE-")],
    ]
    w = sg.Window(title, layout, finalize=True, modal=True, resizable=True)

    def _save(path, vals, name):
        with open(path, "w", encoding="utf-8") as f:
            f.write("Angle (deg),Distance (cm)\n")
            for a, v in zip(angles, vals):
                f.write(f"{float(a):.1f},{float(v):.1f}\n")
        sg.popup_quick_message(f"Saved {name}.", auto_close=True, auto_close_duration=1)

    while True:
        ev, _ = w.read()
        if ev in (sg.WIN_CLOSED, "-CLOSE-"):
            break
        if ev in ("-CSV1-", "-CSV2-"):
            which = "LDR1" if ev == "-CSV1-" else "LDR2"
            vals  = d1_list if ev == "-CSV1-" else d2_list
            path = sg.popup_get_file(f"Save {which} as CSV", save_as=True, no_window=True,
                                     default_extension=".csv", file_types=(("CSV", "*.csv"),))
            if path:
                try: _save(path, vals, which)
                except Exception as e: sg.popup_error(f"Failed to save:\n{e}")
    w.close()
def detect_desc_run_first_min(angles, dists, eps=0.1, min_run=3, min_drop=4.0):
    """
    מאתר מינימום *ראשון* בכל ריצה מונוטונית-יורדת.
    מסנן ריצות קצרות/רדודות בעזרת min_run ו-min_drop.
    מחזיר [(angle_deg, distance_cm), ...].
    """
    n = min(len(angles), len(dists))
    out = []
    if n < 2:
        return out

    run_s = None
    best_idx = None
    best_val = None
    start_val = None
    run_len = 0

    def flush_run():
        nonlocal run_s, best_idx, best_val, start_val, run_len
        if run_s is not None and best_idx is not None:
            drop = (start_val - best_val) if (start_val is not None) else 0.0
            if run_len >= min_run and drop >= min_drop:
                out.append((float(angles[best_idx]), float(dists[best_idx])))
        run_s = None; best_idx = None; best_val = None; start_val = None; run_len = 0

    for i in range(1, n):
        if dists[i] < dists[i-1] - eps:  # ממשיך/מתחיל ירידה
            if run_s is None:
                run_s = i-1
                start_val = dists[i-1]
                best_idx = i
                best_val = dists[i]
                run_len = 2
            else:
                run_len += 1
                if dists[i] < best_val - eps or (abs(dists[i]-best_val) <= eps and i < best_idx):
                    best_val = dists[i]
                    best_idx = i
        else:
            flush_run()  # הסתיימה ריצה יורדת (עלייה/פלטו)

    flush_run()  # לריצה אחרונה אם קיימת
    return out


def pick_single_flat_min(angles, dists, mask_val, eps=0.4):
    """
    מחזיר נקודה יחידה: מרכז הפלטו של המינימום בתוך המסכה (אם יש),
    אחרת – מרכז הפלטו הגלובלי. eps מגדיר כמה 'שווה-מינימום'.
    """
    n = len(dists)
    idxs = [i for i in range(n) if dists[i] <= mask_val]
    if not idxs:
        idxs = list(range(n))
    dmin = min(dists[i] for i in idxs)
    near = [i for i in idxs if abs(dists[i] - dmin) <= eps]
    j = (near[0] + near[-1]) // 2
    return float(angles[j]), float(dists[j])

def pick_lights_two_sides(angles, dists, mask_val,
                          base_pct=LIGHT_BASE_PCT,
                          min_drop_cm=LIGHT_MIN_DROP_CM,
                          smooth_w=LIGHT_SMOOTH_W,
                          min_span=2,
                          min_sep_deg=LIGHT_MIN_SEP_DEG,
                          side_guard_deg=LIGHT_SIDE_GUARD_DEG,
                          center_band_deg=LIGHT_CENTER_BAND_DEG,
                          max_peaks=LIGHT_MAX_SOURCES):
    """
    מאתר את כל המינימות החוקיות על עקומת ה-LDR המאוחדת,
    ממזג קרובות מדי, ובוחר עד 3: אחת לשמאל, אחת לימין, ואחת "באמצע" אם קיימת.
    מחזיר [(angle_deg, distance_cm), ...] ממיונות לפי זווית.
    """
    # 1) קנדידטים: נשתמש במאתר הקיים כדי לקבל מינימות בכל הריצות
    cand = detect_lights_cm(
        dist_list=dists,
        ang_list=angles,
        base_pct=base_pct,
        min_drop_cm=min_drop_cm,
        min_span=min_span,
        gap_allow=1,
        smooth_w=smooth_w
    )

    # 2) מסכה לפי טווח
    cand = [p for p in cand if p[1] <= float(mask_val)]
    if not cand:
        return []

    # 3) מיזוג פסגות קרובות (נשמור את הקרובה יותר)
    cand = _merge_close_peaks(cand, min_sep_deg=min_sep_deg)

    # 4) הפרדת צדדים סביב האמצע של הטווח הנוכחי
    mid = 0.5 * (min(angles) + max(angles))
    left   = [p for p in cand if p[0] <= mid - side_guard_deg]
    right  = [p for p in cand if p[0] >= mid + side_guard_deg]
    center = [p for p in cand if abs(p[0] - mid) < center_band_deg]

    # נעדיף "קרוב יותר" (d קטן יותר) בכל קבוצה
    left_best   = min(left,   key=lambda x: x[1]) if left   else None
    right_best  = min(right,  key=lambda x: x[1]) if right  else None
    center_best = min(center, key=lambda x: x[1]) if center else None

    selected = []
    if left_best:
        selected.append(left_best)
    if right_best:
        # שמור מרחק זוויתי מינימלי מאשר שנבחר לשמאל
        if all(abs(right_best[0] - s[0]) >= min_sep_deg for s in selected):
            selected.append(right_best)

    # 5) מקור שלישי "באמצע" (אם יש מקום והוא לא צמוד לאחרים)
    if len(selected) < max_peaks and center_best:
        if all(abs(center_best[0] - s[0]) >= min_sep_deg for s in selected):
            selected.append(center_best)

    # 6) אם עדיין חסר (למשל יש רק צד אחד), נבחר מהמועמדים הכלליים הקרובים ביותר
    if len(selected) < max_peaks:
        remaining = sorted([p for p in cand if p not in selected], key=lambda x: x[1])
        for p in remaining:
            if all(abs(p[0] - s[0]) >= min_sep_deg for s in selected):
                selected.append(p)
                if len(selected) >= max_peaks:
                    break

    return sorted(selected, key=lambda x: x[0])


# ===== Windows =====
def object_detector_window(ser):
    layout = [
        [sg.Text("Masking Distance [cm]:"),
         sg.Slider(range=(10,400), resolution=1, default_value=80, orientation='h',
                   size=(34,15), key="-MASK-"),
         sg.Checkbox("Debug prints", key="-DBG-", default=False),
         sg.Checkbox("Also print to Terminal", key="-PRINT-", default=False)],
        [sg.Button("Start Objects Scan", key="-START-"), sg.Button("Back", key="-BACK-")],
        [sg.Multiline(size=(70,12), key="-LOG-", autoscroll=True, disabled=True)]
    ]
    win = sg.Window("Objects Detector System", layout, finalize=True, modal=True)
    log_box = win["-LOG-"]

    def log_cb(msg):
        s = msg if msg.endswith("\n") else (msg + "\n")
        log_box.update(value=log_box.get() + s)
        log_box.set_vscroll_position(1.0)
        if bool(win["-PRINT-"].get()):
            try: print(s, end="")
            except Exception: pass

    try:
        if ser:
            ser.reset_input_buffer()
            ser.write(b"1"); ser.flush()
    except Exception as e:
        log_cb(f"[TX error] {e}\n")

    while True:
        ev, vals = win.read()
        if ev in (sg.WIN_CLOSED, "-BACK-"):
            ensure_idle(ser)  # unified cleanup: '0' + flush + clear state
            break
        if ev == "-START-":

            try:
                m = int(vals.get("-MASK-", 80))
            except Exception:
                m = 15
            m = max(1, min(400, m))
            dbg = bool(vals.get("-DBG-", False))
            log_cb(f"[MASK] Using masking distance ≥ {m} cm (זיהוי מתבסס על ערכים נקיים).\n")
            ds, ang, objs = dist_scan_once(ser, m, log_cb=log_cb, debug=dbg)
            show_radar(ds, ang, objs, title="Scanner Map (0–180°)", rmax=m + 5,
                       detected_color="blue", legend_label="Objects")
            show_results_table(ang, ds, title="Object scan – Angle vs Distance")

    win.close()

def ldr_scan_once(ser, log_cb=print, timeout_s=8.0, debug=False):
    """
    Synchronous LDR scan:
      PC -> MCU: '3' (enter LDR mode), 'l' (wake LDR_awake / start)
      MCU -> PC: 0xFF, u16(sample_count), then sample_count * (angle, ldr1, ldr2)
    Returns (angles_deg, combined_cm, d1_cm, d2_cm).
    """
    global current_mode
    if ser is None:
        log_cb("[LDR] Not connected.")
        return [], [], [], []

    # Make sure LUTs are ready if requested
    if use_lut_flag and not lut_ready:
        log_cb("[LDR] LUTs not loaded. Attempting fetch...")
        if not fetch_calibration_from_mcu(ser, log_print=log_cb, verbose=True):
            log_cb("[LDR] Calibration still missing; scanning aborted.")
            return [], [], [], []

    # clear previous batch (for safety)
    LDR_Samp.clear()
    LDR_Angle.clear()

    current_mode = "ldr"
    try:
        # Kick MCU: enter LDR then wake
        ser.reset_input_buffer()
        ser.write(LDR_ENTER_CMD); ser.flush()
        time.sleep(0.05)
        ser.write(LDR_START_CMD); ser.flush()

        nsamp = read_scan_header(ser, timeout_s=timeout_s, log_cb=log_cb, dbg_tag="LDR")
        if nsamp is None or nsamp <= 0:
            return [], [], [], []
        if debug:
            log_cb(f"[LDR RX] Header OK. nsamp={nsamp}  (MCU sent n1/2)")

        ang = []
        d1_cm = []
        d2_cm = []
        comb_cm = []

        missed = 0
        for i in range(nsamp):
            # MCU sends: angle, then (v2<<8)|v1  → two bytes: [MSB=v2][LSB=v1]
            trip = read_exact(ser, 3, max_wait_s=1.5)
            if len(trip) != 3:
                missed += 1
                continue

            a = trip[0]
            if a > 180: a = 180

            # Protocol: angle, LDR1, LDR2  (both are 8-bit ADC slices)
            raw1 = int(trip[1]) # DONT CHANGE!!!!!!!!!!!!!!!!!!       << 2    # LDR1  → upshift to 10-bit scale########################################################
            raw2 = int(trip[2]) #<< DONT CHANGE!!!!!!!!!!!!!!!!!!  2    # LDR2  → upshift to 10-bit scale##########################################################

            if use_lut_flag and lut_ready:
                d1 = LDR_DIST_SCALE * adc_to_cm(raw1, lut1) + LDR_DIST_OFFSET  # כיול מרחק LDR1
                d2 = LDR_DIST_SCALE * adc_to_cm(raw2, lut2) + LDR_DIST_OFFSET  # כיול מרחק LDR2
                #print(d1)
                #print(d2)

            # --- Sensor selection by angle sectors ---
            if a <= SECTOR_LEFT_MAX:
                d = d2
            elif a >= SECTOR_RIGHT_MIN :
                d = d1
            else:
                d = min(d1, d2)  # באזור החפיפה; אפשר לשנות ל-d1 אם תרצה עדיפות ל-LDR1

            # זווית אחת לכל דגימה (תיקון כפילות)
            ang.append(a)

            d1_cm.append(float(d1))
            d2_cm.append(float(d2))
            comb_cm.append(float(d))

            # Debug: print EVERY sample so you can verify GUI input matches your debug script
            if debug:
                log_cb(f"[LDR RX] i={i:03d}  θ={a:3d}  raw1(10b)={raw1:4d} raw2(10b)={raw2:4d}  "
                      f"d1≈{d1:4.0f}cm d2≈{d2:4.0f}cm  comb≈{d:4.0f}cm")

        if missed and debug:
            log_cb(f"[LDR RX] Missed {missed} / {nsamp} triplets (timeouts).")

        # Apply global angle offset for display (after fusion)
        # Angle normalization (NEW): stretch half-scan 0..≈88° → 90..180° + offset
        if ang:
            a_min, a_max = min(ang), max(ang)  # <<< עכשיו a_min/a_max מוגדרים
        else:
            a_min, a_max = 0.0, 180.0
        ang_disp = [map_angle_half(a, a_min, a_max) for a in ang]

        return ang_disp, comb_cm, d1_cm, d2_cm


    except Exception as e:
        log_cb(f"[LDR error] {e}")
        return [], [], [], []
    finally:
        current_mode = None
def combo_scan_once(ser, mask_cm, log_cb=print, timeout_s=8.0, debug=False):
    """
    פרוטוקול מצב משולב:
      PC -> MCU: '4' (כניסה למצב) ואז 'X' (התחלה)
      MCU -> PC: 0xFF, u16(nsamp), ואז לכל דגימה 6 בתים:
                 [0]=angle, [1]=dist LSB, [2]=dist MSB, [3]=angle(אותו ערך),
                 [4]=LDR1,  [5]=LDR2
    מחזיר: (angles_disp, obj_cm, ldr1_cm, ldr2_cm, ldr_comb_cm, objs_found, lights_found)
    """
    if ser is None:
        log_cb("[COMBO] Not connected.")
        return [], [], [], [], [], [], []

    # לוודא LUT אם נדרש
    if use_lut_flag and not lut_ready:
        log_cb("[COMBO] LUTs not loaded. Attempting fetch...")
        if not fetch_calibration_from_mcu(ser, log_print=log_cb, verbose=True):
            log_cb("[COMBO] Calibration still missing; aborted.")
            return [], [], [], [], [], [], []

    try:
        ser.reset_input_buffer()
        ser.write(b"4"); ser.flush()
        time.sleep(0.05)
        ser.write(b"X"); ser.flush()

        # הסבר הסדר של הפריים
        log_cb("[PROTO] Sample = angle, dist_LSB, dist_MSB, angle(dup), LDR1, LDR2")

        nsamp = read_scan_header(ser, timeout_s=timeout_s, log_cb=log_cb, dbg_tag="COMBO")
        if nsamp is None or nsamp <= 0:
            return [], [], [], [], [], [], []
        if debug:
            log_cb(f"[COMBO RX] Header OK. nsamp={nsamp}")

        ang_raw, obj_cm, d1_cm, d2_cm, comb_cm = [], [], [], [], []
        missed = 0

        for i in range(nsamp):
            pkt = read_exact(ser, 6, max_wait_s=1.2)
            if len(pkt) != 6:
                missed += 1
                continue

            a1 = min(180, int(pkt[0]))
            dist = int(pkt[1]) | (int(pkt[2]) << 8)
            a2 = min(180, int(pkt[3]))
            raw1 = int(pkt[4])
            raw2 = int(pkt[5])

            if debug and a1 != a2:
                log_cb(f"[WARN] angle dup mismatch: {a1} vs {a2}")

            d_cm = to_cm(dist)
            if use_lut_flag and lut_ready:
                d1 = LDR_DIST_SCALE * adc_to_cm(raw1, lut1) + LDR_DIST_OFFSET
                d2 = LDR_DIST_SCALE * adc_to_cm(raw2, lut2) + LDR_DIST_OFFSET
            else:
                d1, d2 = float(raw1), float(raw2)

            # פיוז'ן LDR לפי אזורים (כמו בחלון LDR)
            if a1 <= SECTOR_LEFT_MAX:
                d_comb = d2
            elif a1 >= SECTOR_RIGHT_MIN :
                d_comb = d1
            else:
                d_comb = min(d1, d2)

            ang_raw.append(a1)
            obj_cm.append(float(d_cm))
            d1_cm.append(float(d1))
            d2_cm.append(float(d2))
            comb_cm.append(float(d_comb))

            if debug and i < 10:
                log_cb(f"[COMBO RX] i={i:03d} θ={a1:3d} dist={d_cm:4.0f}  LDR1≈{d1:.1f} LDR2≈{d2:.1f} comb≈{d_comb:.1f}")

        if missed and debug:
            log_cb(f"[COMBO RX] Missed {missed} / {nsamp} packets.")

        if not ang_raw:
            log_cb("[COMBO] No samples.")
            return [], [], [], [], [], [], []

        # מיפוי זווית (אותו כלל כמו LDR חצי-סריקה) + אופסט
        a_min, a_max = min(ang_raw), max(ang_raw)
        ang_disp = [map_angle_half(a, a_min, a_max) for a in ang_raw]

        # זיהוי עצמים לפי מרחקי האולטרסוניק
        log_cb("[COMBO] Detecting objects...")
        objs_all = detect_objects_dist_cm(
            obj_cm, ang_disp,
            base_pct=80,
            min_drop_cm=2.0,
            min_span=3,
            gap_allow=0,
            edge_keep_frac=0.35,
            smooth_w=5,
            side_guard_cm=2.0,
            split_jump_cm=4.0,
            split_consec=2,
            min_obj_dist=8.0,
            min_ang_span_deg=0.6,
            max_valid_cm=400,
            min_valid_cm=5,
            beam_deg=BEAM_DEG_COMP,
            use_tan=True,
            plateau_add=True,
            plateau_slope_eps=0.6,
            plateau_min_len=3,
            plateau_drop_cm=2.5,
            endpoint_relax_deg=5.0,
            endpoint_min_span_deg=0.4,
            edge_refine_slope=True,
            min_width_cm=1.5
        )

        # סינון לפי MASK + אופסט כבר בתוך map_angle_half
        try:
            mask_val = float(mask_cm)
        except Exception:
            mask_val = 50.0

        objs_vis = [ (max(0.0, min(180.0, ac)), dc, max(0.0, wc))
                     for (ac, dc, wc) in objs_all if dc <= mask_val ]

        # זיהוי מקורות אור על העקומה המאוחדת
        lights_vis = pick_lights_two_sides(
            angles=ang_disp, dists=comb_cm, mask_val=mask_val,
            base_pct=LIGHT_BASE_PCT, min_drop_cm=LIGHT_MIN_DROP_CM,
            smooth_w=LIGHT_SMOOTH_W, min_span=2,
            min_sep_deg=LIGHT_MIN_SEP_DEG, side_guard_deg=LIGHT_SIDE_GUARD_DEG,
            center_band_deg=LIGHT_CENTER_BAND_DEG, max_peaks=LIGHT_MAX_SOURCES
        )

        return ang_disp, obj_cm, d1_cm, d2_cm, comb_cm, objs_vis, lights_vis

    except Exception as e:
        log_cb(f"[COMBO error] {e}")
        return [], [], [], [], [], [], []
    finally:
        try:
            ser.reset_input_buffer()
        except Exception:
            pass

def light_objects_combo_window(ser):
    """UI למצב המשולב: 3 רדארים + יצוא CSV + הדפסות דיבוג לאופציה."""
    layout = [
        [sg.Text("Light Sources & Objects – Combined", font=("Segoe UI", 12, "bold"))],
        [sg.Checkbox("Use LUT", key="-USE_LUT-", default=True),
         sg.Text("Mask [cm]:"),
         sg.Slider(range=(10, 400), resolution=1, default_value=80, orientation='h',
                   size=(30, 15), key="-MASK-"),
         sg.Checkbox("Debug prints", key="-DBG-", default=False),
         sg.Checkbox("Also print to Terminal", key="-PRINT-", default=False)],
        [sg.Button("Start Combined Scan", key="-START-"), sg.Button("Back", key="-BACK-")],
        [sg.Multiline(size=(98, 16), key="-LOG-", autoscroll=True, disabled=True)]
    ]
    win = sg.Window("Combined Light & Object System", layout, finalize=True, modal=True)
    log_box = win["-LOG-"]

    def log_cb(msg):
        # לוג לחלון, ואם ביקשו – גם למסוף
        s = msg if msg.endswith("\n") else (msg + "\n")
        log_box.update(value=log_box.get() + s)
        log_box.set_vscroll_position(1.0)
        if bool(win["-PRINT-"].get()):
            try: print(s, end="")
            except Exception: pass

    # שליחת '4' רק כהכנה (כמו ביתר המסכים) כדי שה-MCU יקבל מצב
    try:
        if ser:
            ser.reset_input_buffer()
            ser.write(b"4"); ser.flush()
    except Exception as e:
        log_cb(f"[TX error] {e}")

    while True:
        ev, vals = win.read()
        if ev in (sg.WIN_CLOSED, "-BACK-"):
            ensure_idle(ser)
            break

        if ev == "-START-":
            global use_lut_flag
            use_lut_flag = bool(vals.get("-USE_LUT-", True))
            try:
                mask_val = float(vals.get("-MASK-", "80"))
            except Exception:
                mask_val = 80.0
            debug = bool(vals.get("-DBG-", False))

            # סריקה אחת
            ang, obj_cm, d1_cm, d2_cm, comb_cm, objs_vis, lights_vis = combo_scan_once(
                ser, mask_val, log_cb=log_cb, timeout_s=8.0, debug=debug
            )
            if not ang:
                log_cb("[COMBO] No samples captured.")
                continue

            # לוג תוצאות
            if objs_vis:
                for i, (a, d, w) in enumerate(objs_vis, 1):
                    log_cb(f"[OBJ {i}] θ={a:.1f}°, d={d:.1f} cm, w≈{w:.1f} cm")
            else:
                log_cb("[OBJ] No objects within mask.")

            if lights_vis:
                for i, (a, d) in enumerate(lights_vis, 1):
                    log_cb(f"[LIGHT {i}] θ={a:.1f}°, d≈{d:.1f} cm")
            else:
                log_cb("[LIGHT] No light sources within mask.")

            # === ציורים ===
            try: plt.close('all')
            except Exception: pass

            # 1) רדאר LDR בלבד (כולל פסגות)
            objs_for_ldr_plot = [(a, d, 0.0) for (a, d) in lights_vis]
            show_radar(comb_cm, ang, objs_for_ldr_plot,
                       title="LDR Lights (0–180°)",
                       rmax=mask_val + 5,
                       extra_sets=[(ang, d1_cm, "LDR1 samples"),
                                   (ang, d2_cm, "LDR2 samples")])

            # 2) רדאר Objects בלבד
            show_radar(obj_cm, ang, objs_vis,
                       title="Object Detector (0–180°)",
                       rmax=mask_val + 5,
                       detected_color="blue", legend_label="Objects")

            # 3) רדאר משולב – עצמים + פסגות אור + עקומת LDR מאוחדת
            #    (עצמים בצהוב, פסגות אור X שחור, עקומת LDR כרקע נוסף)
            lights_ang = [a for (a, _) in lights_vis]
            lights_dst = [d for (_, d) in lights_vis]
            # לשים את פסגות האור קודם כדי לקבל מרקר 'x'
            extra = [(lights_ang, lights_dst, "Light peaks"),
                     (ang, comb_cm, "LDR fused curve")]
            show_radar(obj_cm, ang, objs_vis,
                       title="Combined View (Objects + Lights)",
                       rmax=mask_val + 5,
                       extra_sets=extra,
                       detected_color="blue", legend_label="Objects")

            # === יצוא CSV ===
            show_results_table(ang, obj_cm, title="Object scan – Angle vs Distance")
            show_results_table_dual(ang, d1_cm, d2_cm, title="LDR scan – LDR1 & LDR2 arrays")

            ensure_idle(ser)

    win.close()



def light_sources_window(ser):
    """UI for the Light Sources Detector System (LDR protocol with two sensors)."""
    layout = [
        [sg.Text("Light Sources Detector System", font=("Segoe UI", 12, "bold"))],
        [sg.Checkbox("Use LUT", key="-USE_LUT-", default=True),
         sg.Text("Mask [cm]:"),
         sg.Slider(range=(10,400), resolution=1, default_value=50, orientation='h',
                   size=(30,15), key="-MASK-"),
         sg.Checkbox("Debug prints", key="-DBG-", default=False),
         sg.Text("Note: fused curve from LDR1 & LDR2"),
         sg.Checkbox("Also print to Terminal", key="-PRINT-", default=False)],
        [sg.Button("Start LDR Scan", key="-START-"), sg.Button("Back", key="-BACK-")],
        [sg.Multiline(size=(90,16), key="-LOG-", autoscroll=True, disabled=True)]
    ]
    win = sg.Window("Light Sources Detector System", layout, finalize=True, modal=True)
    log_box = win["-LOG-"]

    def log_cb(msg):
        s = msg if msg.endswith("\n") else (msg + "\n")
        log_box.update(value=log_box.get() + s)
        log_box.set_vscroll_position(1.0)
        if bool(win["-PRINT-"].get()):
            try: print(s, end="")
            except Exception: pass

    # הכנה (כמו בחלון ה-OBJ)
    try:
        if ser:
            ser.reset_input_buffer()
            ser.write(LDR_ENTER_CMD)  # b"3"
            ser.flush()
    except Exception as e:
        log_cb(f"[TX error] {e}\n")

    while True:
        ev, vals = win.read()
        if ev in (sg.WIN_CLOSED, "-BACK-"):
            ensure_idle(ser)  # '0' + flush + clear flags
            break

        if ev == "-START-":
            global use_lut_flag
            use_lut_flag = bool(vals.get("-USE_LUT-", True))
            try:
                mask_val = float(vals.get("-MASK-", "50"))
            except Exception:
                mask_val = 50.0

            # 1) סריקת LDR אחת (סינכרונית)
            debug = bool(vals.get("-DBG-", False))
            ang, comb_cm, d1_cm, d2_cm = ldr_scan_once(ser, log_cb=log_cb, timeout_s=8.0, debug=debug)
            if not ang or not comb_cm:
                log_cb("[LDR] No samples captured.")
                continue

            log_cb(f"[LDR] Received {len(ang)} samples. Detecting light sources...")

            # 2) זיהוי עד 3 מקורות אור — אחת מכל צד ועוד אחת "מרכזית" אם קיימת
            lights_visible = pick_lights_two_sides(
                angles=ang,
                dists=comb_cm,
                mask_val=mask_val,
                base_pct=LIGHT_BASE_PCT,
                min_drop_cm=LIGHT_MIN_DROP_CM,
                smooth_w=LIGHT_SMOOTH_W,
                min_span=2,
                min_sep_deg=LIGHT_MIN_SEP_DEG,
                side_guard_deg=LIGHT_SIDE_GUARD_DEG,
                center_band_deg=LIGHT_CENTER_BAND_DEG,
                max_peaks=LIGHT_MAX_SOURCES
            )

            # 5) לוג תוצאות


            if not lights_visible:
                log_cb("[LIGHT] No light sources within mask.")
            else:
                for i, (a, d) in enumerate(lights_visible, 1):
                    log_cb(f"[LIGHT {i}] θ={a:.1f}°, d≈{d:.1f} cm")

            # 6) ציור ותצוגת טבלאות
            try:
                plt.close('all')
            except Exception:
                pass

            objs_for_plot = [(a, d, 0.0) for (a, d) in lights_visible]
            show_radar(comb_cm, ang, objs_for_plot,
                       title="LDR Lights (0–180°)",
                       rmax=mask_val + 5,
                       extra_sets=[(ang, d1_cm, "LDR1 samples"), (ang, d2_cm, "LDR2 samples")],
                       detected_color="yellow", legend_label="Light sources")

            show_results_table_dual(ang, d1_cm, d2_cm, title="LDR scan – LDR1 & LDR2 arrays")

            # ניקוי מצב ה-MCU בסוף הריצה
            ensure_idle(ser)

    win.close()



    # --- NEW: small popup for live Telemetry (mirrors console prints) ---
def telemeter_window(ser, angle):
        """
        Opens a small modal window that mirrors the console's live Telemetry prints.
        Back button closes the popup and sends ASCII '0' to MCU (sleep/post-mode).
        """
        global current_mode

        layout = [
            [sg.Text(f"Telemeter — angle {angle}°", font=("Segoe UI", 11, "bold"))],
            [sg.Multiline(size=(64, 14), key="-TELE_LOG-", autoscroll=True, disabled=True)],
            [sg.Button("Back", key="-BACK-")]
        ]
        win = sg.Window("Telemeter Live", layout, finalize=True, modal=True)
        tee = None
        sys_stdout_orig = sys.stdout

        try:
            # Route prints to the popup (and still keep console output)
            tee = StdoutTee(win["-TELE_LOG-"])
            sys.stdout = tee

            # Kick MCU into Telemetry; listener thread will print lines we mirror here
            if ser and getattr(ser, "is_open", False):
                current_mode = "Telemeter"
                try:
                    ser.reset_input_buffer()
                    ser.write(b"2");
                    ser.flush()
                    ser.write(b"t");
                    ser.flush()
                    ser.write(f"{int(angle):03d}".encode("ascii"));
                    ser.flush()
                    print("[Telemeter] Started. Streaming live samples...\n")
                except Exception as e:
                    print(f"[Telemeter TX error] {e}\n")
            else:
                print("[Telemeter] Not connected.\n")

            # Simple modal loop; listener keeps printing into this box
            while True:
                ev, _ = win.read(timeout=200)
                if ev in (sg.WIN_CLOSED, "-BACK-"):
                    break

        finally:
            # Leave Telemetry cleanly
            try:
                ensure_idle(ser)  # unified: '0' + flush + clear flags
            except Exception:
                pass

            try:
                sys.stdout = sys_stdout_orig
            except Exception:
                pass

            try:
                win.close()
            except Exception:
                pass

def ldr_calibration_window(ser):
    layout = [
        [sg.Text("LDR calibration", font=("Segoe UI", 12, "bold"))],
        [sg.Button("Start MCU Calibration (send '6')", key="-START_CAL-"),
         sg.Button("Fetch Calibration from MCU (K)", key="-FETCH_CAL-"),
         sg.Button("Back", key="-BACK-")],
        [sg.Multiline(size=(90,16), key="-LOG-", autoscroll=True, disabled=True)]
    ]
    win = sg.Window("LDR calibration", layout, finalize=True, modal=True)
    log_box = win["-LOG-"]

    def log_cb(msg):
        log_box.update(value=log_box.get() + (msg if msg.endswith("\n") else msg + "\n"))
        log_box.set_vscroll_position(1.0)

    while True:
        ev, _ = win.read()
        if ev in (sg.WIN_CLOSED, "-BACK-"):
            ensure_idle(ser)
            break
        if ev == "-START_CAL-":
            try:
                ser.reset_input_buffer(); ser.write(b"6"); ser.flush()
                log_cb("[CAL] Sent '6' — MCU entered calibration mode (use PB0/PB1 on board).")
            except Exception as e:
                log_cb(f"[CAL error] {e}")
        elif ev == "-FETCH_CAL-":
            ok = fetch_calibration_from_mcu(ser, log_print=log_cb, verbose=True)
            log_cb("[CAL] LUTs ready." if ok else "[CAL] Fetch failed.")

    win.close()


def classic_tools_window(ser):

    listener_thread = None
    script_thread = None # listens to the script
    live_win = None  # חלון התצוגה החיה
    live_data = None  # המטריצה שמוזנת לטבלה
    file_row = [
        sg.Text("File path:"), sg.Input(key="-FILE-", size=(48,1)),
        sg.FileBrowse(file_types=(("Text", "*.txt"), ("All", "*.*"))),
        sg.Text("Slot:"), sg.Combo(values=[str(i) for i in range(1, 11)],
                                   default_value="1", key="-SLOT-", size=(4,1)),
        sg.Text("Type:"), sg.Combo(values=["Script","Text"], default_value="Script",
                                   key="-FTYPE-", size=(8,1), readonly=True),
        sg.Button("Send File (5 → A..J)", key="-SEND_FILE-"),
        sg.Button("Send 5 + Type only", key="-SEND_TYPE_ONLY-"),
        sg.Button("File Mode TXT", key="-FILE_MODE_TXT-")
    ]
    script_row = [
        sg.Button("Fetch Script Names", key="-FETCH_LIST-"),
        sg.Combo(values=Local_Scripts, key="-SCRIPT_LIST-", size=(24,1), readonly=True),
        sg.Text("Name:"), sg.Input(key="-SCRIPT_NAME-", size=(18,1)),
        sg.Button("Send Script Name", key="-SEND_SCRIPT_NAME-")
    ]
    layout = [
        [sg.Text("Advanced Tools")],
        file_row, script_row,
        [sg.Multiline(size=(100, 22), key="-LOG-", autoscroll=True, disabled=True)],
        [sg.Button("Back")]
    ]

    win = sg.Window("Script/Cal Tools", layout, finalize=True, modal=True)
    tee = StdoutTee(win["-LOG-"]); sys_stdout_orig = sys.stdout; sys.stdout = tee

    def start_listener():
        nonlocal listener_thread
        if listener_thread and listener_thread.is_alive():
            return
        listener_thread = threading.Thread(target=uart_listener, args=(ser,), daemon=True)
        listener_thread.start()

    start_listener()

    def open_live_samples_window():  # <<< ADD
        headings = ["Angle °", "Distance (cm)"]
        data = [[f"{i:3d}", "-"] for i in range(181)]
        layout = [
            [sg.Text("Live Samples (Script Mode)", key="-CUR-", font=("Segoe UI", 12, "bold"))],
            [sg.Table(values=data, headings=headings, key="-TAB-", auto_size_columns=True,
                      justification="center", num_rows=20, alternating_row_color="#f0f0f0",
                      enable_events=False, expand_x=True, expand_y=True)],
            [sg.Button("Close", key="-LIVE_CLOSE-")]
        ]
        w = sg.Window("Live Samples", layout, finalize=True, modal=False, resizable=True)
        return w, data

    def script_session_listen():  # <<< ADD: האזנה בין '$' ל־0xFF,1
        try:
            # 1) חכה ל־'$' שמגיע מ-MCU לפני play_script_by_index()
            deadline = time.time() + 3.0
            while not stop_event.is_set() and time.time() < deadline:
                c = ser.read(1)
                if not c:
                    continue
                if c == b"$":
                    win.write_event_value("-LIVE_START-", None)  # signal GUI "started"
                    print("[SCRIPT] Start: MCU entered Script Mode; listening for samples...\n")
                    break
            else:
                print("[SCRIPT] No '$' start — maybe script has no runtime part.\n")
                return

            # 2) קרא דגימות עד סוף Script (0xFF + u16(1))
            samples = 0
            while not stop_event.is_set():
                b0 = ser.read(1)
                if not b0:
                    continue

                # כותרות 0xFF: או "מס' צעדים בסריקה" או סוף סקריפט (value==1)
                if b0 == b"\xFF":
                    ui = read_exact(ser, 2, 0.6)
                    if len(ui) != 2:
                        continue
                    val = ui[0] | (ui[1] << 8)
                    if val == 1:
                        print(f"[SCRIPT] End. Total samples: {samples}\n")
                        win.write_event_value("-LIVE_DONE-", None)
                        break
                    else:
                        print(f"[SCRIPT] Scan header: {val} steps\n")
                        continue

                # פריים דגימה שמתחיל בזווית:
                angle = b0[0]
                b1 = read_exact(ser, 1, 0.2)
                if len(b1) != 1:
                    continue

                # נסה להבחין בין DIST (3 בייטים) ל-LDR (2 בייטים)
                b2 = ser.read(1)
                if b2:
                    # DIST: angle + LSB + MSB (t_diff) -> cm
                    raw = b1[0] | (b2[0] << 8)
                    cm = to_cm(raw)

                    print(f"[DIST] {cm} cm @ θ={angle}\n")
                    win.write_event_value("-LIVE_UPDATE-", (angle, cm))
                else:
                    # LDR: angle + 1 byte (ADC>>2), המרה ל-cm אם LUT קיים
                    raw8 = b1[0]
                    if lut_ready:
                        cm = adc_to_cm(int(raw8) << 2, (lut2 if lut_sensor_sel == 2 else lut1))
                        print(f"[LDR] ≈{cm} cm @ θ={angle} (ADC={raw8})\n")
                        win.write_event_value("-LIVE_UPDATE-", (angle, cm))
                    else:
                        print(f"[LDR] ADC={raw8} @ θ={angle}\n")

                samples += 1
        except Exception as e:
            print(f"[SCRIPT listen error] {e}\n")

    def send_file_with_checksum(slot: str, path: str, ftype: str):
        Mess_Flag = False
        if not ser:
            print("[TX] Not connected."); return
        if not path or not os.path.isfile(path):
            print("[TX] Please choose a valid file."); return
        try:
            if ftype == "Script":
                with open(path, "r", encoding="utf-8") as f:
                    payload = encode_script_text(f.read())
            else:
                with open(path, "rb") as f:
                    payload = f.read()
        except Exception as e:
            print(f"[TX] Failed reading file: {e}"); return
        base = os.path.basename(path)
        base_ascii = base.encode("ascii", "ignore").decode("ascii", "ignore")
        if len(base_ascii) > 16: base_ascii = base_ascii[:16]
        header_name = base_ascii.encode("ascii", "ignore")
        if len(header_name) < 16: header_name = header_name + b' ' * (16 - len(header_name))
        payload_full = header_name + payload
        total = sum(payload_full) & 0xFF
        chk = (-total) & 0xFF
        if len(payload_full) > 2048:
            print(f"[TX] File too big for 2KB limit ({len(payload_full)} bytes)."); return
        try:
            n = int(slot)
        except ValueError:
            print("[TX] Slot must be a number 1..10."); return
        if not (1 <= n <= 10):
            print("[TX] Slot out of range. Choose 1..10."); return
        slot_char = bytes([ord('A') + (n - 1)])
        type_char = b"S" if (ftype == "Script") else b"T"
        try:
            ser.reset_input_buffer()
            ser.write(b"5"); ser.flush(); time.sleep(0.5)
            ser.write(slot_char); ser.flush(); time.sleep(0.3)
            ser.write(type_char); ser.flush(); time.sleep(0.3)
            length = len(payload_full)
            ser.write(bytes([length & 0xFF, (length >> 8) & 0xFF])); ser.flush()
            while Mess_Flag is False:
                for bval in payload_full:
                    ser.write(bytes([bval])); time.sleep(0.02)
                ser.write(bytes([chk])); ser.flush()
                deadline = time.time() + 5.0
                ack = b""
                while time.time() < deadline:
                    c = ser.read(1)
                    if not c:
                        time.sleep(0.05); continue
                    if c == b"$":
                        ack = c; break
                if ack == b"$":
                    kind = "SCRIPT" if type_char == b"S" else "TEXT"
                    print(f"[ACK] MCU confirmed with '$'. Sent {length} bytes + chk {chk:#04x} to slot {n} ({kind}).")
                    if ftype == "Script":
                        if base_ascii not in Local_Scripts:
                            Local_Scripts.append(base_ascii)
                            save_local_scripts()
                            win["-SCRIPT_LIST-"].update(values=Local_Scripts)
                    Mess_Flag = True
                else:
                    print(f"[ACK] Timeout waiting for '$' from MCU."); Mess_Flag = False
        except Exception as e:
            print(f"[TX error] {e}")

    while True:
        ev, vals = win.read(timeout=200)
        # --- Thread-to-UI events for Live window ---
        if ev == "-LIVE_START-":  # <<< ADD: start UI
            try:
                if (live_win is None):
                    live_win, live_data = open_live_samples_window()
                # reset table values
                if live_data is not None:
                    for i in range(181):
                        live_data[i][1] = "-"
                    live_win["-TAB-"].update(values=live_data, select_rows=[])
                live_win["-CUR-"].update("Live Samples — Started.", text_color="#0a7f00")

                try:
                    live_win["-TAB-"].update(disabled=False)
                except Exception:
                    pass
            except Exception:
                pass

        elif ev == "-LIVE_UPDATE-" and live_win is not None and live_data is not None:
            try:
                angle, cm = vals["-LIVE_UPDATE-"]
                if 0 <= angle <= 180:
                    live_data[angle][1] = f"{int(cm)}"
                    live_win["-TAB-"].update(values=live_data, select_rows=[angle])
                    live_win["-CUR-"].update(f"Live Samples (θ={angle}°, d≈{int(cm)} cm)")
            except Exception:
                pass
            continue
        elif ev == "-LIVE_DONE-" and live_win is not None:  # richer "end"
            try:
                live_win["-CUR-"].update("Live Samples — Finished.", text_color="#666666")
                try:
                    live_win["-TAB-"].update(disabled=True)
                except Exception:
                    pass
            except Exception:
                pass
        elif ev == "-LIVE_CLOSE-" and live_win is not None:
            ensure_idle(ser)  # unified cleanup on live window close
            try:
                live_win.close()
            except Exception:
                pass
            finally:
                live_win = None
                live_data = None
            continue

        if ev in (sg.WIN_CLOSED, "Back"):
            ensure_idle(ser)
            break
        elif ev == "-SEND_FILE-":
            send_file_with_checksum(vals.get("-SLOT-","1"), vals.get("-FILE-","").strip(), vals.get("-FTYPE-","Script"))
        elif ev == "-SEND_TYPE_ONLY-":
            if ser:
                try:
                    ser.reset_input_buffer()
                    type_char = b"S" if vals.get("-FTYPE-","Script") == "Script" else b"T"
                    ser.write(b"5"); ser.flush(); time.sleep(0.3)
                    ser.write(type_char); ser.flush()
                    print(f"[TX] Sent only 5 and {type_char.decode()} (no file).")
                except Exception as e:
                    print(f"[TX error] {e}")
        elif ev == "-FILE_MODE_TXT-":
            if ser:
                try:
                    ser.reset_input_buffer()
                    ser.write(b"5"); ser.flush(); time.sleep(0.1)
                    ser.write(b"N"); ser.flush(); time.sleep(0.05)
                    ser.write(b"T"); ser.flush()
                    print("[TX] Sent '5' + 'N' + 'T' (File Mode TXT).")
                except Exception as e:
                    print(f"[TX error] {e}")
            else:
                print("[TX] Not connected.")
        elif ev == "-FETCH_LIST-":
            names = fetch_script_names(None)
            win["-SCRIPT_LIST-"].update(values=names, value=(names[0] if names else ""))
            print(f"[Info] Loaded {len(names)} local script names.")
        elif ev == "-SEND_SCRIPT_NAME-":
            name = (vals.get("-SCRIPT_NAME-","") or "").strip()
            if not name:
                list_sel = vals.get("-SCRIPT_LIST-","")
                if isinstance(list_sel, str):
                    name = list_sel.strip()
            if not name:
                name = ascii_name_from_path(vals.get("-FILE-","").strip())
            if not name:
                print("[TX] Choose or type a script name first.")
            else:
                try:
                    nb = pad16_name(name)
                    if ser:
                        win["-FTYPE-"].update("Script")
                        ser.reset_input_buffer()
                        ser.write(b"5"); ser.flush(); time.sleep(0.1)
                        ser.write(b"N"); ser.flush(); time.sleep(0.05)
                        ser.write(b"S"); ser.flush(); time.sleep(0.05)
                        ser.write(nb); ser.flush()
                        if name not in Local_Scripts:
                            Local_Scripts.append(name)
                            save_local_scripts()
                            win["-SCRIPT_LIST-"].update(values=Local_Scripts)
                        print(f"[TX] Sent '5' + 'N' + 'S' + script name '{name}' ({len(nb)} bytes).")
                        # --- Live window ---
                        if (live_win is None):
                            live_win, live_data = open_live_samples_window()
                        else:
                            try:
                                live_win.TKroot.winfo_exists()
                            except Exception:
                                live_win, live_data = open_live_samples_window()
                                #kick off a Script-listen session
                        if (script_thread is None) or (not script_thread.is_alive()):
                            print("[SCRIPT] Waiting for '$' start marker from MCU...\n")
                            script_thread = threading.Thread(target=script_session_listen, daemon=True)
                            script_thread.start()
                except Exception as e:
                    print(f"[TX error] {e}")
    sys.stdout = sys_stdout_orig
    try:
        ensure_idle(ser)
    except Exception:
        pass
    try:
        if live_win is not None:
            live_win.close()
    except Exception:
        pass
    win.close()

def image_button(path, key, text):
    """
    Modernized menu button:
      • Uses image if present (no border, larger pad)
      • Stylish fallback (accent color) if image missing
    """
    if os.path.isfile(path):
        return sg.Button(
            image_filename=path,
            image_subsample=2,
            key=key,
            border_width=0,
            pad=(8, 10),
            mouseover_colors=("#ffffff", "#1d4ed8"),
        )
    # Fallback — big, comfy accent button
    return sg.Button(
        text,
        key=key,
        size=(24, 3),
        button_color=(ACCENT_TXT, ACCENT_BG),
        mouseover_colors=("#ffffff", "#1d4ed8"),
        border_width=0,
        pad=(8, 10),
    )
def run_gui():
    ser = None
    listener_thread = None

    def connect():
        nonlocal ser, listener_thread
        if ser and ser.is_open:
            return
        try:
            ser = serial.Serial(PORT, BAUD, timeout=0.05)
            ser.reset_input_buffer(); ser.reset_output_buffer()
            stop_event.clear()
            if not (listener_thread and listener_thread.is_alive()):
                listener_thread = threading.Thread(target=uart_listener, args=(ser,), daemon=True)
                listener_thread.start()
            fetch_calibration_from_mcu(ser, log_print=print, verbose=False)
        except Exception as e:
            print(f"[Connect failed] {e}")

    connect()

    btn_dir = os.path.join(os.getcwd(), "Menu Buttons")

    grid = [
        [image_button(os.path.join(btn_dir, "object_detector.png"), "-OBJ-", "Object Detector System"),
         image_button(os.path.join(btn_dir, "telemeter.png"), "-TELEM-", "Telemeter"),
         image_button(os.path.join(btn_dir, "light_sources.png"), "-LDR-", "Light Sources Detector System")],
        [image_button(os.path.join(btn_dir, "combo.png"), "-COMBO-", "Light Sources & Objects Detector System"),
         image_button(os.path.join(btn_dir, "script_mode.png"), "-SCRIPT-", "Script Mode"),
         image_button(os.path.join(btn_dir, "exit.png"), "-EXIT-", "EXIT")],
        [image_button(os.path.join(btn_dir, "ldr_calibration.png"), "-LDR_CAL-", "LDR calibration")],
    ]

    layout = [
        [sg.Text("Light Source & Object Proximity", font=("Segoe UI", 16, "bold"))],
        [sg.Text("Main Menu", font=("Segoe UI", 12), text_color="#94a3b8")],
        [sg.Frame("", grid, border_width=0, relief=sg.RELIEF_FLAT, pad=(0, 4))],
        [sg.Text("Tip: use the Combined scan to see Objects (blue) and Lights (yellow) together.",
                 text_color="#94a3b8", font=("Segoe UI", 9))]
    ]

    window = sg.Window(
        "Light Source and Object Proximity – Main Menu",
        layout,
        finalize=True,
        resizable=True,
        element_justification="center",
    )
    while True:
        ev, _ = window.read()
        if ev in (sg.WIN_CLOSED, "-EXIT-"):
            ensure_idle(ser)  # send '0' + flush + reset local state
            break
        elif ev == "-OBJ-":
            object_detector_window(ser)
        elif ev == "-SCRIPT-":
            classic_tools_window(ser)
        elif ev == "-TELEM-":
            a = sg.popup_get_text("Enter angle (0..180):", default_text="90")
            try:
                ang = int(a) if a is not None else 0
            except Exception:
                ang = 0
            ang = max(0, min(180, ang))
            telemeter_window(ser, ang)
        elif ev == "-LDR-":
            light_sources_window(ser)
        elif ev == "-COMBO-":                        # NEW
            light_objects_combo_window(ser)
        elif ev == "-LDR_CAL-":
            ldr_calibration_window(ser)

    window.close()
    try:
        stop_event.set(); time.sleep(0.05)
        if ser: ser.close()
    except Exception:
        pass

if __name__ == "__main__":
    run_gui()
