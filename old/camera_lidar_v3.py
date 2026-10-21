"""
OLD SCRIPT (kept for reference only - do NOT use for driving).
Camera lane following + LiDAR obstacle stop (file docstring below calls it qcar_drive.py).
Older version of camera_lidar_v4.py. Speed controller can go negative (reverse).
Originally named: TestGimmik3.py
"""
"""
qcar_drive.py - QCar 2 driving with the front CAMERA + the LIDAR. No map, no calibration.

CAMERA  -> lane keeping
    Each frame, on 5 scan rows near the bottom of the image:
      left edge  = the yellow center line
      right edge = where the dark road ends (white curb / island)
      lane center = midpoint  (if one edge is missing, offset from the other
                               by the lane width it has learned)
    Steering = PD controller that keeps the lane center in the middle of the image.
    Also spots a white stop line across the lane -> waits for green.

LIDAR   -> safety + following
    Reads the raw 360 deg scan in a background thread and looks at a narrow
    cone in front of the car:
      closer than D_SLOW -> slow down,   closer than D_STOP -> stop.
    Works for other QCars queued at a light, people, walls.

Usage (on the QCar):
    export PYTHONPATH=~/Documents/Quanser/libraries/python
    python3 qcar_drive.py --lidartest    # no driving: shows which way is "front" for the LiDAR
    python3 qcar_drive.py --camtest      # no driving: saves debug/cam_*.jpg
    python3 qcar_drive.py                # drive.  Ctrl-C to stop.
    python3 qcar_drive.py --nolidar      # drive with camera only
Start the car in the RIGHT lane with the yellow line on its LEFT.
"""
import argparse, csv, os, signal, threading, time
import numpy as np
import cv2

from pal.products.qcar import QCar, QCarCameras, QCarLidar

# ============================================================== SETTINGS ===
# --- speed
V_MAX        = 0.30     # m/s cruise
V_CURVE      = 0.20     # m/s when steering hard
# --- camera
IMG_W, IMG_H = 820, 410
SCAN_ROWS    = (0.68, 0.75, 0.82, 0.89, 0.96)   # fraction of image height (horizon ~0.57)
ROW_WEIGHTS  = (0.30, 0.25, 0.20, 0.15, 0.10)   # far rows count more -> turns earlier
YELLOW_DELTA = 12       # LAB-b above road = yellow (lower = more sensitive)
ROAD_DELTA   = 40       # brightness difference from road that counts as "not road"
EDGE_RUN_PX  = 10       # non-road run this wide = road edge
LANE_HALF0   = (110, 140, 170, 200, 230)        # starting guess, lane half-width px per row
CAM_OFFSET   = 0        # px, + if the car rides too far LEFT in its lane
KP, KD       = 1.0, 0.12
STEER_SIGN   = 1        # -1 only if it steers the wrong way
MAX_STEER    = 0.5      # rad
STEER_TRIM   = 0.0      # rad, + = left
LOST_TIME    = 0.8      # s with no lane before stopping
# --- stop lines / light
USE_STOP_LINES = True
STOP_FILL      = 0.65   # fraction of the lane width that must be white
STOP_ROWS      = (0.86, 0.96)
STOP_IGNORE_S  = 3.0    # after leaving a stop line, ignore lines for this long
# --- lidar
LIDAR_FRONT  = 0.0      # rad: LiDAR angle that points straight ahead (find with --lidartest)
LIDAR_DIR    = 1        # +1 if LiDAR angles increase to the LEFT, -1 if to the RIGHT
CONE_DEG     = 20       # half-width of the "in front" cone
D_STOP       = 0.35     # m: stop if anything is closer than this ahead
D_SLOW       = 0.90     # m: start slowing
MIN_RANGE    = 0.08     # m: ignore readings closer than this (car body)
# --- misc
RUN_TIME   = 300
RATE_HZ    = 200
SAVE_EVERY = 10
DEBUG_DIR  = "debug"


def light_is_green():
    """Traffic light hook. Replace with your adaptive controller / V2I message.
    Default: 8 s green, 8 s red cycle."""
    return (time.time() % 16.0) < 8.0

# ================================================================ CAMERA ===
class LaneDetector:
    def __init__(self):
        self.yx = [None] * len(SCAN_ROWS)        # tracked yellow x per row
        self.half = list(LANE_HALF0)
        self.rows = []                           # debug: (r, yellow, edge, center)

    @staticmethod
    def _runs(b):
        d = np.diff(np.concatenate(([0], b.astype(np.int8), [0])))
        return list(zip(np.flatnonzero(d == 1), np.flatnonzero(d == -1)))

    def update(self, bgr):
        """Returns (lane_error in -1..1 or None, stop_line_seen)."""
        blur = cv2.GaussianBlur(bgr, (5, 5), 0)
        lab = cv2.cvtColor(blur, cv2.COLOR_BGR2LAB)
        L, B = lab[:, :, 0].astype(np.int16), lab[:, :, 2].astype(np.int16)
        patch = (slice(int(0.85 * IMG_H), IMG_H), slice(int(0.40 * IMG_W), int(0.60 * IMG_W)))
        road_L, road_B = np.median(L[patch]), np.median(B[patch])
        yellow = B > road_B + YELLOW_DELTA
        not_road = np.abs(L - road_L) > ROAD_DELTA

        self.rows, errs, ws = [], [], []
        stop_hits = 0
        for k, (frac, w) in enumerate(zip(SCAN_ROWS, ROW_WEIGHTS)):
            r = int(frac * IMG_H)
            # -- yellow line: track the blob nearest last position (start: rightmost in left 60%)
            ys = [(a + b) / 2 for a, b in self._runs(yellow[r]) if b - a >= 4]
            prev = self.yx[k]
            if prev is None:
                cand = [c for c in ys if c < 0.6 * IMG_W]
                Y = max(cand) if cand else None
            else:
                cand = [c for c in ys if abs(c - prev) < 90]
                Y = min(cand, key=lambda c: abs(c - prev)) if cand else None
            self.yx[k] = Y
            # -- stop line check first: is the lane (yellow -> expected right edge) mostly bright?
            is_stop_row = False
            if Y is not None:
                seg = L[r, int(Y) + 10:int(min(Y + 2 * self.half[k], IMG_W)) - 5]
                is_stop_row = seg.size > 20 and np.mean(seg > road_L + ROAD_DELTA) > STOP_FILL
                if is_stop_row and STOP_ROWS[0] <= frac <= STOP_ROWS[1]:
                    stop_hits += 1
            # -- right road edge: first wide non-road run right of the yellow line (or of center)
            start = int(Y + 15) if Y is not None else IMG_W // 2
            E = None
            if not is_stop_row:                  # a stop line is not the road edge
                for a, b in self._runs(not_road[r, start:]):
                    if b - a >= EDGE_RUN_PX:
                        E = start + a
                        break
            # -- lane center
            if Y is not None and E is not None and E - Y > 40:
                self.half[k] = 0.9 * self.half[k] + 0.1 * (E - Y) / 2
                C = (Y + E) / 2
            elif Y is not None:
                C = Y + self.half[k]
            elif E is not None:
                C = E - self.half[k]
            else:
                C = None
            self.rows.append((r, Y, E, C))
            if C is not None:
                errs.append((C - IMG_W / 2 - CAM_OFFSET) / (IMG_W / 2)); ws.append(w)
        err = float(np.average(errs, weights=ws)) if errs else None
        return err, stop_hits >= 1

    def draw(self, bgr, label):
        out = bgr.copy()
        cv2.line(out, (IMG_W // 2, int(0.6 * IMG_H)), (IMG_W // 2, IMG_H), (255, 255, 255), 1)
        for r, Y, E, C in self.rows:
            cv2.line(out, (0, r), (IMG_W, r), (80, 80, 80), 1)
            if Y is not None: cv2.circle(out, (int(Y), r), 6, (0, 200, 255), -1)   # orange = yellow line
            if E is not None: cv2.circle(out, (int(E), r), 6, (255, 0, 0), -1)     # blue = road edge
            if C is not None: cv2.circle(out, (int(C), r), 6, (0, 255, 0), -1)     # green = lane center
        cv2.putText(out, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        return out


def open_camera():
    return QCarCameras(frameWidth=IMG_W, frameHeight=IMG_H, frameRate=30,
                       enableFront=True, enableLeft=False, enableRight=False, enableBack=False)


def grab(cams):
    if cams.csiFront.read():
        img = cams.csiFront.imageData
        if img is not None and img.size:
            return img.copy()
    return None

# ================================================================= LIDAR ===
class LidarWatcher(threading.Thread):
    """Background thread: keeps self.front = nearest distance in the front cone."""
    def __init__(self):
        super().__init__(daemon=True)
        try:
            self.lidar = QCarLidar(numMeasurements=720, rangingDistanceMode=2, interpolationMode=0)
        except TypeError:
            self.lidar = QCarLidar()
        self.front, self.angles, self.dists = np.inf, None, None
        self.ok, self.stop = False, threading.Event()

    def run(self):
        while not self.stop.is_set():
            try:
                self.lidar.read()
                a = np.asarray(self.lidar.angles, float).ravel()
                d = np.asarray(self.lidar.distances, float).ravel()
                if a.size and d.size == a.size:
                    rel = np.degrees(np.arctan2(np.sin(LIDAR_DIR * (a - LIDAR_FRONT)),
                                                np.cos(LIDAR_DIR * (a - LIDAR_FRONT))))
                    sel = (np.abs(rel) < CONE_DEG) & (d > MIN_RANGE)
                    self.front = float(d[sel].min()) if sel.any() else np.inf
                    self.angles, self.dists, self.ok = a, d, True
            except Exception as e:
                print("LiDAR read error:", e); time.sleep(0.2)
            time.sleep(0.01)

    def close(self):
        self.stop.set()
        for n in ("terminate", "close"):
            if hasattr(self.lidar, n):
                try: getattr(self.lidar, n)()
                except Exception: pass
                return


def close_cam(cams):
    for n in ("terminate", "close"):
        if hasattr(cams, n):
            try: getattr(cams, n)()
            except Exception: pass
            return

# ================================================================= MODES ===
def lidartest():
    print("LiDAR test - put a box/hand ~30 cm straight in FRONT of the car. Ctrl-C to quit.")
    w = LidarWatcher(); w.start()
    try:
        while True:
            time.sleep(0.5)
            if not w.ok:
                print("waiting for LiDAR..."); continue
            d = np.where(w.dists > MIN_RANGE, w.dists, np.inf)
            i = int(np.argmin(d))
            print(f"nearest object: {d[i]:.2f} m at LiDAR angle {w.angles[i]:+.2f} rad "
                  f"({np.degrees(w.angles[i]):+.0f} deg) | front-cone distance: {w.front:.2f} m")
    except KeyboardInterrupt:
        pass
    finally:
        w.close()
    print("If the object in front was NOT near LIDAR_FRONT, set LIDAR_FRONT to that angle (rad).")
    print("Then move the object to the car's LEFT: if the angle went UP keep LIDAR_DIR = 1, if DOWN set -1.")


def camtest():
    os.makedirs(DEBUG_DIR, exist_ok=True)
    cams, det, img = open_camera(), LaneDetector(), None
    try:
        t0 = time.time()
        while time.time() - t0 < 3.0:
            f = grab(cams)
            if f is not None:
                img = f; err, stop = det.update(img)
            time.sleep(0.02)
    finally:
        close_cam(cams)
    if img is None:
        print("No camera frame."); return
    err, stop = det.update(img)
    cv2.imwrite(f"{DEBUG_DIR}/cam_raw.jpg", img)
    cv2.imwrite(f"{DEBUG_DIR}/cam_lanes.jpg", det.draw(img, f"err={err} stop={stop}"))
    for r, Y, E, C in det.rows:
        print(f"row y={r}: yellow={Y}  road edge={E}  lane center={None if C is None else int(C)}")
    print(f"lane error: {err}  (negative = lane is LEFT of image center)   stop line: {stop}")
    print(f"Saved {DEBUG_DIR}/cam_raw.jpg and cam_lanes.jpg "
          "(orange = yellow line, blue = road edge, green = lane center)")


def drive(use_lidar):
    os.makedirs(DEBUG_DIR, exist_ok=True)
    quit_ = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: quit_.set())

    det, cams = LaneDetector(), open_camera()
    lidar = LidarWatcher() if use_lidar else None
    if lidar: lidar.start()
    qcar = QCar(readMode=1, frequency=RATE_HZ)

    state, delta, err_prev, ei = "DRIVE", 0.0, 0.0, 0.0
    last_lane, ignore_stop_until, frame_n = time.time(), 0.0, 0
    v_ref, t_prev, t_cam, t0 = 0.0, time.time(), time.time(), time.time()
    log = csv.writer(open(f"run_{time.strftime('%H%M%S')}.csv", "w", newline=""))
    log.writerow(["t", "state", "v", "v_ref", "steer", "lane_err", "lidar_front_m"])
    print("Driving. Ctrl-C to stop.")
    try:
        with qcar:
            while not quit_.is_set() and time.time() - t0 < RUN_TIME:
                now = time.time(); dt = min(max(now - t_prev, 1e-3), 0.05); t_prev = now
                qcar.read()
                v = float(qcar.motorTach)

                # ---------- camera: steering + stop line
                img = grab(cams)
                lane_err = None
                if img is not None:
                    frame_n += 1
                    lane_err, stop_seen = det.update(img)
                    cdt = max(now - t_cam, 1e-3); t_cam = now
                    if lane_err is not None:
                        last_lane = now
                        d_err = (lane_err - err_prev) / cdt
                        delta = float(np.clip(-STEER_SIGN * (KP * lane_err + KD * d_err), -MAX_STEER, MAX_STEER))
                        err_prev = lane_err
                    if USE_STOP_LINES and state == "DRIVE" and stop_seen and now > ignore_stop_until:
                        state = "STOP_LINE"; print("Stop line - waiting for green")
                    if SAVE_EVERY and frame_n % SAVE_EVERY == 0:
                        cv2.imwrite(f"{DEBUG_DIR}/drive_{(frame_n // SAVE_EVERY) % 60:02d}.jpg",
                                    det.draw(img, f"{state} steer={delta:+.2f}"))

                # ---------- state machine -> target speed
                front = lidar.front if lidar else np.inf
                if state == "STOP_LINE":
                    if light_is_green():
                        state, ignore_stop_until = "DRIVE", now + STOP_IGNORE_S
                        print("Green - go")
                    v_ref = 0.0
                else:
                    lost = now - last_lane > LOST_TIME
                    state = "LOST" if lost else "DRIVE"
                    v_ref = 0.0 if lost else (V_CURVE if abs(delta) > 0.25 else V_MAX)
                # LiDAR always has the last word on speed
                if front < D_STOP:
                    v_ref, state = 0.0, state + "+OBSTACLE"
                elif front < D_SLOW:
                    v_ref = min(v_ref, V_MAX * (front - D_STOP) / (D_SLOW - D_STOP))

                # ---------- speed PI -> throttle
                e = v_ref - v
                ei = float(np.clip(ei + e * dt, -0.3, 0.3))
                if v_ref == 0.0 and abs(v) < 0.05:
                    ei = 0.0
                u = float(np.clip(0.1 * e + 1.0 * ei, -0.3, 0.3))
                braking = v_ref < v - 0.05
                qcar.write(throttle=u, steering=delta + STEER_TRIM,
                           LEDs=np.array([0, 0, 0, 0, int(braking), int(braking), 1, 1]))
                if img is not None:
                    log.writerow([f"{now - t0:.2f}", state, f"{v:.2f}", f"{v_ref:.2f}", f"{delta:+.3f}",
                                  "" if lane_err is None else f"{lane_err:+.3f}",
                                  "" if not np.isfinite(front) else f"{front:.2f}"])
            qcar.write(throttle=0, steering=0)
    finally:
        close_cam(cams)
        if lidar: lidar.close()
    print(f"Stopped. Pictures in {DEBUG_DIR}/drive_XX.jpg, log in run_*.csv")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--camtest", action="store_true")
    ap.add_argument("--lidartest", action="store_true")
    ap.add_argument("--nolidar", action="store_true")
    a = ap.parse_args()
    if a.lidartest: lidartest()
    elif a.camtest: camtest()
    else: drive(not a.nolidar)