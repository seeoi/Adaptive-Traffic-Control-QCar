"""
OLD SCRIPT (kept for reference only - do NOT use for driving).
Camera (yellow line) + LiDAR route fused (file docstring below calls it drive_fused.py).
Never tested on the car. Needs a good LiDAR calibration. Speed controller can go negative (reverse).
Originally named: TestGimmik1-2.py
"""
"""
QCar 2: drive a roadmap route using the CAMERA and the LIDAR together.

  Steering
    * CAMERA (primary): keep the yellow center line where it was at startup.
    * LIDAR (backup):   Stanley path tracking on the SDCSRoadMap route. Used when
                        the yellow line is lost (intersections, gaps) or when the
                        LiDAR says the car is > GUARD_M off the route.
  Stopping
    * LIDAR position: when the car reaches a stop line (an intersection approach
      node) and the light for that approach is red, it stops until green.

Startup checklist
  1. LiDAR calibrated once:   python3 drive_fused.py --calibrate
     (car at x=0, y=2 facing -y: center road, lane heading into the light intersection)
  2. Place the car on the FIRST node of the route, centered in its lane,
     yellow line on its left. It stands still ~1.5 s to learn the camera target.
  3. Drive:                   python3 drive_fused.py
                              python3 drive_fused.py --route 10 4 20 10
Ctrl-C stops. Log: run_<time>.csv, camera frames: debug/fused_XX.jpg
"""
import argparse, csv, os, signal, threading, time
import numpy as np
import cv2

from pal.products.qcar import QCar, QCarGPS, QCarCameras
from hal.products.mats import SDCSRoadMap
try:
    from hal.products.qcar import QCarEKF
except ImportError:
    QCarEKF = None
    print("WARNING: QCarEKF not found - using raw LiDAR position.")

# ----------------------------------------------------------------- CONFIG ---
ROUTE            = [10, 4, 20, 10]          # SDCSRoadMap node IDs (Quanser example route)
CALIBRATION_POSE = [0.0, 2.0, -np.pi / 2]

# Stop lines = approach nodes of the traffic-light intersection, and which light phase they obey
STOP_NODES = {12: "NS", 1: "NS", 6: "EW", 9: "EW"}
STOP_ZONE  = 0.50     # m: start slowing this far before the stop node
STOP_DIST  = 0.08     # m: hold when this close
STOP_CLEAR = 0.40     # m: once this far past a stop node, it's "done"

V_CRUISE    = 0.30    # m/s
AXLE_OFFSET = 0.20    # m, center -> front axle
MAX_STEER   = 0.5     # rad
STEER_TRIM  = 0.0     # rad, + = left
K_STANLEY   = 1.0
GUARD_M     = 0.30    # m off route (LiDAR) -> ignore camera, use LiDAR steering

# camera
IMG_W, IMG_H, FPS = 820, 410, 30
YELLOW_DELTA = 12
MIN_BLOB_PX  = 4
SCAN_ROWS    = (0.70, 0.76, 0.82, 0.88, 0.94)
LOOK_ROW     = 1
SEARCH_WIN   = 90
KP_CAM, KD_CAM = 1.2, 0.10
MIN_ROWS     = 2      # need yellow on at least this many rows to trust the camera
CALIB_S      = 1.5

RUN_TIME   = 180
RATE_HZ    = 200
SAVE_EVERY = 10


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi

# ---------------------------------------------------------------- CAMERA ---
def yellow_mask(bgr):
    lab = cv2.cvtColor(cv2.GaussianBlur(bgr, (5, 5), 0), cv2.COLOR_BGR2LAB)
    b = lab[:, :, 2].astype(np.int16)
    road_b = np.median(b[int(0.80 * IMG_H):, int(0.35 * IMG_W):int(0.65 * IMG_W)])
    mask = (b > road_b + YELLOW_DELTA).astype(np.uint8) * 255
    mask[: int(0.62 * IMG_H)] = 0
    return cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))


def blobs(row):
    d = np.diff(np.concatenate(([0], row.astype(np.int8), [0])))
    return list(zip(np.flatnonzero(d == 1), np.flatnonzero(d == -1)))


class YellowTracker:
    def __init__(self):
        self.x = [None] * len(SCAN_ROWS)
        self.found = [None] * len(SCAN_ROWS)

    def update(self, mask):
        for k, frac in enumerate(SCAN_ROWS):
            r = int(frac * IMG_H)
            row = mask[max(r - 2, 0):r + 3].max(axis=0) > 0
            cands = [(a + b) / 2 for a, b in blobs(row) if b - a >= MIN_BLOB_PX]
            prev = self.x[k]
            if prev is None:
                left = [c for c in cands if c < IMG_W * 0.6]
                pick = max(left) if left else None
            else:
                near = [c for c in cands if abs(c - prev) < SEARCH_WIN]
                pick = min(near, key=lambda c: abs(c - prev)) if near else None
            if pick is not None:
                self.x[k] = pick
            self.found[k] = pick
        return self.found

    def reset(self):
        self.x = [None] * len(SCAN_ROWS)


def camera_error(found, target):
    errs, ws = [], []
    for k, (p, t) in enumerate(zip(found, target)):
        if p is not None and t is not None:
            errs.append(p - t); ws.append(1.0 / (1 + abs(k - LOOK_ROW)))
    if len(errs) < MIN_ROWS:
        return None
    return float(np.average(errs, weights=ws)) / (IMG_W / 2)


def annotate(img, found, target, label):
    out = img.copy()
    for k, frac in enumerate(SCAN_ROWS):
        r = int(frac * IMG_H)
        cv2.line(out, (0, r), (IMG_W, r), (90, 90, 90), 1)
        if target and target[k] is not None:
            cv2.drawMarker(out, (int(target[k]), r), (0, 255, 0), cv2.MARKER_CROSS, 14, 2)
        if found[k] is not None:
            cv2.circle(out, (int(found[k]), r), 6, (0, 0, 255), -1)
    cv2.putText(out, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
    return out

# ----------------------------------------------------------- CONTROLLERS ---
class SpeedController:
    def __init__(self, kp=0.1, ki=1.0, u_max=0.3):
        self.kp, self.ki, self.u_max, self.ei = kp, ki, u_max, 0.0

    def update(self, v, v_ref, dt):
        e = v_ref - v
        self.ei = np.clip(self.ei + e * dt, -self.u_max, self.u_max)
        if v_ref == 0.0 and abs(v) < 0.05:
            self.ei = 0.0
        return float(np.clip(self.kp * e + self.ki * self.ei, -self.u_max, self.u_max))


class Stanley:
    def __init__(self, wp, k=K_STANLEY):
        self.wp, self.N, self.k, self.i, self.ect = wp, wp.shape[1], k, 0, 0.0

    def update(self, p, th, v):
        while self.i < self.N - 2:
            a, b = self.wp[:, self.i], self.wp[:, self.i + 1]
            L = np.linalg.norm(b - a)
            if L < 1e-6 or np.dot(p - a, b - a) / L >= L:
                self.i += 1
            else:
                break
        a, b = self.wp[:, self.i], self.wp[:, self.i + 1]
        u = (b - a) / max(np.linalg.norm(b - a), 1e-6)
        self.ect = u[0] * (p[1] - a[1]) - u[1] * (p[0] - a[0])          # + = left of route
        d = wrap(wrap(np.arctan2(u[1], u[0]) - th) + np.arctan2(-self.k * self.ect, max(v, 0.1)))
        return float(np.clip(d, -MAX_STEER, MAX_STEER))

    @property
    def done(self):
        return self.i >= self.N - 2


class FixedTimeLight:
    """Software light. Replace with your adaptive algorithm / V2I client."""
    def __init__(self, g_ns=8.0, g_ew=8.0, all_red=2.0):
        self.t0, self.g_ns, self.g_ew, self.ar = time.time(), g_ns, g_ew, all_red

    def is_green(self, phase):
        t = (time.time() - self.t0) % (self.g_ns + self.g_ew + 2 * self.ar)
        if phase == "NS":
            return t < self.g_ns
        return self.g_ns + self.ar <= t < self.g_ns + self.ar + self.g_ew


class StopLines:
    """Finds the next stop node ahead of the car (same heading, in front)."""
    def __init__(self, roadmap):
        self.poses = {n: np.asarray(roadmap.get_node_pose(n), float).squeeze() for n in STOP_NODES}
        self.cleared = set()

    def check(self, p_front, th):
        """Returns (node, phase, distance) of the stop line ahead, or None."""
        best = None
        for n, (x, y, h) in self.poses.items():
            rel = np.array([x, y]) - p_front
            along = rel[0] * np.cos(h) + rel[1] * np.sin(h)        # + = node still ahead
            side = abs(-rel[0] * np.sin(h) + rel[1] * np.cos(h))
            aligned = abs(wrap(th - h)) < np.pi / 4
            if not aligned or side > 0.25:
                continue
            if along < -STOP_CLEAR:
                self.cleared.discard(n)                             # far past: re-arm for next lap
                continue
            if n in self.cleared or along < -0.05 or along > STOP_ZONE:
                continue
            if best is None or along < best[2]:
                best = (n, STOP_NODES[n], along)
        return best

# ------------------------------------------------------------------ MAIN ---
def open_camera():
    return QCarCameras(frameWidth=IMG_W, frameHeight=IMG_H, frameRate=FPS,
                       enableFront=True, enableLeft=False, enableRight=False, enableBack=False)


def close(obj):
    for name in ("terminate", "close"):
        if hasattr(obj, name):
            try:
                getattr(obj, name)()
            except Exception:
                pass
            return


def calibrate_only():
    print(f"Calibrating LiDAR with car at {CALIBRATION_POSE}. Keep it still...")
    g = QCarGPS(initialPose=CALIBRATION_POSE, calibrate=True)
    close(g)
    print("Done. Place the car on the route's first node and run without --calibrate.")


def drive(route):
    os.makedirs("debug", exist_ok=True)
    roadmap = SDCSRoadMap(leftHandTraffic=False)
    wp = roadmap.generate_path(route)
    if wp is None:
        print(f"No path for route {route}. Try another node sequence."); return
    x0 = np.asarray(roadmap.get_node_pose(route[0]), float).squeeze()

    stop_flag = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop_flag.set())

    stanley, tracker, speed = Stanley(wp), YellowTracker(), SpeedController()
    light, stops = FixedTimeLight(), StopLines(roadmap)

    qcar = QCar(readMode=1, frequency=RATE_HZ)
    gps = QCarGPS(initialPose=CALIBRATION_POSE, calibrate=False)
    ekf = QCarEKF(x_0=x0) if QCarEKF else None
    cams = open_camera()

    x, y, th = x0
    delta, cam_delta, err_prev = 0.0, None, 0.0
    target, calib, found = None, [], [None] * len(SCAN_ROWS)
    mode, waiting, frame_n = "CALIB", None, 0
    t_start = t_prev = t_cam = time.time()
    log_path = f"run_{time.strftime('%H%M%S')}.csv"

    print(f"Route {route}: {wp.shape[1]} waypoints. Learning camera target, keep still...")
    try:
        with qcar, open(log_path, "w", newline="") as f:
            log = csv.writer(f)
            log.writerow(["t", "x", "y", "th", "v", "v_ref", "delta", "mode", "ect", "cam_err", "stop"])
            while not stop_flag.is_set() and time.time() - t_start < RUN_TIME:
                now = time.time(); dt = max(now - t_prev, 1e-3); t_prev = now

                # ---- LiDAR / EKF pose ------------------------------------------------
                qcar.read()
                v = float(qcar.motorTach)
                y_gps = None
                if gps.readGPS():
                    y_gps = np.array([gps.position[0], gps.position[1], gps.orientation[2]])
                if ekf is not None:
                    ekf.update([v, delta], dt, y_gps, qcar.gyroscope[2])
                    x, y, th = ekf.x_hat[0, 0], ekf.x_hat[1, 0], ekf.x_hat[2, 0]
                elif y_gps is not None:
                    x, y, th = y_gps
                p_front = np.array([x, y]) + AXLE_OFFSET * np.array([np.cos(th), np.sin(th)])
                lidar_delta = stanley.update(p_front, th, v)

                # ---- Camera ----------------------------------------------------------
                cam_err = None
                if cams.csiFront.read():
                    img = cams.csiFront.imageData
                    if img is not None and img.size:
                        frame_n += 1
                        found = tracker.update(yellow_mask(img))
                        cdt = max(now - t_cam, 1e-3); t_cam = now
                        if target is None:
                            calib.append(list(found))
                        else:
                            cam_err = camera_error(found, target)
                            if cam_err is not None:
                                cam_delta = float(np.clip(-(KP_CAM * cam_err + KD_CAM * (cam_err - err_prev) / cdt),
                                                          -MAX_STEER, MAX_STEER))
                                err_prev = cam_err
                            else:
                                cam_delta = None
                                tracker.reset()
                        if SAVE_EVERY and frame_n % SAVE_EVERY == 0:
                            cv2.imwrite(f"debug/fused_{(frame_n // SAVE_EVERY) % 60:02d}.jpg",
                                        annotate(img, found, target, f"{mode} d={delta:+.2f}"))

                # ---- Camera target calibration (first CALIB_S seconds, standing still) -
                if target is None:
                    if now - t_start > CALIB_S:
                        cols = list(zip(*calib)) if calib else [[]] * len(SCAN_ROWS)
                        target = [float(np.median([p for p in c if p is not None]))
                                  if any(p is not None for p in c) else None for c in cols]
                        n_ok = sum(t is not None for t in target)
                        print(f"Camera target rows found: {n_ok}/{len(SCAN_ROWS)}"
                              + ("" if n_ok >= MIN_ROWS else " -> camera unused, LiDAR only"))
                        if n_ok < MIN_ROWS:
                            target = [None] * len(SCAN_ROWS)
                        print(f"LiDAR start: x={x:+.2f} y={y:+.2f} hdg={th:+.2f} "
                              f"(route starts at {x0[0]:+.2f}, {x0[1]:+.2f}, {x0[2]:+.2f})")
                    qcar.write(throttle=0, steering=STEER_TRIM)
                    continue

                # ---- Choose steering source -----------------------------------------
                if cam_delta is not None and abs(stanley.ect) < GUARD_M:
                    delta, mode = cam_delta, "CAM"
                else:
                    delta, mode = lidar_delta, "LIDAR"

                # ---- Stop lines / traffic light (LiDAR position) ---------------------
                v_ref = V_CRUISE
                hit = stops.check(p_front, th)
                if hit is not None:
                    n, phase, dist = hit
                    if light.is_green(phase):
                        if waiting == n:
                            print(f"Green at node {n} - go"); waiting = None
                        stops.cleared.add(n)
                    else:
                        if waiting != n:
                            print(f"Red at node {n} ({phase}) - stopping"); waiting = n
                        v_ref = 0.0 if dist < STOP_DIST else V_CRUISE * (dist - STOP_DIST) / (STOP_ZONE - STOP_DIST)
                        mode += "+STOP"
                if stanley.done:
                    v_ref = 0.0

                u = speed.update(v, v_ref, dt)
                braking = v_ref < v - 0.05
                leds = np.array([0, 0, 0, 0, int(braking), int(braking), 1, 1])
                qcar.write(throttle=u, steering=delta + STEER_TRIM, LEDs=leds)

                log.writerow([f"{now - t_start:.3f}", f"{x:.3f}", f"{y:.3f}", f"{th:.3f}", f"{v:.3f}",
                              f"{v_ref:.2f}", f"{delta:.3f}", mode, f"{stanley.ect:.3f}",
                              "" if cam_err is None else f"{cam_err:.3f}", "" if hit is None else hit[0]])
                if stanley.done and v < 0.02:
                    print("Route finished."); break
            qcar.write(throttle=0, steering=0)
    finally:
        close(cams); close(gps)
    print(f"Log: {log_path}   Camera frames: debug/fused_XX.jpg")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--calibrate", action="store_true", help="calibrate LiDAR and exit")
    ap.add_argument("--route", type=int, nargs="+", default=ROUTE, help="node IDs, e.g. 10 4 20 10")
    a = ap.parse_args()
    calibrate_only() if a.calibrate else drive(a.route)