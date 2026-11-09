"""
map_drive.py - Make a QCar 2 drive a ROUTE on the Quanser road map, using the map + wheels + gyro + camera.

THE BIG IDEA (read this first)
------------------------------
1. The road map (SDCSRoadMap, already inside the Quanser library) knows every lane of the mat.
   A route is a list of map nodes, for example 10,4,20,10. The library turns it into a path:
   thousands of points, 1 cm apart, along the CENTER of the correct lane, including every turn
   at intersections and the roundabout.
2. The car keeps track of WHERE it is (x, y, heading) by itself:
      - wheel speed  -> how far it moved,
      - gyroscope    -> how much it turned.
   This is called "dead reckoning". It starts from the first node of the route, so the car must be
   placed on that node, facing the way traffic goes.
3. Dead reckoning slowly drifts. The CAMERA fixes it: when it sees the yellow line well and the car is
   centered in its lane, we know the car is ON the path, so we pull the estimate back onto the path.
4. Steering: "pure pursuit" = look at a point a little ahead on the path and steer toward it.
   Where the yellow line is visible, the camera steering is mixed in, so the car stays centered.
5. Speed: the path tells us where the curves are BEFORE we reach them, so the car slows down smoothly
   before a curve and speeds up after it. No stop-and-go.
6. LiDAR (optional, --lidar): stops for cars or objects in front. It is NOT needed to know the position,
   so cars without LiDAR can use this program too.

HOW TO RUN (on the QCar, or with map.ps1 from the laptop)
--------------------------------------------------------
    export PYTHONPATH=~/Documents/Quanser/libraries/python
    python3 map_drive.py --graph                  # list nodes and where each one can go (no driving)
    python3 map_drive.py --odotest                # motor OFF: push the car by hand, see distance and turn
    python3 map_drive.py --route 9,13 --dry       # motor OFF: push the car along the route, see the position
    python3 map_drive.py --route 9,13             # drive. Ctrl+C stops.
    python3 map_drive.py --route 10,4,20,10 --laps 2

Place the car ON the first node of the route, in its lane, facing the way traffic goes.
"""

import argparse
import csv
import os
import signal
import threading
import time

import numpy as np

from pal.products.qcar import QCar
from hal.products.mats import SDCSRoadMap

# We reuse the camera lane detector and the LiDAR watcher from lane_follow.py (same folder).
import lane_follow as lf

VERSION = "2.0.2"

# =============================================================================
# SETTINGS
# =============================================================================
# ---- Car and sensors ----------------------------------------------------------
WHEELBASE = 0.256      # Distance between front and back wheels (m). QCar 2 value.
TACH_SCALE = 1.0       # Real meters per meter that the wheel sensor says. Measured on car 6: 1.00 m -> 1.01 m, OK.
GYRO_SIGN = 1          # +1 if turning LEFT gives a positive gyro value. Measured on car 6: left turn = +92 deg, OK.
STEER_GAIN = 1.7       # Multiply the steering by this. The real wheels turn LESS than asked: on car 6,
                       # steering 0.40 gave a 1.06 m circle, but the math says 0.60 m (0.256 / tan 0.40).
                       # 0.60 / 1.06 = 0.59, and 1 / 0.59 = 1.7. Raise it if the car turns too WIDE.
STEER_TRIM = 0.0       # Small constant steering offset (rad), positive = left.
MAX_STEER = 0.5        # Largest steering angle (rad).
STEER_RATE = 3.0       # Largest steering change per second (rad/s).

# ---- Speed ---------------------------------------------------------------------
V_MAX = 0.35           # Speed on straight road (m/s).
V_MIN_TURN = 0.18      # Slowest speed used in the tightest turns (m/s).
A_LAT = 0.20           # Allowed sideways acceleration in a curve (m/s^2). Lower = slower in curves.
PREVIEW = 0.80         # How far ahead (m) the car looks for curves to slow down in time.
ACCEL = 0.25           # Speed may grow by at most this much per second.
DECEL = 0.50           # Speed may drop by at most this much per second (stops are immediate).
THROTTLE_FF0 = 0.035   # Motor push needed to just start rolling (feed-forward, from our logs).
THROTTLE_FF1 = 0.17    # Extra push per m/s of speed (feed-forward, from our logs).
KP_V = 0.15            # Speed correction (proportional).
KI_V = 0.30            # Speed correction (accumulated).
MAX_THROTTLE = 0.15    # Hard limit of the motor command. Never negative (negative = reverse).

# ---- Path following ------------------------------------------------------------
LOOKAHEAD_MIN = 0.30   # Pure pursuit looks this far ahead at low speed (m)...
LOOKAHEAD_GAIN = 0.60  # ...plus this many meters per m/s of speed.
OFF_PATH_STOP = 0.40   # SAFETY: if the estimate is this far (m) from the path, stop.

# ---- Camera corrections ----------------------------------------------------------
USE_CAMERA = True      # False = drive with the map only (also: --nocam).
CAM_MIN_ROWS = 3       # The camera is "locked" when it sees the yellow line in this many of 5 rows...
CAM_LOCK_S = 0.3       # ...for this long.
CAM_M_PER_UNIT = 0.36  # Lane error 1.0 = this many meters sideways (lane half = 0.135 m = about 155 pixels).
CAM_AHEAD = 0.40       # The camera measures the lane about this far in front of the car (m).
CAM_GATE = 0.15        # Ignore a camera reading that disagrees with the estimate by more than this (m):
                       # it is probably another yellow line (for example at an intersection).
CAM_STEER_MIX = 0.7    # 0 = map steering only, 1 = camera only (when the camera is trusted).
KP_CAM = 0.9           # Camera steering strength (same idea as KP in lane_follow.py).
FIX_LATERAL = 2.5      # How fast the camera corrects the sideways position (per second).
FIX_HEADING = 2.5      # How fast the camera corrects the heading (per second).
FIX_ALONG = 1.0        # How fast the position ALONG the path is corrected in curves (per second).

RUN_TIME = 120         # SAFETY: the program stops by itself after this many seconds.
RATE_HZ = 200
PRINT_HZ = 5


def wrap(angle):
    """Wrap an angle to -pi..pi."""
    return (angle + np.pi) % (2 * np.pi) - np.pi


# =============================================================================
# MAP: nodes, graph and the path of a route
# =============================================================================
def node_pose(roadmap, node_id):
    """(x, y, heading) of one map node."""
    pose = np.asarray(roadmap.get_node_pose(node_id), dtype=float).squeeze()
    return float(pose[0]), float(pose[1]), float(pose[2])


def turn_name(from_heading, to_heading):
    """Say if going from one heading to another is LEFT, STRAIGHT, RIGHT or a U-TURN."""
    change = np.degrees(wrap(to_heading - from_heading))
    if abs(change) < 45:
        return "STRAIGHT"
    if abs(change) > 135:
        return "U-TURN"
    return "LEFT" if change > 0 else "RIGHT"


def print_graph(roadmap):
    """Print every node and the nodes you can drive to from it (with LEFT / STRAIGHT / RIGHT)."""
    nodes = list(getattr(roadmap, "nodes", []))
    print("node: x, y, heading  ->  where you can go next")
    for i, node in enumerate(nodes):
        x, y, h = node_pose(roadmap, i)
        exits = []
        for edge in getattr(node, "outEdges", []):
            try:
                j = nodes.index(edge.toNode)
                exits.append(f"{j} ({turn_name(h, node_pose(roadmap, j)[2])})")
            except Exception:
                exits.append("?")
        print(f"{i:2d}: ({x:+.2f}, {y:+.2f}) {np.degrees(h):+5.0f} deg  ->  {', '.join(exits) or 'no data'}")


class Path:
    """The route as a list of points, with the heading and curvature at every point."""

    def __init__(self, roadmap, route, laps):
        points = np.asarray(roadmap.generate_path(route), dtype=float)
        if points.ndim != 2 or points.shape[1] < 2:
            raise SystemExit(f"The map could not make a path for route {route}.")
        points = points[:2].T                                   # -> N rows of (x, y)
        # Remove repeated points (they would give a zero distance).
        keep = np.r_[True, np.hypot(*np.diff(points, axis=0).T) > 1e-4]
        points = points[keep]
        self.closed = route[0] == route[-1]                     # Starts and ends on the same node = a loop.
        if self.closed and laps > 1:
            points = np.vstack([points] + [points[1:]] * (laps - 1))
        self.xy = points
        step = np.hypot(*np.diff(points, axis=0).T)
        self.s = np.r_[0.0, np.cumsum(step)]                    # Distance along the path (m).
        self.length = float(self.s[-1])
        d = np.diff(points, axis=0)
        heading = np.arctan2(d[:, 1], d[:, 0])
        self.heading = np.r_[heading, heading[-1]]
        # Curvature = how fast the heading changes per meter (1/m), smoothed over about 10 cm.
        dh = np.r_[0.0, wrap(np.diff(self.heading))] / np.maximum(np.r_[1e-3, step], 1e-3)
        kernel = np.ones(9) / 9
        self.curv = np.convolve(dh, kernel, mode="same")
        self.spacing = self.length / max(len(points) - 1, 1)

    def nearest(self, x, y, index, back_m=0.3, ahead_m=1.0):
        """Closest path point to (x, y), searching only near the current index (so we never jump to
        another lane or another part of the route that crosses this one)."""
        lo = max(0, index - int(back_m / self.spacing))
        hi = min(len(self.xy), index + int(ahead_m / self.spacing) + 1)
        d2 = np.sum((self.xy[lo:hi] - (x, y)) ** 2, axis=1)
        return lo + int(np.argmin(d2))

    def index_at(self, s_value):
        return int(np.clip(np.searchsorted(self.s, s_value), 0, len(self.s) - 1))

    def cross_track(self, x, y, i):
        """Signed distance from the path (m). Positive = the car is LEFT of the path."""
        h = self.heading[i]
        dx, dy = x - self.xy[i, 0], y - self.xy[i, 1]
        return float(-np.sin(h) * dx + np.cos(h) * dy)


# =============================================================================
# TEST MODE: odometer and gyro check (motor OFF)
# =============================================================================
def odo_test(seconds=60):
    """Motor stays OFF. Push the car by hand and watch the distance and the turn angle.

    1) Mark 1 m on the floor with a tape. Push the car straight exactly 1 m.
       TACH_SCALE = 1.00 / (distance printed).
    2) Push it around a LEFT corner (about 90 degrees). The printed turn must be POSITIVE (about +90).
       If it is negative, set GYRO_SIGN = -1.
    """
    qcar = QCar(readMode=1, frequency=RATE_HZ)
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    with qcar:
        print("Hold the car still for 1 second (measuring the gyro)...")
        bias, n, t0 = 0.0, 0, time.time()
        while time.time() - t0 < 1.0:
            qcar.read()
            bias += float(qcar.gyroscope[2])
            n += 1
        bias /= max(n, 1)
        print("Now push the car by hand. Ctrl+C to finish.")
        dist, turn, t_prev, t_print, t_end = 0.0, 0.0, time.time(), 0.0, time.time() + seconds
        while not stop.is_set() and time.time() < t_end:
            qcar.read()
            now = time.time()
            dt = min(max(now - t_prev, 1e-5), 0.05)
            t_prev = now
            dist += abs(float(qcar.motorTach)) * dt
            turn += (float(qcar.gyroscope[2]) - bias) * dt
            qcar.write(throttle=0, steering=0)
            if now - t_print > 0.5:
                t_print = now
                print(f"distance (wheel sensor) = {dist:5.2f} m    turn (gyro) = {np.degrees(turn):+6.1f} deg")
    print(f"FINAL: distance {dist:.2f} m, turn {np.degrees(turn):+.1f} deg")
    print("If you pushed exactly 1.00 m: TACH_SCALE = 1.00 / distance. Left turn must be POSITIVE.")


# =============================================================================
# MAIN: drive the route
# =============================================================================
def drive(route, laps, dry_run, use_lidar, use_camera, run_time):
    roadmap = SDCSRoadMap(leftHandTraffic=False)
    path = Path(roadmap, route, laps)
    x, y, th = node_pose(roadmap, route[0])
    print(f"Route {route}: {path.length:.1f} m, {'loop' if path.closed else 'one way'}"
          f"{f', {laps} laps' if path.closed and laps > 1 else ''}.")
    print(f"Start pose = node {route[0]}: x={x:+.2f} y={y:+.2f} heading={np.degrees(th):+.0f} deg. "
          "The car must be there, in its lane, facing the way traffic goes.")

    os.makedirs(lf.DEBUG_DIR, exist_ok=True)
    quit_flag = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, lambda *_: quit_flag.set())

    detector = lf.LaneDetector() if use_camera else None
    cameras = lf.open_camera() if use_camera else None
    lidar = lf.LidarWatcher() if use_lidar else None
    if lidar:
        lidar.start()
    qcar = QCar(readMode=1, frequency=RATE_HZ)

    idx = 0                    # Index of the closest path point.
    steer = 0.0                # Steering sent to the wheels now.
    speed_cmd = 0.0            # Smoothed speed command.
    speed_integral = 0.0
    gyro_bias = 0.0
    lock_since = None          # Since when the camera sees the yellow line well.
    lane_error = None
    yellow_rows = 0
    t_cam = time.time()        # Time of the last camera frame (to scale the corrections).
    blocked = False
    last_image = time.time() + 1.0
    mode = "MAP"
    t_print = 0.0

    log_file = open(f"map_{time.strftime('%H%M%S')}.csv", "w", newline="")
    log = csv.writer(log_file)
    log.writerow(["t", "x", "y", "heading_deg", "s", "cte", "speed", "v_target", "throttle",
                  "steer", "steer_map", "steer_cam", "lane_error", "yellow_rows", "mode", "lidar_front"])

    try:
        with qcar:
            # ---- 1) Measure the gyro error while the car is still ------------------
            print("Hold still: measuring the gyro for 1 second...")
            t0, n = time.time(), 0
            while time.time() - t0 < 1.0:
                qcar.read()
                gyro_bias += float(qcar.gyroscope[2])
                n += 1
                qcar.write(throttle=0, steering=0)
            gyro_bias /= max(n, 1)
            print("DRY RUN: motor is OFF, push the car along the route." if dry_run
                  else "Driving. Press Ctrl+C to stop.")

            t_start = t_prev = time.time()
            while not quit_flag.is_set() and time.time() - t_start < run_time:
                now = time.time()
                dt = min(max(now - t_prev, 1e-5), 0.05)   # Real time step (never invented).
                t_prev = now

                # ---- 2) DEAD RECKONING: move the estimate with wheels + gyro -------
                qcar.read()
                speed = float(qcar.motorTach) * TACH_SCALE
                yaw_rate = GYRO_SIGN * (float(qcar.gyroscope[2]) - gyro_bias)
                th = wrap(th + yaw_rate * dt)
                x += speed * np.cos(th) * dt
                y += speed * np.sin(th) * dt
                idx = path.nearest(x, y, idx)
                cte = path.cross_track(x, y, idx)

                # ---- 3) CAMERA: lane error and corrections ------------------------
                image = lf.grab_frame(cameras) if use_camera else None
                camera_steer = None
                if image is not None:
                    last_image = now
                    lane_error, _, yellow_rows = detector.update(image)
                    if yellow_rows >= CAM_MIN_ROWS and lane_error is not None:
                        lock_since = lock_since or now
                    else:
                        lock_since = None

                # Map steering = pure pursuit toward a point ahead on the path.
                lookahead = LOOKAHEAD_MIN + LOOKAHEAD_GAIN * max(speed, 0.0)
                target = path.xy[path.index_at(path.s[idx] + lookahead)]
                alpha = wrap(np.arctan2(target[1] - y, target[0] - x) - th)
                map_steer = float(np.arctan2(2 * WHEELBASE * np.sin(alpha), lookahead)) * STEER_GAIN

                locked = lock_since is not None and now - lock_since >= CAM_LOCK_S and lane_error is not None
                trusted = False
                if locked:
                    camera_steer = float(-lf.STEER_SIGN * KP_CAM * lane_error)
                    # Where does the camera say the lane is, compared with where we think we are?
                    # Look at the point CAM_AHEAD in front of the car (that is what the camera sees).
                    ax, ay = x + CAM_AHEAD * np.cos(th), y + CAM_AHEAD * np.sin(th)
                    i_ahead = path.nearest(ax, ay, idx, back_m=0.1, ahead_m=CAM_AHEAD + 0.3)
                    measured = lane_error * CAM_M_PER_UNIT          # + = car is LEFT of the lane center
                    estimated = path.cross_track(ax, ay, i_ahead)
                    innovation = measured - estimated               # How wrong the estimate is (m).
                    trusted = abs(innovation) < CAM_GATE
                if trusted and image is not None:
                    cam_dt = min(now - t_cam, 0.1)
                    h = path.heading[i_ahead]
                    # a) Sideways: move the estimate toward what the camera measured.
                    x += FIX_LATERAL * cam_dt * innovation * -np.sin(h)
                    y += FIX_LATERAL * cam_dt * innovation * np.cos(h)
                    # b) Heading: if the point ahead is off, the car is pointing a bit wrong.
                    th = wrap(th + FIX_HEADING * cam_dt * innovation / CAM_AHEAD)
                    # c) In a curve, the place where the road heading equals our heading is where we are.
                    i_lo, i_hi = path.index_at(path.s[idx] - 0.4), path.index_at(path.s[idx] + 0.4)
                    if abs(wrap(path.heading[i_hi] - path.heading[i_lo])) > np.radians(25):
                        window = slice(i_lo, i_hi + 1)
                        j = i_lo + int(np.argmin(np.abs(wrap(path.heading[window] - th))))
                        x += FIX_ALONG * cam_dt * (path.xy[j, 0] - x)
                        y += FIX_ALONG * cam_dt * (path.xy[j, 1] - y)
                    steer_target = (1 - CAM_STEER_MIX) * map_steer + CAM_STEER_MIX * camera_steer
                    mode = "MAP+CAM"
                else:
                    steer_target = map_steer
                    mode = "MAP"
                if image is not None:
                    t_cam = now
                steer_target = float(np.clip(steer_target, -MAX_STEER, MAX_STEER))
                max_change = STEER_RATE * dt
                steer += float(np.clip(steer_target - steer, -max_change, max_change))

                # ---- 4) SPEED: slow down BEFORE curves (the map shows them) ---------
                i_far = path.index_at(path.s[idx] + PREVIEW)
                worst_curv = float(np.max(np.abs(path.curv[idx:i_far + 1]))) if i_far > idx else 0.0
                v_target = float(np.clip(np.sqrt(A_LAT / max(worst_curv, 1e-3)), V_MIN_TURN, V_MAX))
                remaining = path.length - path.s[idx]
                if remaining < 0.6:                      # End of a one-way route: roll to a stop.
                    v_target = min(v_target, max(remaining / 0.6 * V_MIN_TURN, 0.0))
                if remaining < 0.05:
                    print("End of the route.")
                    break

                # Safety checks: each one can force a stop.
                if use_camera and now - last_image > lf.CAMERA_TIMEOUT:
                    v_target, mode = 0.0, "CAMERA_STOPPED"
                if abs(cte) > OFF_PATH_STOP:
                    v_target, mode = 0.0, "OFF_PATH"
                front = lidar.front if lidar else np.inf
                if lidar:
                    if front < lf.D_STOP:
                        blocked = True
                    elif front > lf.D_GO:
                        blocked = False
                    if blocked:
                        v_target, mode = 0.0, mode + "+OBSTACLE"

                # ---- 5) THROTTLE: smooth speed command + feed-forward + small PI -----
                if v_target <= 0.0:
                    speed_cmd, speed_integral, throttle = 0.0, 0.0, 0.0
                else:
                    if speed_cmd <= 0.0:
                        speed_cmd = max(speed, 0.10)
                    speed_cmd += float(np.clip(v_target - speed_cmd, -DECEL * dt, ACCEL * dt))
                    error = speed_cmd - speed
                    speed_integral = float(np.clip(speed_integral + error * dt, -0.05, 0.05))
                    feed_forward = THROTTLE_FF0 + THROTTLE_FF1 * speed_cmd / TACH_SCALE
                    throttle = float(np.clip(feed_forward + KP_V * error + KI_V * speed_integral,
                                             0.0, MAX_THROTTLE))
                if dry_run:
                    throttle = 0.0
                braking = v_target < speed - 0.05
                qcar.write(throttle=throttle, steering=steer + STEER_TRIM,
                           LEDs=np.array([0, 0, 0, 0, int(braking), int(braking), 1, 1]))

                # ---- 6) LOG and STATUS -------------------------------------------
                log.writerow([f"{now - t_start:.2f}", f"{x:.3f}", f"{y:.3f}", f"{np.degrees(th):.1f}",
                              f"{path.s[idx]:.2f}", f"{cte:+.3f}", f"{speed:.2f}", f"{v_target:.2f}",
                              f"{throttle:.3f}", f"{steer:+.3f}", f"{map_steer:+.3f}",
                              "" if camera_steer is None else f"{camera_steer:+.3f}",
                              "" if lane_error is None else f"{lane_error:+.3f}", yellow_rows, mode,
                              "" if not np.isfinite(front) else f"{front:.2f}"])
                if now - t_print > 1.0 / PRINT_HZ:
                    t_print = now
                    print(f"{now - t_start:6.1f}s {mode:<16} x={x:+.2f} y={y:+.2f} "
                          f"head={np.degrees(th):+5.0f} s={path.s[idx]:5.2f}/{path.length:.1f}m "
                          f"off={cte * 100:+4.0f}cm speed={speed:.2f}/{v_target:.2f} "
                          f"steer={steer:+.2f} yellow={yellow_rows}/5")
            qcar.write(throttle=0, steering=0)
    finally:
        if cameras is not None:
            lf.close_device(cameras)
        if lidar:
            lidar.close()
        log_file.close()
    print("Stopped. Log: map_*.csv")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="QCar 2: drive a route on the road map")
    parser.add_argument("--route", default="10,4,20,10", help="map nodes, comma separated (first = start)")
    parser.add_argument("--laps", type=int, default=1, help="laps for a loop route (first node = last node)")
    parser.add_argument("--graph", action="store_true", help="print the nodes and where each can go, then exit")
    parser.add_argument("--odotest", action="store_true", help="motor OFF: push the car, see distance and turn")
    parser.add_argument("--dry", action="store_true", help="motor OFF: push the car along the route")
    parser.add_argument("--lidar", action="store_true", help="stop for objects in front")
    parser.add_argument("--nocam", action="store_true", help="do not use the camera (map only)")
    parser.add_argument("--time", type=int, default=RUN_TIME, help=f"stop after this many seconds ({RUN_TIME})")
    args = parser.parse_args()
    print(f"map_drive.py version {VERSION}")

    if args.graph:
        print_graph(SDCSRoadMap(leftHandTraffic=False))
    elif args.odotest:
        odo_test()
    else:
        route = [int(n) for n in args.route.split(",")]
        drive(route, args.laps, args.dry, args.lidar, USE_CAMERA and not args.nocam, args.time)
