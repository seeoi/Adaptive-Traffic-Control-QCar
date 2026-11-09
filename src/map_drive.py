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

VERSION = "2.2.0"

# =============================================================================
# SETTINGS
# =============================================================================
# ---- Car and sensors ----------------------------------------------------------
WHEELBASE = 0.256      # Distance between front and back wheels (m). QCar 2 value.
TACH_SCALE = 1.0       # Real meters per meter that the wheel sensor says. Measured on car 6: 1.00 m -> 1.01 m, OK.
GYRO_SIGN = 1          # +1 if turning LEFT gives a positive gyro value. Measured on car 6: left turn = +92 deg, OK.
# How the wheels really turn on car 6 (--circletest at full lock 0.5, slow):
#   LEFT  -> 0.60 m circle = real angle atan(0.256 / 0.60) = 0.403 rad
#   RIGHT -> 0.84 m circle = real angle atan(0.256 / 0.84) = 0.296 rad
#   (asking for 0.6 gave the same circles: the car's own limit is 0.5.)
# So: real angle = STEER_SCALE * command + STEER_BIAS. The wheels turn only 70 % of what is asked, and
# they are pointed a little to the LEFT even at "0". To get a wanted real angle we send
# (angle - STEER_BIAS) / STEER_SCALE.
STEER_SCALE = 0.70
STEER_BIAS = 0.054     # rad, + = left
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
THROTTLE_BASE = 0.07   # Fixed motor push while driving. This is what gave the smooth lap in lane_follow 1.0.2.
                       # (2.0.3/2.0.4 used a big speed controller: the car stalled, then surged to 0.6 m/s.)
OVERSPEED = 0.15       # Only if the car is this much faster than wanted (m/s), cut the motor.
V_LOST = 0.15          # Drive this slowly when the estimate is far from the path (unsure where we are).
UNSURE_M = 0.10        # "Far from the path" = more than this many meters.
KP_V = 0.10            # Small speed correction (proportional), same as lane_follow.
KI_V = 0.30            # Small speed correction (accumulated), same as lane_follow.
MAX_THROTTLE = 0.11    # Hard limit of the motor command. Never negative (negative = reverse).

# ---- Path following ------------------------------------------------------------
LOOKAHEAD_MIN = 0.22   # Pure pursuit looks this far ahead at low speed (m)...
LOOKAHEAD_GAIN = 0.50  # ...plus this many meters per m/s of speed. (2.1.1: shorter, because a long look-ahead
                       # makes the car cut corners - the right turn cut the corner on the right.)
MIN_RADIUS_LEFT = 0.66   # Left corners are made at least this wide (m). Car 6 at full lock left: 0.60 m.
MIN_RADIUS_RIGHT = 0.92  # Right corners at least this wide (m). Car 6 at full lock right: 0.84 m.
                         # The extra 6-8 cm leaves the steering some room to correct.
                       # Map corners tighter than this are reshaped before driving (see smooth_tight_corners).
CORNER_SHIFT = 0.0     # Optional: move the path this far (m) to the OUTSIDE of curves. Off: in tests on paper it
                       # made the turns tighter at the ends, which our car cannot do.
TIGHT_CURV = 1.0       # A curve tighter than this (1/m, = radius under 1 m) is too tight for our car
                       # (smallest circle about 0.85 m). Then the car looks farther ahead so it starts
TIGHT_EXTRA = 0.0      # turning a bit earlier (m added to the look-ahead) and does not swing wide.
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
ALIGN_HEADING = 0.5    # On a STRAIGHT road, while the camera keeps the car centered, slowly turn the heading
                       # estimate toward the road direction (per second). The gyro itself is very accurate
                       # (one full turn by hand read 359.3 deg), so this only removes the start error.

RUN_TIME = 120         # SAFETY: the program stops by itself after this many seconds.
RATE_HZ = 200
READ_MODE = 0          # 0 = read the sensors NOW (latest values). Mode 1 keeps a queue of old readings:
                       # --motortest showed the wheel speed arriving 2 s LATE with mode 1 (v2.0.6).
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


def curvature_of(points):
    """Curvature (1/m) at every point of a polyline, smoothed over about 10 cm."""
    step = np.hypot(*np.diff(points, axis=0).T)
    d = np.diff(points, axis=0)
    heading = np.arctan2(d[:, 1], d[:, 0])
    heading = np.r_[heading, heading[-1]]
    dh = np.r_[0.0, wrap(np.diff(heading))] / np.maximum(np.r_[1e-3, step], 1e-3)
    return np.convolve(dh, np.ones(9) / 9, mode="same")


def resample(points, spacing=0.01):
    """Put the points back at an even spacing (1 cm)."""
    s = np.r_[0.0, np.cumsum(np.hypot(*np.diff(points, axis=0).T))]
    new_s = np.arange(0.0, s[-1], spacing)
    if s[-1] - new_s[-1] < spacing / 2:
        new_s[-1] = s[-1]                # Last point is very close to the end: move it to the end.
    else:
        new_s = np.r_[new_s, s[-1]]      # Otherwise add the end point.
    return np.c_[np.interp(new_s, s, points[:, 0]), np.interp(new_s, s, points[:, 1])]


def shift_to_outside(points, shift=CORNER_SHIFT):
    """Move the path a few cm to the OUTSIDE of every curve (straight parts do not move)."""
    if shift <= 0:
        return points
    # Signed amount: + for a left curve, - for a right curve, 1 at radius 1 m or tighter, 0 on straight road.
    amount = np.clip(curvature_of(points) / 1.0, -1.0, 1.0)
    for _ in range(2):                                         # Blend in and out smoothly (about 60 cm).
        amount = np.convolve(amount, np.ones(31) / 31, mode="same")
    amount[:20] = 0.0
    amount[-20:] = 0.0
    d = np.diff(points, axis=0)
    heading = np.r_[np.arctan2(d[:, 1], d[:, 0]), np.arctan2(d[-1, 1], d[-1, 0])]
    left_normal = np.c_[-np.sin(heading), np.cos(heading)]
    # The outside of a left curve is to the RIGHT (minus the left normal), of a right curve to the LEFT.
    return resample(points - (shift * amount)[:, None] * left_normal)


def smooth_tight_corners(points):
    """Reshape corners that are tighter than the car can turn.

    The map's right turns (about 0.7 m radius) are tighter than our car's smallest circle (about 0.82 m).
    If the car tries to follow them it goes wide and ends on the yellow line. Here each such corner is
    replaced by a circle arc of radius `min_radius` that touches the same entry road and exit road, like a
    driver who starts the turn a little earlier and cuts the corner a little. For a 90 degree corner the
    middle of the turn moves only about 6 cm to the inside. Returns (new_points, biggest_shift_m).
    """
    signed = curvature_of(points)
    # "How much too tight" at each point: > 1 means tighter than the car can turn in that direction.
    curv = np.where(signed > 0, signed * MIN_RADIUS_LEFT, -signed * MIN_RADIUS_RIGHT)
    limit = 1.0
    n = len(points)
    d = np.diff(points, axis=0)
    heading = np.r_[np.arctan2(d[:, 1], d[:, 0]), np.arctan2(d[-1, 1], d[-1, 0])]
    pieces, start, i, shift = [], 0, 0, 0.0
    while i < n:
        if curv[i] <= limit:
            i += 1
            continue
        # Found a corner that is too tight. Walk back and forward to where the road is nearly straight.
        i0 = i
        while i0 > start and np.abs(signed[i0]) > 0.25 and i - i0 < 80:
            i0 -= 1
        i1 = i
        while i1 < n - 1 and (np.abs(signed[i1]) > 0.25 or np.any(curv[i1:min(i1 + 15, n)] > limit)) and i1 - i < 300:
            i1 += 1
        # Go a bit further along the straight roads on both sides (up to 0.5 m), so the bigger arc fits.
        for _ in range(50):
            if i0 > start and abs(signed[i0 - 1]) < 0.3:
                i0 -= 1
            if i1 < n - 2 and abs(signed[i1 + 1]) < 0.3:
                i1 += 1
        turn = wrap(heading[i1] - heading[i0])
        u0 = np.array([np.cos(heading[i0]), np.sin(heading[i0])])
        u1 = np.array([np.cos(heading[i1]), np.sin(heading[i1])])
        cross = u0[0] * u1[1] - u0[1] * u1[0]
        if abs(turn) > np.radians(150) or abs(cross) < 1e-3:
            i = i1 + 1                       # U-turn or roundabout: cannot fix it this way.
            continue
        # Corner point V where the entry road line and the exit road line meet.
        p0, p1 = points[i0], points[i1]
        t0 = ((p1[0] - p0[0]) * u1[1] - (p1[1] - p0[1]) * u1[0]) / cross
        vertex = p0 + t0 * u0
        t1 = float(np.dot(p1 - vertex, u1))
        # Arc of radius R touching both lines: it starts R*tan(turn/2) before the corner point.
        radius = MIN_RADIUS_LEFT if turn > 0 else MIN_RADIUS_RIGHT
        tangent = radius * np.tan(abs(turn) / 2)
        room = min(t0, t1)
        if room <= 0:
            i = i1 + 1
            continue
        if tangent > room:                   # Not enough straight road: use the biggest arc that fits.
            tangent, radius = room, room / np.tan(abs(turn) / 2)
        a_pt = vertex - tangent * u0
        b_pt = vertex + tangent * u1
        # Keep the map points up to the arc start, then the arc, then the map points after the arc end.
        k0 = i0 + int(np.argmin(np.hypot(*(points[i0:i1] - a_pt).T)))
        k1 = i0 + int(np.argmin(np.hypot(*(points[i0:i1 + 1] - b_pt).T)))
        side = np.sign(turn)
        center = a_pt + side * radius * np.array([-u0[1], u0[0]])
        ang0 = np.arctan2(a_pt[1] - center[1], a_pt[0] - center[0])
        steps = max(int(radius * abs(turn) / 0.01), 2)
        angs = ang0 + side * np.linspace(0.0, abs(turn), steps)
        arc = np.c_[center[0] + radius * np.cos(angs), center[1] + radius * np.sin(angs)]
        for q in arc[::5]:
            shift = max(shift, float(np.min(np.hypot(*(points[k0:k1 + 1] - q).T))))
        pieces.append(points[start:k0])
        pieces.append(arc)
        start = k1 + 1
        i = k1 + 1
    pieces.append(points[start:])
    return resample(np.vstack([p for p in pieces if len(p)])), shift


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
        points = shift_to_outside(resample(points))           # 1) optional: open up a bit in curves (off)
        points, shift = smooth_tight_corners(points)          # 2) no corner tighter than the car can turn
        print(f"Path adjusted for our car: tight corners rounded to {MIN_RADIUS_LEFT} m (left) / "
              f"{MIN_RADIUS_RIGHT} m (right), moved up to {shift * 100:.0f} cm.")
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
    qcar = QCar(readMode=READ_MODE, frequency=RATE_HZ)
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


def circle_test(seconds=7.0, lock=MAX_STEER):
    """Drive slow circles at FULL LOCK (left, then right) and print the real circle radius.
    Needs a free floor area of about 2 m x 2 m. Ctrl+C stops."""
    qcar = QCar(readMode=READ_MODE, frequency=RATE_HZ)
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    with qcar:
        bias, n, t0 = 0.0, 0, time.time()
        print("Hold still 1 s (gyro)...")
        while time.time() - t0 < 1.0:
            qcar.read(); bias += float(qcar.gyroscope[2]); n += 1
        bias /= max(n, 1)
        print(f"Steering lock = {lock:.2f} rad")
        for name, steer_cmd in (("LEFT", lock), ("RIGHT", -lock)):
            print(f"--- full lock {name} for {seconds:.0f} s ---")
            dist = turn = 0.0
            t_step = t_prev = t_print = time.time()
            while not stop.is_set() and time.time() - t_step < seconds:
                now = time.time(); dt = min(max(now - t_prev, 1e-5), 0.05); t_prev = now
                qcar.read()
                v = float(qcar.motorTach) * TACH_SCALE
                w = GYRO_SIGN * (float(qcar.gyroscope[2]) - bias)
                if now - t_step > 1.5:                 # Skip the first 1.5 s (speeding up).
                    dist += abs(v) * dt; turn += abs(w) * dt
                throttle = float(np.clip(0.062 + 0.10 * (0.15 - v), 0.0, 0.09))
                qcar.write(throttle=throttle, steering=steer_cmd)
                if now - t_print > 0.5:
                    t_print = now
                    r = dist / turn if turn > 0.05 else float("nan")
                    print(f"  speed={v:.2f} m/s  turn={np.degrees(turn):5.0f} deg  radius so far={r:.2f} m")
                time.sleep(max(0.0, 1.0 / RATE_HZ - (time.time() - now)))
            qcar.write(throttle=0, steering=0)
            if turn > 0.05:
                print(f"RESULT {name}: radius = {dist / turn:.2f} m ({dist:.2f} m for {np.degrees(turn):.0f} deg)")
            time.sleep(1.0)
    print("Done. Send this text.")


def motor_test():
    """WHEELS IN THE AIR (car on a box). Sends a few fixed motor pushes and prints what the wheel sensor says.

    It shows: the smallest push that turns the wheels, the speed for each push, and how long the wheel
    sensor takes to react after the push changes (look at the time column).
    """
    steps = [(0.00, 2.0), (0.06, 3.0), (0.07, 3.0), (0.08, 3.0), (0.00, 3.0)]
    qcar = QCar(readMode=READ_MODE, frequency=RATE_HZ)
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    print("The car must be ON A BOX with the wheels in the air. Starting in 3 seconds...")
    time.sleep(3)
    with qcar:
        t0 = t_print = time.time()
        loops = 0
        for push, seconds in steps:
            t_step = time.time()
            print(f"--- push {push:.2f} for {seconds:.0f} s ---")
            while not stop.is_set() and time.time() - t_step < seconds:
                now = time.time()
                qcar.read()
                qcar.write(throttle=push, steering=0)
                loops += 1
                if now - t_print >= 0.1:
                    hz = loops / (now - t_print)
                    loops, t_print = 0, now
                    print(f"t={now - t0:5.1f}s push={push:.2f} wheel_speed={float(qcar.motorTach):+.3f} m/s loop={hz:.0f}Hz")
                time.sleep(max(0.0, 1.0 / RATE_HZ - (time.time() - now)))
        qcar.write(throttle=0, steering=0)
    print("Done. Send this text.")


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
    qcar = QCar(readMode=READ_MODE, frequency=RATE_HZ)

    idx = 0                    # Index of the closest path point.
    steer = 0.0                # Steering sent to the wheels now.
    speed_cmd = 0.0            # Smoothed speed command.
    speed_integral = 0.0
    loops = 0                  # Loop counter, to print how many loops per second we run.
    throttle = 0.0
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
                i_near = path.index_at(path.s[idx] + 0.6)
                if i_near > idx and np.max(np.abs(path.curv[idx:i_near + 1])) > TIGHT_CURV:
                    lookahead += TIGHT_EXTRA         # Tight corner ahead: start turning earlier.
                target = path.xy[path.index_at(path.s[idx] + lookahead)]
                alpha = wrap(np.arctan2(target[1] - y, target[0] - x) - th)
                wanted_angle = float(np.arctan2(2 * WHEELBASE * np.sin(alpha), lookahead))
                map_steer = (wanted_angle - STEER_BIAS) / STEER_SCALE   # what to send to get that real angle

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
                    # b) Heading: NOT taken from the camera's sideways reading any more (2.0.7). A small fixed
                    #    offset in the camera reading turned into a heading error of several degrees, and
                    #    the car drove straight through the intersection at an angle. Instead: on a straight
                    #    road, when the car is centered and not steering, it must point along the road.
                    i_lo, i_hi = path.index_at(path.s[idx] - 0.4), path.index_at(path.s[idx] + 0.4)
                    straight_road = abs(wrap(path.heading[i_hi] - path.heading[i_lo])) < np.radians(5)
                    if straight_road and abs(lane_error) < 0.10 and abs(steer) < 0.10:
                        th = wrap(th + ALIGN_HEADING * cam_dt * wrap(path.heading[idx] - th))
                    # (No correction ALONG the path: the wheel sensor is accurate to about 1 %, and the old
                    #  along-path correction froze the estimate when the car was off course - v2.0.4.)
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
                if abs(cte) > UNSURE_M:
                    v_target = min(v_target, V_LOST)
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

                # ---- 5) THROTTLE: fixed base push + small correction (like lane_follow) ---
                if v_target <= 0.0:
                    speed_cmd, speed_integral, throttle = 0.0, 0.0, 0.0
                else:
                    if speed_cmd <= 0.0:
                        speed_cmd = max(speed, 0.10)
                    speed_cmd += float(np.clip(v_target - speed_cmd, -DECEL * dt, ACCEL * dt))
                    error = speed_cmd - speed
                    speed_integral = float(np.clip(speed_integral + error * dt, -0.05, 0.05))
                    throttle = float(np.clip(THROTTLE_BASE + KP_V * error + KI_V * speed_integral,
                                             0.0, MAX_THROTTLE))
                    if speed > v_target + OVERSPEED:
                        throttle = 0.0                       # Too fast: let it roll and slow down.
                        speed_integral = min(speed_integral, 0.0)
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
                loops += 1
                if now - t_print > 1.0 / PRINT_HZ:
                    loop_hz = loops / max(now - t_print, 1e-3) if t_print else 0.0
                    loops = 0
                    t_print = now
                    print(f"{now - t_start:6.1f}s {mode:<16} x={x:+.2f} y={y:+.2f} "
                          f"head={np.degrees(th):+5.0f} s={path.s[idx]:5.2f}/{path.length:.1f}m "
                          f"off={cte * 100:+4.0f}cm speed={speed:.2f}/{v_target:.2f} thr={throttle:.3f} "
                          f"steer={steer:+.2f} yellow={yellow_rows}/5 loop={loop_hz:.0f}Hz")
                # Keep a steady loop rate (do not flood the car with commands).
                spare = 1.0 / RATE_HZ - (time.time() - now)
                if spare > 0:
                    time.sleep(spare)
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
    parser.add_argument("--circletest", action="store_true", help="drive slow full-lock circles (2x2 m free floor)")
    parser.add_argument("--lock", type=float, default=MAX_STEER, help="steering for --circletest (rad, max 0.7)")
    parser.add_argument("--motortest", action="store_true", help="WHEELS IN THE AIR: fixed motor pushes, prints wheel speed")
    parser.add_argument("--dry", action="store_true", help="motor OFF: push the car along the route")
    parser.add_argument("--lidar", action="store_true", help="stop for objects in front")
    parser.add_argument("--nocam", action="store_true", help="do not use the camera (map only)")
    parser.add_argument("--vmax", type=float, default=None, help=f"top speed in m/s (default {V_MAX})")
    parser.add_argument("--time", type=int, default=RUN_TIME, help=f"stop after this many seconds ({RUN_TIME})")
    args = parser.parse_args()
    print(f"map_drive.py version {VERSION}")
    if args.vmax is not None:
        V_MAX = float(np.clip(args.vmax, V_MIN_TURN, 0.5))   # Safety: never above 0.5 m/s.
        print(f"Top speed set to {V_MAX:.2f} m/s")

    if args.graph:
        print_graph(SDCSRoadMap(leftHandTraffic=False))
    elif args.odotest:
        odo_test()
    elif args.circletest:
        circle_test(lock=float(np.clip(args.lock, 0.1, 0.7)))
    elif args.motortest:
        motor_test()
    else:
        route = [int(n) for n in args.route.split(",")]
        drive(route, args.laps, args.dry, args.lidar, USE_CAMERA and not args.nocam, args.time)
