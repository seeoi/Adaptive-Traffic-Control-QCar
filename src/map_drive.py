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
import contextlib
import csv
import os
import signal
import socket
import threading
import time

import numpy as np

from pal.products.qcar import QCar
from hal.products.mats import SDCSRoadMap

# We reuse the camera lane detector and the LiDAR watcher from lane_follow.py (same folder).
import lane_follow as lf

VERSION = "2.7.8"

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
MAX_STEER = 0.6        # Largest steering command (rad). Quanser's QCar.write allows -0.6..+0.6.
                       # (2.7.5: was 0.5 - in the roundabout the steering sat at 0.50 for 4 s and the car drifted
                       # to the outer edge. More room lets it turn a bit tighter there.)
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
MAX_RESHAPE = 0.10     # Never move the path more than this (m) when rounding a corner (2.6.2: a 61 cm cut
                       # across the roundabout entry sent the car straight off the curve).
TIGHT_CURV = 1.0       # A curve tighter than this (1/m, = radius under 1 m) is too tight for our car
                       # (smallest circle about 0.85 m). Then the car looks farther ahead so it starts
TIGHT_EXTRA = 0.0      # turning a bit earlier (m added to the look-ahead) and does not swing wide.
# LANE TRIMS: small sideways moves of the path in places where the car, seen on the mat, is not centered
# (the LiDAR position and the Quanser map are each a few cm off in some parts of the room).
# (x, y, radius m, heading deg, shift m). Shift + = move LEFT. Only path points inside the circle AND going
# in that direction (+-60 deg) are moved, fading out to 0 at the edge of the circle.
LANE_TRIMS = [
    # 2.7.7: roundabout, south side, where the right street (node 14) joins: the car went a little to the right,
    # over the tip of that street's yellow triangle. Move the path 3 cm left (toward the island) there only.
    (1.65, 3.18, 0.35, 10, 0.03),
    # 2.7.8: top street going EAST into the roundabout and straight out to the central street (route 23,21,16,18,...):
    # a ~150 deg RIGHT turn of radius 0.68 m, but car 7 turns right no tighter than ~0.77 m (steering stuck at -0.60
    # for 6 s, exit 13 cm wide into the other lane). Like a racing line: enter a bit wide (6 cm LEFT just before the
    # turn) so the car starts turning earlier and exits less wide. Simulation: exit 17 -> 10 cm, entry +4 cm.
    (0.25, 4.23, 0.50, -10, 0.06),
]
OFF_PATH_STOP = 0.40   # SAFETY: if the estimate is this far (m) from the path, stop.

# ---- LiDAR position ("GPS") - the official Quanser way to know where the car is ----------------
# QCarGPS compares what the LiDAR sees (walls, tables, chairs of the ROOM) with a scan saved during
# calibration, and gives x, y, heading on the map. It does not drift like wheels + gyro.
# Quanser's own examples use it (vehicle_control.py in Quanser_Academic_Resources).
GPS_CAL_POSE = [0.0, 2.0, -np.pi / 2]  # Where the car stands for --gpscal: x=0, y=2 m, facing -90 deg (see README).
GPS_GAIN = 0.15        # Each LiDAR position moves our estimate this part of the way toward it (0..1).
GPS_GAIN_HEADING = 0.10
GPS_OFFSET = 0.16      # The LiDAR position is this far (m) IN FRONT of the rear axle (our "car position").
                       # Measured on car 7 (2.7.2): without it the car cut every curve to the inside by 5-8 cm.
GPS_SHIFT_Y = 0.08     # Added to every LiDAR y (m). 2.7.6: photos showed the car ~8-10 cm too far NORTH (up on the
                       # map) on every east-west road (top street half out, on the yellow before the crossing and in
                       # the S-curve) while the log said 1-2 cm; --whereami at node 9 also read y 8 cm too low.
                       # Most likely the car was a few cm off along the road at --gpscal. Set 0 after a new --gpscal.
GPS_DELAY = 0.0        # The LiDAR position is this old (s) when it arrives. It is compared with where WE thought
                       # we were that long ago, not now (otherwise in curves it drags the car to the inside).
GPS_GATE = 0.35        # Ignore a LiDAR position farther than this (m) from our estimate (a bad match).
GPS_GATE_HEADING = 25  # ...or a heading more than this many degrees off.

# ---- Camera corrections ----------------------------------------------------------
USE_CAMERA = True      # False = drive with the map only (also: --nocam).
CAM_MIN_ROWS = 2.0     # The camera is "locked" when it sees the yellow line in this many of 5 rows ON AVERAGE...
CAM_LOCK_S = 0.3       # ...for this long. (2.6.3: an average, because on a DASHED yellow line the count jumps
CAM_ROWS_SMOOTH = 0.15 # 1-3-1-3 from frame to frame and the old "3 rows every frame" rule never locked.)
                       # CAM_ROWS_SMOOTH = how fast the average follows each new frame (0..1).
CAM_M_PER_UNIT = 0.36  # Lane error 1.0 = this many meters sideways (lane half = 0.135 m = about 155 pixels).
CAM_AHEAD = 0.40       # The camera measures the lane about this far in front of the car (m).
CAM_GATE = 0.20        # Ignore a camera reading that disagrees with the estimate by more than this (m):
                       # it is probably another yellow line (for example at an intersection).
NO_CAMERA_ZONES = [    # (x, y, radius in m): inside these circles the camera is NOT used (map only).
    (1.576, 3.817, 1.00),  # Roundabout (center from nodes 16/17, lane radius 0.68 m). Its yellow traffic-island
                           # triangles fooled the camera and pushed the estimate off before the roundabout.
    (0.30, 2.20, 0.65),    # Central street north of the intersection + the diagonal to the roundabout: the camera
]                          # disagreed with the map by 7-9 cm there in every loop (2.4.1).
START_FIX_MAX = 0.12   # The start check may move the start position by at most this much (m).
CAM_MAX_CURV = 0.35    # The camera is only used where the road is almost straight (curvature below this, 1/m).
CAM_STEER_MIX = 0.7    # 0 = map steering only, 1 = camera only (when the camera is trusted).
KP_CAM = 0.9           # Camera steering strength (same idea as KP in lane_follow.py).
FIX_LATERAL = 2.5      # How fast the camera corrects the sideways position (per second).
ALIGN_HEADING = 0.5    # On a STRAIGHT road, while the camera keeps the car centered, slowly turn the heading
                       # estimate toward the road direction (per second). The gyro itself is very accurate
                       # (one full turn by hand read 359.3 deg), so this only removes the start error.

LINK_TIMEOUT = 4.0     # SAFETY: stop if the laptop has not confirmed our messages for this many seconds
LINK_MIN_BYTES = 1500  # ...and at least this much text is waiting (slow Wi-Fi alone must not stop the car).
RUN_TIME = 120         # SAFETY: the program stops by itself after this many seconds.
RATE_HZ = 200
READ_MODE = 0          # 0 = read the sensors NOW (latest values). Mode 1 keeps a queue of old readings:
                       # --motortest showed the wheel speed arriving 2 s LATE with mode 1 (v2.0.6).
PRINT_HZ = 5


class LinkWatch:
    """SAFETY: stop the car if the laptop's SSH connection dies.

    When the Wi-Fi drops, Ctrl+C can no longer reach the car, and Linux may not notice the dead connection
    for minutes, so the car kept driving (it happened twice). We print a status line 5 times per second;
    on a live connection the laptop confirms (ACKs) it within milliseconds. Here we look at the SSH
    connection in /proc/net/tcp: if sent data stays unconfirmed for LINK_TIMEOUT seconds, the link is dead.
    Running directly on the car (no SSH) turns this check off.
    """

    def __init__(self):
        self.key = None
        parts = os.environ.get("SSH_CLIENT", "").split()
        if len(parts) >= 3:
            ip, port, local = parts[0], int(parts[1]), int(parts[2])
            ip_hex = "".join(f"{int(b):02X}" for b in reversed(ip.split(".")))   # Linux writes it backwards.
            self.key = (f"{ip_hex}:{port:04X}", f":{local:04X}")
        self.stuck_since = None

    def dead(self, now, timeout):
        if self.key is None:
            return False
        try:
            with open("/proc/net/tcp") as f:
                lines = f.readlines()[1:]
        except OSError:
            return False
        for line in lines:
            cols = line.split()
            if cols[2] == self.key[0] and cols[1].endswith(self.key[1]):
                waiting = int(cols[4].split(":")[0], 16)     # bytes sent but not yet confirmed
                if waiting > 0:
                    self.stuck_since = self.stuck_since or now
                else:
                    self.stuck_since = None
                return (self.stuck_since is not None and now - self.stuck_since > timeout
                        and waiting >= LINK_MIN_BYTES)
        return False


# Each car is a little different (steering, wheel sensor). Measured values per car, found by the car's name
# (the name on its screen / prompt, e.g. qcar-67815). A car not in this list uses the default numbers above;
# measure it with --odotest and --circletest and add it here.
CAR_PROFILES = {
    "qcar-67815": dict(name="car 6", TACH_SCALE=1.0, STEER_SCALE=0.70, STEER_BIAS=0.054,
                       MIN_RADIUS_LEFT=0.66, MIN_RADIUS_RIGHT=0.92),   # left 0.60 m, right 0.84 m at full lock
    "qcar-67807": dict(name="car 7", TACH_SCALE=1.0, STEER_SCALE=0.707, STEER_BIAS=0.032,
                       MIN_RADIUS_LEFT=0.69, MIN_RADIUS_RIGHT=0.84),   # left 0.63 m, right 0.77 m at full lock
}


def apply_car_profile():
    """Load the measured numbers of THIS car (by its name)."""
    host = socket.gethostname()
    profile = CAR_PROFILES.get(host)
    if profile is None:
        print(f"Car '{host}' has no measured profile: using default numbers. "
              f"Run --odotest and --circletest on it and send the results.")
        return
    for key, value in profile.items():
        if key != "name":
            globals()[key] = value
    print(f"Car profile loaded: {profile['name']} ({host}).")


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


def apply_lane_trims(points):
    """Move the path sideways in the LANE_TRIMS places. Returns the new points."""
    d = np.gradient(points, axis=0)
    heading = np.arctan2(d[:, 1], d[:, 0])
    left = np.stack([-np.sin(heading), np.cos(heading)], axis=1)   # unit vector to the LEFT of the path
    out = points.copy()
    for zx, zy, zr, zh, shift in LANE_TRIMS:
        dist = np.hypot(points[:, 0] - zx, points[:, 1] - zy)
        same_way = np.abs(wrap(heading - np.radians(zh))) < np.radians(60)
        weight = np.where((dist < zr) & same_way, 0.5 * (1 + np.cos(np.pi * dist / zr)), 0.0)  # 1 center, 0 edge
        out += (shift * weight)[:, None] * left
    return out


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
        this_shift = 0.0
        for q in arc[::5]:
            this_shift = max(this_shift, float(np.min(np.hypot(*(points[k0:k1 + 1] - q).T))))
        if this_shift > MAX_RESHAPE or k1 <= k0:
            i = i1 + 1                       # This "corner" is really several curves (e.g. the roundabout
            continue                         # entry): rounding it would cut far across the road. Keep the map.
        shift = max(shift, this_shift)
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
        points = apply_lane_trims(points)                      # 3) small fixes for places on this mat
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


def circle_test(seconds=7.0, lock=0.5):
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
def open_gps(calibrate=False):
    """Start the LiDAR position system (QCarGPS)."""
    from pal.products.qcar import QCarGPS
    return QCarGPS(initialPose=GPS_CAL_POSE, calibrate=calibrate)


def gps_calibrate():
    """Save the reference LiDAR scan of the room. The car must stand still at GPS_CAL_POSE."""
    print(f"CALIBRATION: the car must stand at x={GPS_CAL_POSE[0]:.1f} y={GPS_CAL_POSE[1]:.1f}, "
          f"facing {np.degrees(GPS_CAL_POSE[2]):.0f} deg, and NOT move. Keep people away from the car.")
    with open_gps(calibrate=True):
        pass
    print("Calibration done. Now check it with --whereami.")


def read_gps_pose(gps):
    """LiDAR position moved back to the rear axle: (x, y, heading)."""
    gth = float(gps.orientation[2])
    return (float(gps.position[0]) - GPS_OFFSET * np.cos(gth),
            float(gps.position[1]) - GPS_OFFSET * np.sin(gth) + GPS_SHIFT_Y, gth)


def where_am_i(seconds=30):
    """Motor OFF. Print the LiDAR position 5 times per second and the closest map node."""
    roadmap = SDCSRoadMap(leftHandTraffic=False)
    nodes = {}
    for i in range(len(roadmap.nodes)):
        nodes[i] = node_pose(roadmap, i)
    print("Motor is OFF. Put the car on a node (or push it along the road) and watch the numbers.")
    t_end = time.time() + seconds
    with open_gps() as gps:
        t_print = 0.0
        while time.time() < t_end:
            if gps.readGPS() and time.time() - t_print > 0.2:
                t_print = time.time()
                gx, gy, gth = read_gps_pose(gps)
                best = min(nodes, key=lambda n: (nodes[n][0] - gx) ** 2 + (nodes[n][1] - gy) ** 2)
                bx, by, bth = nodes[best]
                print(f"x={gx:+.2f} y={gy:+.2f} heading={np.degrees(gth):+6.1f} deg | closest node {best}: "
                      f"{np.hypot(bx - gx, by - gy) * 100:.0f} cm away, heading differs "
                      f"{np.degrees(wrap(gth - bth)):+.0f} deg")
            time.sleep(0.01)
    print("Done. Send this text.")


def drive(route, laps, dry_run, use_lidar, use_camera, run_time, use_gps=False):
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
    gps = open_gps() if use_gps else None
    gps_ok_at = -10.0          # Last time a LiDAR position was used.
    gps_x = gps_y = None
    history = []               # (time, x, y, heading) of our estimate, to compare with the late LiDAR position.

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
    yellow_avg = 0.0           # Smoothed number of rows that see the yellow line (handles dashed lines).
    t_cam = time.time()        # Time of the last camera frame (to scale the corrections).
    blocked = False
    last_image = time.time() + 1.0
    mode = "MAP"
    t_print = 0.0

    log_file = open(f"map_{time.strftime('%H%M%S')}.csv", "w", newline="")
    log = csv.writer(log_file)
    log.writerow(["t", "x", "y", "heading_deg", "s", "cte", "speed", "v_target", "throttle",
                  "steer", "steer_map", "steer_cam", "lane_error", "yellow_rows", "mode", "lidar_front",
                  "gps_x", "gps_y"])

    try:
        with qcar, (gps if gps is not None else contextlib.nullcontext()):
            # ---- 1) Measure the gyro error while the car is still ------------------
            print("Hold still: measuring the gyro (and checking the start position) for 1.5 seconds...")
            t0, n = time.time(), 0
            start_errors = []
            start_gps = []
            while time.time() - t0 < (3.0 if gps is not None else 1.5):
                qcar.read()
                gyro_bias += float(qcar.gyroscope[2])
                n += 1
                qcar.write(throttle=0, steering=0)
                if gps is not None and gps.readGPS():
                    start_gps.append(read_gps_pose(gps))
                if use_camera:
                    frame = lf.grab_frame(cameras)
                    if frame is not None:
                        err, _, rows = detector.update(frame)
                        if err is not None and rows >= 2:
                            start_errors.append(err)
            gyro_bias /= max(n, 1)
            # START CHECK: the car is placed by hand, so it is rarely exactly on the node. If the start
            # is on a straight road and the camera sees the yellow line well, use it to correct the
            # sideways start position (this is the only place where the camera can see the error
            # before the first curve).
            if gps is not None and len(start_gps) >= 3:
                # LiDAR START: take the car's real start pose, so placing it by hand does not matter much.
                g = np.array(start_gps[len(start_gps) // 2:])
                gx, gy = float(np.median(g[:, 0])), float(np.median(g[:, 1]))
                gth = float(np.arctan2(np.median(np.sin(g[:, 2])), np.median(np.cos(g[:, 2]))))
                print(f"LiDAR start: x={gx:+.2f} y={gy:+.2f} heading={np.degrees(gth):+.0f} deg "
                      f"({np.hypot(gx - x, gy - y) * 100:.0f} cm and {np.degrees(wrap(gth - th)):+.0f} deg "
                      f"from node {route[0]}).")
                if np.hypot(gx - x, gy - y) > 0.5 or abs(wrap(gth - th)) > np.radians(40):
                    print("LiDAR start is too far from the start node: re-calibrate (--gpscal) or move the car. "
                          "Not driving.")
                    return
                x, y, th = gx, gy, gth
            elif gps is not None:
                print("LiDAR start: NO position from the LiDAR. Run --whereami to check. Not driving.")
                return
            elif len(start_errors) >= 5 and abs(path.curv[:path.index_at(0.6)]).max() < CAM_MAX_CURV:
                side = float(np.clip(np.median(start_errors) * CAM_M_PER_UNIT, -START_FIX_MAX, START_FIX_MAX))
                x += side * -np.sin(th)
                y += side * np.cos(th)
                print(f"Start check: the car is {abs(side) * 100:.0f} cm to the "
                      f"{'LEFT' if side > 0 else 'RIGHT'} of the lane center. Start position corrected.")
            else:
                print("Start check: camera could not check the start position (using the node as it is).")
            print("DRY RUN: motor is OFF, push the car along the route." if dry_run
                  else "Driving. Press Ctrl+C to stop.")

            link = LinkWatch()
            if link.key is None:
                print("Link check OFF (not started over SSH).")
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
                # ---- 2b) LiDAR POSITION: pull the estimate toward it (no drift) ------
                if gps is not None and gps.readGPS():
                    gps_x, gps_y, gth = read_gps_pose(gps)
                    # Where did WE think we were when the LiDAR took this position?
                    px, py, pth = x, y, th
                    for t_h, hx, hy, hth in reversed(history):
                        px, py, pth = hx, hy, hth
                        if t_h <= now - GPS_DELAY:
                            break
                    ex, ey, eth = gps_x - px, gps_y - py, wrap(gth - pth)   # how wrong we were then
                    if np.hypot(ex, ey) < GPS_GATE and abs(eth) < np.radians(GPS_GATE_HEADING):
                        x += GPS_GAIN * ex
                        y += GPS_GAIN * ey
                        th = wrap(th + GPS_GAIN_HEADING * eth)
                        # The same fix also applies to the saved past, so the next reading is not counted twice.
                        history = [(t_h, hx + GPS_GAIN * ex, hy + GPS_GAIN * ey, wrap(hth + GPS_GAIN_HEADING * eth))
                                   for t_h, hx, hy, hth in history]
                        gps_ok_at = now
                if gps is not None:
                    history.append((now, x, y, th))
                    while history and history[0][0] < now - 1.0:
                        history.pop(0)
                idx = path.nearest(x, y, idx)
                cte = path.cross_track(x, y, idx)

                # ---- 3) CAMERA: lane error and corrections ------------------------
                image = lf.grab_frame(cameras) if use_camera else None
                camera_steer = None
                if image is not None:
                    last_image = now
                    lane_error, _, yellow_rows = detector.update(image)
                    yellow_avg += CAM_ROWS_SMOOTH * (yellow_rows - yellow_avg)
                    if yellow_avg >= CAM_MIN_ROWS:
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
                in_no_camera_zone = any((x - zx) ** 2 + (y - zy) ** 2 < zr ** 2 for zx, zy, zr in NO_CAMERA_ZONES)
                # The camera is only used on STRAIGHT road (2.3.0). In curves and merges its idea of the
                # lane center was off by 5-10 cm (it guesses the center from the yellow line), while the
                # map + wheels + gyro alone drove the blind parts (intersection, roundabout) very well.
                j_lo, j_hi = path.index_at(path.s[idx] - 0.2), path.index_at(path.s[idx] + 0.3)
                camera_road = np.max(np.abs(path.curv[j_lo:j_hi + 1])) < CAM_MAX_CURV
                if locked and not in_no_camera_zone and camera_road:
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
                if gps is not None:
                    mode += "+GPS" if now - gps_ok_at < 0.5 else ""
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

                # ---- SAFETY: is the laptop still connected? ------------------------
                if link.dead(now, LINK_TIMEOUT):
                    qcar.write(throttle=0, steering=0)
                    print("Lost the connection to the laptop - stopping the car.")
                    break

                # ---- 6) LOG and STATUS -------------------------------------------
                log.writerow([f"{now - t_start:.2f}", f"{x:.3f}", f"{y:.3f}", f"{np.degrees(th):.1f}",
                              f"{path.s[idx]:.2f}", f"{cte:+.3f}", f"{speed:.2f}", f"{v_target:.2f}",
                              f"{throttle:.3f}", f"{steer:+.3f}", f"{map_steer:+.3f}",
                              "" if camera_steer is None else f"{camera_steer:+.3f}",
                              "" if lane_error is None else f"{lane_error:+.3f}", yellow_rows, mode,
                              "" if not np.isfinite(front) else f"{front:.2f}",
                              "" if gps_x is None else f"{gps_x:.3f}", "" if gps_y is None else f"{gps_y:.3f}"])
                loops += 1
                if now - t_print > 1.0 / PRINT_HZ:
                    log_file.flush()            # Save the log now, so it survives if the program is killed.
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
    parser.add_argument("--lock", type=float, default=0.5, help="steering for --circletest (rad, max 0.7)")
    parser.add_argument("--motortest", action="store_true", help="WHEELS IN THE AIR: fixed motor pushes, prints wheel speed")
    parser.add_argument("--dry", action="store_true", help="motor OFF: push the car along the route")
    parser.add_argument("--lidar", action="store_true", help="stop for objects in front")
    parser.add_argument("--gps", action="store_true", help="use the LiDAR position (QCarGPS) - needs --gpscal once")
    parser.add_argument("--gpscal", action="store_true", help="save the LiDAR room scan (car at x=0, y=2, facing -90)")
    parser.add_argument("--whereami", action="store_true", help="motor OFF: print the LiDAR position for 30 s")
    parser.add_argument("--nocam", action="store_true", help="do not use the camera (map only)")
    parser.add_argument("--vmax", type=float, default=None, help=f"top speed in m/s (default {V_MAX})")
    parser.add_argument("--time", type=int, default=RUN_TIME, help=f"stop after this many seconds ({RUN_TIME})")
    args = parser.parse_args()
    print(f"map_drive.py version {VERSION}")
    apply_car_profile()
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
    elif args.gpscal:
        gps_calibrate()
    elif args.whereami:
        where_am_i()
    else:
        route = [int(n) for n in args.route.split(",")]
        if args.gps and args.lidar:
            print("--gps and --lidar both need the LiDAR: using --gps only.")
        drive(route, args.laps, args.dry, args.lidar and not args.gps, USE_CAMERA and not args.nocam,
              args.time, use_gps=args.gps)
