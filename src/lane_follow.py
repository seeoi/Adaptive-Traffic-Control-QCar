"""
lane_follow.py - Make a QCar 2 drive along a lane using its FRONT CAMERA.

THE BIG IDEA (read this first)
------------------------------
1. The camera looks at the road.
2. The program finds the YELLOW center line and the right edge of the road.
3. It computes the middle of the lane and how far the car is from it ("lane error").
4. It steers to push that error toward zero.
5. It sets the motor so the car moves at a slow, steady speed.

The LiDAR is OPTIONAL. If you turn it on (--lidar), the car stops when something
is in front of it.

HOW TO RUN (on the QCar, over SSH)
----------------------------------
    export PYTHONPATH=~/Documents/Quanser/libraries/python   # needed once per terminal

    python3 lane_follow.py --camtest     # takes a photo, the car does NOT move
    python3 lane_follow.py --dry         # runs everything, but the MOTOR STAYS OFF
    python3 lane_follow.py               # drives. Press Ctrl+C to stop.
    python3 lane_follow.py --lidar       # drives and stops if something is ahead
    python3 lane_follow.py --lidartest   # shows which LiDAR angle is "front"

    python3 lane_follow.py --route S,L,R2   # follow a ROUTE (see "ROUTE AND MANEUVERS" below)
    python3 lane_follow.py --turntest L     # do ONE maneuver now, to calibrate it (car on a stop line)

Place the car in the RIGHT lane with the yellow line on its LEFT side.
Do NOT use sudo (it causes a privileges error on our cars).

WHAT CHANGED COMPARED TO THE OLD SCRIPTS (old/ folder)
------------------------------------------------------
* The throttle can NEVER be negative. In the old scripts the speed controller
  could output a negative throttle, which makes the QCar drive in REVERSE.
  That caused the fast forward/backward jerking.
* The throttle is a fixed "base push" plus a small correction, so the car
  starts moving right away (the old one was slow to start).
* Steering is smoothed so the wheels do not twitch.
* Stop lines / traffic light and LiDAR are OFF by default.
"""

import argparse
import csv
import os
import signal
import threading
import time

import cv2
import numpy as np

from pal.products.qcar import QCar, QCarCameras, QCarLidar

# Version of THIS file. It is printed every time the script starts, so you can
# always tell which version is running on the car. Change it with every edit and
# write the change in CHANGELOG.md (see the "Versions" section of the README).
VERSION = "1.1.6"

# =============================================================================
# SETTINGS  (the numbers you will most likely want to change are at the top)
# =============================================================================

# ---- Speed -------------------------------------------------------------------
V_MAX = 0.35           # Target speed on straight road, in meters per second.
V_CURVE = 0.20         # Target speed while turning hard (slower is safer).
THROTTLE_BASE = 0.08   # Fixed motor push (0 to 1) whenever the car should move.
                       # Car too slow? Raise by 0.01. Car too fast? Lower it.
MAX_THROTTLE = 0.15    # Hard limit on the motor command (0 to 1). Never negative.
KP_V = 0.10            # Speed correction strength (proportional part).
KI_V = 0.30            # Speed correction strength (accumulated part).

# ---- Camera ------------------------------------------------------------------
IMG_W, IMG_H = 820, 410   # Camera image size in pixels (width, height).

# We only look at 5 horizontal rows of the image, near the bottom (close to the car).
# Each number is a fraction of the image height: 0.0 = top, 1.0 = bottom.
SCAN_ROWS = (0.68, 0.75, 0.82, 0.89, 0.96)
# How much each row matters. Rows farther away count more, so the car turns earlier.
ROW_WEIGHTS = (0.30, 0.25, 0.20, 0.15, 0.10)

YELLOW_DELTA = 12   # How much "more yellow than the road" a pixel must be. Lower = more sensitive.
ROAD_DELTA = 40     # How much brighter than the road a pixel must be to count as "not road".
EDGE_RUN_PX = 10    # A "not road" stripe must be this wide (pixels) to count as the road edge.
# Starting guess of half the lane width (in pixels) for each row. The program learns the real value.
LANE_HALF0 = (110, 140, 170, 200, 230)
CAM_OFFSET = 0      # Shift the target lane center, in pixels.
                    # Positive number = use it if the car rides too far to the LEFT.

# ---- Steering ----------------------------------------------------------------
KP = 0.9            # Steering strength. Car zig-zags? Lower it (try 0.6).
KD = 0.05           # Damping: reduces wobble.
STEER_SIGN = 1      # Change to -1 ONLY if the wheels turn the wrong way.
MAX_STEER = 0.5     # Largest steering angle allowed, in radians.
STEER_RATE = 3.0    # Largest steering change per second (smooths the movement).
STEER_ALPHA = 0.5   # Smoothing of the camera reading (0 to 1). Lower = smoother but slower.
STEER_TRIM = 0.0    # Small constant steering offset in radians (positive = left).
                    # Use it if the car drifts to one side on a straight road.
LOST_TIME = 3.0     # Seconds without seeing the lane before the car stops.
                    # 3.0 worked on a real car: in a curve the line can disappear for a moment.
CAMERA_TIMEOUT = 0.5  # SAFETY: seconds without ANY camera picture before the car stops at once.
                      # Different from LOST_TIME: here the camera itself went silent.
CAMERA_GIVE_UP = 5.0  # If the camera stays silent this long, end the run (the car is already stopped).

# ---- Stop lines / traffic light (OFF for now) --------------------------------
USE_STOP_LINES = False  # Set True later, when the traffic lights are working.
STOP_FILL = 0.65        # How much of the lane must look white to count as a stop line.
STOP_ROWS = (0.86, 0.96)  # Which image rows are checked for a stop line.
STOP_IGNORE_S = 3.0     # After leaving a stop line, ignore stop lines for this many seconds.

# ---- Route and maneuvers (NEW in 1.1.0) ---------------------------------------
# HOW IT WORKS: on a normal road the car follows the yellow line with the camera.
# At an intersection or roundabout there is no yellow line, so when the car sees a stop
# line it stops, waits for green and then does a short MANEUVER by "dead reckoning":
# it drives a set distance (wheel speed) and turns a set angle (gyroscope), and when
# that is done it goes back to the camera.
#
# The ROUTE is the list of decisions, one per stop line. It repeats forever (loop).
#   S    = go straight          L = turn left          R = turn right
#   RB1, RB2, RB3 = roundabout: take the 1st, 2nd or 3rd exit
# Example:  --route S,L,RB2
ROUTE_TRIGGER = "yellow_end"  # What tells the car "an intersection starts here":
                        #   "yellow_end" = the yellow line ENDS (no white stop lines on our mat).
                        #   "stop_line"  = a white stop line (needs the line to exist).
END_LOST_S = 0.3        # yellow_end: the yellow line must be gone for this long (seconds)...
END_SEEN_S = 2.0        # ...after being seen well (3+ of 5 rows) no more than this many seconds ago.
END_COOLDOWN_S = 6.0    # After a maneuver, ignore new triggers for this long (a curve can hide the line too).
V_MANEUVER = 0.20       # Speed during a maneuver (m/s). Slow = more repeatable.
MIN_STOP_S = 1.0        # Wait at least this long at a stop line, even if the light is green.
ARC_MIN_FRACTION = 0.5  # In a turn, the camera may take over once this fraction of the angle is done...
HANDOVER_ROWS = 4       # ...and it sees the yellow line in at least this many of the 5 rows...
HANDOVER_S = 0.25       # ...for this many seconds in a row. Then the turn ends early.
TURNTEST_FOLLOW_S = 4.0 # --turntest: after the maneuver, follow the line this long to see the handover.
MANEUVER_MAX_S = 12.0   # SAFETY: if a maneuver takes longer than this, give up and use the camera.

# Each maneuver is a list of steps:
#   ("straight", meters)               drive straight this far
#   ("arc", steering_rad, degrees)     steer this much until the car has turned this many degrees
# Steering: POSITIVE = LEFT, NEGATIVE = RIGHT.
# !!! These numbers are FIRST GUESSES. Calibrate them with:  --turntest L   (see README). !!!
MANEUVERS = {
    "S":   [("straight", 1.00)],
    "L":   [("straight", 0.45), ("arc", +0.35, 90), ("straight", 0.10)],
    "R":   [("straight", 0.05), ("arc", -0.50, 90), ("straight", 0.10)],   # -0.50 = tightest turn the QCar can do
    # Roundabout (cars drive counter-clockwise): go in, circle LEFT, leave at the chosen exit.
    "RB1": [("straight", 0.20), ("arc", +0.30, 60),  ("straight", 0.30)],
    "RB2": [("straight", 0.20), ("arc", +0.30, 150), ("straight", 0.30)],
    "RB3": [("straight", 0.20), ("arc", +0.30, 240), ("straight", 0.30)],
}

# ---- LiDAR (only used with --lidar) ------------------------------------------
LIDAR_FRONT = 0.0   # LiDAR angle (radians) that points straight ahead. Find it with --lidartest.
LIDAR_DIR = 1       # +1 if LiDAR angles grow toward the LEFT, -1 if toward the RIGHT.
CONE_DEG = 15       # Only look at objects within this many degrees of straight ahead.
D_STOP = 0.30       # Stop if something is closer than this (meters).
D_GO = 0.40         # Start again only when the road is clear farther than this (meters).
MIN_RANGE = 0.15    # Ignore LiDAR points closer than this (that is the car body).

# ---- Other -------------------------------------------------------------------
RUN_TIME = 60       # SAFETY: the program stops by itself after this many seconds.
                    # Keep it short while testing: if the Wi-Fi drops you cannot press
                    # Ctrl+C anymore, and this timer is what stops the car.
                    # Change it for one run with:  --time 120
RATE_HZ = 200       # How often the QCar interface runs.
PRINT_HZ = 5        # How many status lines per second are printed on screen.
SAVE_EVERY = 0.25   # Save one debug picture every this many seconds (0 = never). 80 pictures = the last 20 s.
DEBUG_DIR = "debug"  # Folder where debug pictures are saved.


def light_is_green():
    """TRAFFIC LIGHT HOOK. Replace this with the real light / V2I message later.

    For now it is a fake light: 8 seconds green, then 8 seconds red, repeating.
    It is only used when USE_STOP_LINES is True.
    """
    return (time.time() % 16.0) < 8.0


# =============================================================================
# CAMERA: finds the lane in the image
# =============================================================================
class LaneDetector:
    """Looks at one camera image and works out where the lane is."""

    def __init__(self):
        # Last x position (in pixels) where we saw the yellow line, one value per scan row.
        self.yellow_x = [None] * len(SCAN_ROWS)
        # Our current estimate of half the lane width, one value per scan row.
        self.half = list(LANE_HALF0)
        # Results of the last image, only used to draw the debug picture.
        self.rows = []

    @staticmethod
    def _runs(mask_row):
        """Find groups of True values in a row. Returns a list of (start, end) pairs.

        Example: [0,1,1,0,1] -> [(1, 3), (4, 5)]
        """
        padded = np.concatenate(([0], mask_row.astype(np.int8), [0]))
        change = np.diff(padded)
        return list(zip(np.flatnonzero(change == 1), np.flatnonzero(change == -1)))

    def update(self, bgr_image):
        """Analyze one image.

        Returns three things:
          lane_error  - number from -1 to 1 (negative = lane is LEFT of the image
                        center, positive = RIGHT), or None if no lane was found.
          stop_seen   - True if a white stop line crosses the lane.
          yellow_rows - how many of the 5 scan rows found the yellow line.
        """
        # Blur a little to remove noise, then convert to the LAB color space.
        # In LAB, "L" is brightness and "B" is how yellow (high) or blue (low) a pixel is.
        blurred = cv2.GaussianBlur(bgr_image, (5, 5), 0)
        lab = cv2.cvtColor(blurred, cv2.COLOR_BGR2LAB)
        brightness = lab[:, :, 0].astype(np.int16)
        yellowness = lab[:, :, 2].astype(np.int16)

        # Measure what the ROAD looks like, using a patch right in front of the car.
        # (We compare against the road because the camera has a pink tint, so fixed
        # color numbers do not work.)
        patch = (slice(int(0.85 * IMG_H), IMG_H),
                 slice(int(0.40 * IMG_W), int(0.60 * IMG_W)))
        road_brightness = np.median(brightness[patch])
        road_yellowness = np.median(yellowness[patch])

        # Yellow pixels = clearly more yellow than the road.
        yellow_mask = yellowness > road_yellowness + YELLOW_DELTA
        # "Not road" pixels = much brighter or darker than the road (curbs, islands, white paint).
        not_road_mask = np.abs(brightness - road_brightness) > ROAD_DELTA

        self.rows = []
        errors, weights = [], []
        stop_hits = 0
        yellow_rows = 0

        for k, (fraction, weight) in enumerate(zip(SCAN_ROWS, ROW_WEIGHTS)):
            row = int(fraction * IMG_H)   # Convert the fraction to a pixel row number.

            # ---- 1) Find the yellow line on this row -------------------------
            candidates = [(a + b) / 2 for a, b in self._runs(yellow_mask[row]) if b - a >= 4]
            previous = self.yellow_x[k]
            if previous is None:
                # First time: pick the right-most yellow blob in the left 60% of the image.
                left_side = [c for c in candidates if c < 0.6 * IMG_W]
                yellow = max(left_side) if left_side else None
            else:
                # Later: pick the blob closest to where the line was last time.
                near = [c for c in candidates if abs(c - previous) < 90]
                yellow = min(near, key=lambda c: abs(c - previous)) if near else None
            self.yellow_x[k] = yellow
            if yellow is not None:
                yellow_rows += 1

            # ---- 2) Is this row a white STOP LINE? ----------------------------
            # A stop line = the whole lane (from the yellow line to the right edge) is bright.
            is_stop_row = False
            if yellow is not None:
                lane_pixels = brightness[row, int(yellow) + 10:int(min(yellow + 2 * self.half[k], IMG_W)) - 5]
                is_stop_row = (lane_pixels.size > 20
                               and np.mean(lane_pixels > road_brightness + ROAD_DELTA) > STOP_FILL)
                if is_stop_row and STOP_ROWS[0] <= fraction <= STOP_ROWS[1]:
                    stop_hits += 1

            # ---- 3) Find the RIGHT edge of the road ---------------------------
            # Start looking just right of the yellow line (or the image middle if no yellow).
            start = int(yellow + 15) if yellow is not None else IMG_W // 2
            edge = None
            if not is_stop_row:   # A stop line is not a road edge, so skip it.
                for a, b in self._runs(not_road_mask[row, start:]):
                    if b - a >= EDGE_RUN_PX:
                        edge = start + a
                        break

            # ---- 4) Compute the CENTER of the lane on this row ----------------
            if yellow is not None and edge is not None and edge - yellow > 40:
                # We see both sides: center is halfway. Also learn the lane width.
                self.half[k] = 0.9 * self.half[k] + 0.1 * (edge - yellow) / 2
                center = (yellow + edge) / 2
            elif yellow is not None:
                center = yellow + self.half[k]   # Only yellow seen: step half a lane to the right.
            elif edge is not None:
                center = edge - self.half[k]     # Only the edge seen: step half a lane to the left.
            else:
                center = None                    # Nothing seen on this row.

            self.rows.append((row, yellow, edge, center))
            if center is not None:
                # Convert to a number from -1 (far left) to +1 (far right).
                errors.append((center - IMG_W / 2 - CAM_OFFSET) / (IMG_W / 2))
                weights.append(weight)

        # Combine all rows into one lane error (weighted average).
        lane_error = float(np.average(errors, weights=weights)) if errors else None
        return lane_error, stop_hits >= 1, yellow_rows

    def draw(self, bgr_image, label):
        """Draw what the detector sees on a copy of the image (for debug pictures).

        Orange dot = yellow line, blue dot = road edge, green dot = lane center.
        """
        out = bgr_image.copy()
        # White vertical line = middle of the image (where the lane center should be).
        cv2.line(out, (IMG_W // 2, int(0.6 * IMG_H)), (IMG_W // 2, IMG_H), (255, 255, 255), 1)
        for row, yellow, edge, center in self.rows:
            cv2.line(out, (0, row), (IMG_W, row), (80, 80, 80), 1)
            if yellow is not None:
                cv2.circle(out, (int(yellow), row), 6, (0, 200, 255), -1)
            if edge is not None:
                cv2.circle(out, (int(edge), row), 6, (255, 0, 0), -1)
            if center is not None:
                cv2.circle(out, (int(center), row), 6, (0, 255, 0), -1)
        cv2.putText(out, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        return out


def open_camera():
    """Start only the FRONT camera."""
    return QCarCameras(frameWidth=IMG_W, frameHeight=IMG_H, frameRate=30,
                       enableFront=True, enableLeft=False, enableRight=False,
                       enableBack=False)


def grab_frame(cameras):
    """Return the newest front-camera image, or None if there is no new image."""
    if cameras.csiFront.read():
        image = cameras.csiFront.imageData
        if image is not None and image.size:
            return image.copy()
    return None


def close_device(device):
    """Safely shut down a camera or LiDAR object (works with different library versions)."""
    for name in ("terminate", "close"):
        if hasattr(device, name):
            try:
                getattr(device, name)()
            except Exception:
                pass
            return


# =============================================================================
# LIDAR: watches for obstacles in front of the car (runs in its own thread)
# =============================================================================
class LidarWatcher(threading.Thread):
    """Background thread that keeps `self.front` = distance (m) to the nearest
    object straight ahead. Other code just reads that one number."""

    def __init__(self):
        super().__init__(daemon=True)
        try:
            self.lidar = QCarLidar(numMeasurements=720, rangingDistanceMode=2,
                                   interpolationMode=0)
        except TypeError:
            self.lidar = QCarLidar()   # Older library versions take no arguments.
        self.front = np.inf     # Distance ahead. inf = nothing seen.
        self.angles = None      # Last scan: angle of every point.
        self.distances = None   # Last scan: distance of every point.
        self.ok = False         # Becomes True after the first good scan.
        self.stop_event = threading.Event()

    def run(self):
        while not self.stop_event.is_set():
            try:
                self.lidar.read()
                angles = np.asarray(self.lidar.angles, float).ravel()
                dists = np.asarray(self.lidar.distances, float).ravel()
                if angles.size and dists.size == angles.size:
                    # Angle of each point relative to "straight ahead", wrapped to -180..180 degrees.
                    shifted = LIDAR_DIR * (angles - LIDAR_FRONT)
                    rel_deg = np.degrees(np.arctan2(np.sin(shifted), np.cos(shifted)))
                    # Keep points inside the front cone and not part of the car body.
                    in_cone = (np.abs(rel_deg) < CONE_DEG) & (dists > MIN_RANGE)
                    self.front = float(dists[in_cone].min()) if in_cone.any() else np.inf
                    self.angles, self.distances, self.ok = angles, dists, True
            except Exception as error:
                print("LiDAR read error:", error)
                time.sleep(0.2)
            time.sleep(0.01)

    def close(self):
        self.stop_event.set()
        close_device(self.lidar)


# =============================================================================
# TEST MODES (the car does not move in any of these)
# =============================================================================
def lidar_test():
    """Print the closest object every half second so you can find the LiDAR's 'front'."""
    print("LiDAR test: hold your hand ~30 cm directly IN FRONT of the car. Ctrl+C to quit.")
    watcher = LidarWatcher()
    watcher.start()
    try:
        while True:
            time.sleep(0.5)
            if not watcher.ok:
                print("waiting for LiDAR...")
                continue
            dists = np.where(watcher.distances > MIN_RANGE, watcher.distances, np.inf)
            i = int(np.argmin(dists))
            print(f"closest object: {dists[i]:.2f} m at {watcher.angles[i]:+.2f} rad "
                  f"({np.degrees(watcher.angles[i]):+.0f} deg) | "
                  f"front cone distance: {watcher.front:.2f} m")
    except KeyboardInterrupt:
        pass
    finally:
        watcher.close()
    print("If your hand was NOT near LIDAR_FRONT, set LIDAR_FRONT to the angle printed above.")


def camera_test():
    """Take a few seconds of images, save the last one, and print what the detector found."""
    os.makedirs(DEBUG_DIR, exist_ok=True)
    cameras, detector, image = open_camera(), LaneDetector(), None
    try:
        start = time.time()
        while time.time() - start < 3.0:
            frame = grab_frame(cameras)
            if frame is not None:
                image = frame
                detector.update(image)   # Let the tracker "warm up" on real frames.
            time.sleep(0.02)
    finally:
        close_device(cameras)
    if image is None:
        print("NO CAMERA IMAGES ARRIVED. Check the front camera and its cable.")
        return
    error, _, yellow_rows = detector.update(image)
    cv2.imwrite(f"{DEBUG_DIR}/cam_raw.jpg", image)
    cv2.imwrite(f"{DEBUG_DIR}/cam_lanes.jpg",
                detector.draw(image, f"error={error} yellow rows={yellow_rows}/5"))
    for row, yellow, edge, center in detector.rows:
        print(f"row y={row}: yellow={yellow}  road edge={edge}  "
              f"lane center={None if center is None else int(center)}")
    print(f"lane error: {error}  |  rows that found yellow: {yellow_rows}/5")
    print(f"Saved {DEBUG_DIR}/cam_raw.jpg and {DEBUG_DIR}/cam_lanes.jpg "
          "(orange = yellow line, blue = road edge, green = lane center)")


# =============================================================================
# MAIN DRIVING LOOP
# =============================================================================
def drive(use_lidar, dry_run, run_time=RUN_TIME, route=None, turntest=None):
    """Drive using the camera. Runs until Ctrl+C or RUN_TIME seconds.

    use_lidar - also stop for obstacles seen by the LiDAR.
    dry_run   - do everything EXCEPT turn the motor (throttle is forced to 0).
    run_time  - stop by itself after this many seconds.
    route     - list like ["S", "L", "RB2"]: one decision per stop line, repeated forever.
                None = no route (old behavior: just follow the line).
    turntest  - a single maneuver name (like "L"): do it right away, then end the run.
    """
    os.makedirs(DEBUG_DIR, exist_ok=True)

    # Ctrl+C sets this flag so the loop can end cleanly and stop the motor.
    quit_flag = threading.Event()
    # Ctrl+C (SIGINT) is the normal way to stop. We ALSO catch SIGTERM and SIGHUP.
    # SIGHUP is what Linux sends when your SSH connection closes (VS Code closed,
    # laptop asleep, Wi-Fi lost). Without this the program would be killed instantly
    # and would never send the "throttle = 0" command that stops the car.
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, lambda *_: quit_flag.set())

    detector = LaneDetector()
    cameras = open_camera()
    lidar = LidarWatcher() if use_lidar else None
    if lidar:
        lidar.start()
    qcar = QCar(readMode=1, frequency=RATE_HZ)

    # ---- Variables that change while driving ----
    state = "DRIVE"            # DRIVE, LOST (no lane), or STOP_LINE (waiting for green).
    steer = 0.0                # Steering angle we are sending to the wheels right now.
    steer_target = 0.0         # Steering angle the camera says we SHOULD have.
    error_filtered = None      # Smoothed lane error.
    error_previous = 0.0       # Lane error from the previous camera frame.
    speed_integral = 0.0       # Accumulated speed error (for the speed controller).
    blocked = False            # True while the LiDAR says something is in the way.
    last_lane_time = time.time()   # Last moment we saw the lane.
    last_image_time = time.time() + 1.0  # Last moment the camera sent ANY picture.
                                   # (+1 s of extra time at the start: the first picture can be slow.)
    camera_warned = False          # True after we printed the "camera stopped" warning.
    ignore_stop_until = 0.0
    use_stops = USE_STOP_LINES or (bool(route) and ROUTE_TRIGGER == "stop_line")
    yellow_lost_since = None   # When the camera stopped seeing ANY yellow (None = it sees some).
    route_index = 0            # Which decision of the route comes next.
    stop_start = 0.0           # When the car started waiting at a stop line.
    gyro_bias = 0.0            # Gyroscope reading when standing still (rad/s). Subtracted.
    man_steps = []             # Steps of the maneuver we are doing now.
    man_i = 0                  # Which step we are on.
    man_dist = 0.0             # Meters driven in this step.
    man_turn = 0.0             # Degrees turned in this step (always positive).
    man_start = 0.0            # When the maneuver began.
    man_name = ""
    turntest_end = None        # --turntest: when to stop after the maneuver.
    yellow_good_since = None   # When the camera started seeing the yellow line well (None = it does not).
    last_yellow_ok = 0.0       # Last moment the camera saw the yellow line well (3+ of 5 rows).
    man_total_dist = man_total_turn = 0.0
    frame_count = 0
    yellow_rows = 0
    t_prev = t_cam = t_start = time.time()
    t_print = 0.0
    t_save = 0.0               # Last time a debug picture was saved.
    pictures_saved = 0
    frames_since_print = 0

    log_file = open(f"run_{time.strftime('%H%M%S')}.csv", "w", newline="")
    log = csv.writer(log_file)
    log.writerow(["t", "state", "speed", "speed_target", "throttle", "steer",
                  "lane_error", "yellow_rows", "lidar_front_m"])

    if turntest:
        man_name, man_steps, man_i = turntest, MANEUVERS[turntest], 0
        man_dist = man_turn = man_total_dist = man_total_turn = 0.0
        man_start = time.time()
        state = "MANEUVER"
        print(f"TURN TEST: doing maneuver {turntest} now, then stopping.")
    elif route:
        print(f"Route: {' -> '.join(route)} (repeats). One decision per stop line.")

    print("DRY RUN: motor is OFF." if dry_run else "Driving. Press Ctrl+C to stop.")
    try:
        with qcar:
            while not quit_flag.is_set() and time.time() - t_start < run_time:
                now = time.time()
                dt = min(max(now - t_prev, 1e-3), 0.05)   # Time since last loop (seconds).
                t_prev = now

                # ---- Read the car's wheel speed (m/s) -------------------------
                qcar.read()
                speed = float(qcar.motorTach)
                # Gyroscope turn rate around the vertical axis (rad/s). If the car has no gyro, 0.
                try:
                    gyro = float(qcar.gyroscope[2])
                except Exception:
                    gyro = 0.0
                if abs(speed) < 0.02:   # Standing still: whatever the gyro says is its error (bias).
                    gyro_bias = 0.98 * gyro_bias + 0.02 * gyro

                # ---- CAMERA: how far are we from the lane center? -------------
                image = grab_frame(cameras)
                lane_error = None
                if image is not None:
                    last_image_time = now          # The camera is alive.
                    frame_count += 1
                    frames_since_print += 1
                    lane_error, stop_seen, yellow_rows = detector.update(image)
                    if yellow_rows >= 3:
                        last_yellow_ok = now
                    if yellow_rows == 0:
                        if yellow_lost_since is None:
                            yellow_lost_since = now
                    else:
                        yellow_lost_since = None
                    # Route trigger: the yellow line was there and has just ended -> intersection.
                    if (route and ROUTE_TRIGGER == "yellow_end" and state == "DRIVE"
                            and now > ignore_stop_until and yellow_lost_since is not None
                            and now - yellow_lost_since >= END_LOST_S
                            and now - last_yellow_ok <= END_SEEN_S):
                        man_name = route[route_index % len(route)]
                        route_index += 1
                        man_steps, man_i = MANEUVERS[man_name], 0
                        man_dist = man_turn = man_total_dist = man_total_turn = 0.0
                        man_start = now
                        state = "MANEUVER"
                        print(f"Yellow line ended - maneuver {man_name}")
                    if yellow_rows >= HANDOVER_ROWS:
                        if yellow_good_since is None:
                            yellow_good_since = now
                    else:
                        yellow_good_since = None
                    cam_dt = max(now - t_cam, 0.02)   # Time between camera frames.
                    t_cam = now

                    if lane_error is not None:
                        last_lane_time = now
                        # Smooth the raw error so one bad frame does not jerk the wheels.
                        if error_filtered is None:
                            error_filtered = lane_error
                        else:
                            error_filtered = (STEER_ALPHA * lane_error
                                              + (1 - STEER_ALPHA) * error_filtered)
                        # How fast is the error changing? (limited to avoid huge spikes)
                        error_rate = float(np.clip((error_filtered - error_previous) / cam_dt,
                                                   -2.0, 2.0))
                        # PD controller: steer against the error.
                        steer_target = float(np.clip(
                            -STEER_SIGN * (KP * error_filtered + KD * error_rate),
                            -MAX_STEER, MAX_STEER))
                        error_previous = error_filtered

                    if use_stops and state == "DRIVE" and stop_seen and now > ignore_stop_until:
                        state = "STOP_LINE"
                        stop_start = now
                        print("Stop line seen - waiting for green")

                    # Save a debug picture every SAVE_EVERY frames (they loop over 60 files).
                    if SAVE_EVERY and now - t_save >= SAVE_EVERY:
                        t_save = now
                        cv2.imwrite(f"{DEBUG_DIR}/drive_{pictures_saved % 80:02d}.jpg",
                                    detector.draw(image, f"{now - t_start:.1f}s {state} steer={steer:+.2f}"))
                        pictures_saved += 1

                # ---- MANEUVER: measure progress and choose the steering -------
                if state == "MANEUVER":
                    kind = man_steps[man_i][0]
                    man_dist += max(speed, 0.0) * dt
                    man_total_dist += max(speed, 0.0) * dt
                    turned = abs(gyro - gyro_bias) * dt * 180.0 / np.pi   # degrees this loop
                    man_turn += turned
                    man_total_turn += turned
                    if kind == "straight":
                        # If the camera still sees the yellow line, keep following it (it keeps the
                        # car straight). If not (inside the intersection), hold the wheels at 0.
                        if (now - last_yellow_ok) > 0.3:
                            steer_target = 0.0
                        step_done = man_dist >= man_steps[man_i][1]
                    else:
                        steer_target = float(np.clip(man_steps[man_i][1], -MAX_STEER, MAX_STEER))
                        step_done = man_turn >= man_steps[man_i][2]
                        # Camera takeover: the yellow line is back, so stop turning blindly.
                        if (not step_done and man_turn >= ARC_MIN_FRACTION * man_steps[man_i][2]
                                and yellow_good_since is not None
                                and now - yellow_good_since >= HANDOVER_S):
                            print(f"Camera sees the line again after {man_total_turn:.0f} deg - taking over")
                            man_i = len(man_steps) - 1   # Skip to the last step...
                            step_done = True             # ...and finish it now.
                    if step_done:
                        man_i += 1
                        man_dist = man_turn = 0.0
                    if man_i >= len(man_steps) or (now - man_start) > MANEUVER_MAX_S:
                        timed_out = man_i < len(man_steps)
                        print(f"Maneuver {man_name} {'TIMED OUT' if timed_out else 'done'}: "
                              f"drove {man_total_dist:.2f} m, turned {man_total_turn:.0f} deg")
                        if turntest and turntest_end is None:
                            turntest_end = now + TURNTEST_FOLLOW_S   # Calibration: watch the camera for a while.
                        # Back to the camera: forget the old line positions and search again.
                        detector.yellow_x = [None] * len(SCAN_ROWS)
                        error_filtered = None
                        last_lane_time = now
                        ignore_stop_until = now + max(STOP_IGNORE_S, END_COOLDOWN_S if route else 0.0)
                        state = "DRIVE"

                if turntest_end is not None and now > turntest_end:
                    print("Turn test finished.")
                    break

                # ---- STEERING: move toward the target, but not too fast -------
                max_change = STEER_RATE * dt
                steer += float(np.clip(steer_target - steer, -max_change, max_change))

                # ---- CAMERA WATCHDOG: is the camera still sending pictures? ---
                # If the camera dies, the car must NOT keep driving blind.
                camera_dead = (now - last_image_time) > CAMERA_TIMEOUT
                if camera_dead and not camera_warned:
                    print("WARNING: the camera stopped sending pictures. Stopping the car.")
                camera_warned = camera_dead      # Warn again if it happens a second time.
                if camera_dead and (now - last_image_time) > CAMERA_GIVE_UP:
                    print("The camera did not come back. Ending the run.")
                    break                        # The car is already stopped (throttle 0).

                # ---- STATE MACHINE: decide the target speed -------------------
                if state == "MANEUVER":
                    speed_target = 0.0 if camera_dead else V_MANEUVER   # Never drive blind.
                elif state == "STOP_LINE":
                    speed_target = 0.0
                    if light_is_green() and (now - stop_start) >= MIN_STOP_S:
                        print("Green - go")
                        if route:
                            man_name = route[route_index % len(route)]
                            route_index += 1
                            man_steps, man_i = MANEUVERS[man_name], 0
                            man_dist = man_turn = man_total_dist = man_total_turn = 0.0
                            man_start = now
                            state = "MANEUVER"
                            print(f"Maneuver {man_name}")
                        else:
                            state = "DRIVE"
                            ignore_stop_until = now + STOP_IGNORE_S
                else:
                    lane_lost = (now - last_lane_time) > LOST_TIME
                    state = "LOST" if lane_lost else "DRIVE"
                    if camera_dead:
                        state = "CAMERA_STOPPED"
                        speed_target = 0.0          # No pictures at all -> stop at once.
                    elif lane_lost:
                        speed_target = 0.0          # Cannot see the lane -> stop.
                    elif abs(steer) > 0.25:
                        speed_target = V_CURVE      # Turning hard -> slow down.
                    else:
                        speed_target = V_MAX        # Straight road -> normal speed.

                # ---- LIDAR has the last word: obstacle ahead -> stop ----------
                front = lidar.front if lidar else np.inf
                label = state            # Text shown on screen (may get "+OBSTACLE" added).
                if lidar:
                    # "Hysteresis": stop at D_STOP but only restart past D_GO.
                    # This prevents rapid stop/go flickering.
                    if front < D_STOP:
                        blocked = True
                    elif front > D_GO:
                        blocked = False
                    if blocked:
                        speed_target = 0.0
                        label += "+OBSTACLE"

                # ---- THROTTLE: only 0 to MAX_THROTTLE, NEVER negative ---------
                if speed_target <= 0.0:
                    throttle = 0.0          # Want to stop: just release the motor.
                    speed_integral = 0.0
                else:
                    speed_error = speed_target - speed
                    speed_integral = float(np.clip(speed_integral + speed_error * dt, -0.05, 0.05))
                    throttle = float(np.clip(
                        THROTTLE_BASE + KP_V * speed_error + KI_V * speed_integral,
                        0.0, MAX_THROTTLE))
                if dry_run:
                    throttle = 0.0          # Dry run: motor always off.

                # ---- SEND the commands to the car -----------------------------
                braking = speed_target < speed - 0.05   # Turn on brake lights when slowing.
                qcar.write(throttle=throttle, steering=steer + STEER_TRIM,
                           LEDs=np.array([0, 0, 0, 0, int(braking), int(braking), 1, 1]))

                # ---- LOG to the CSV file --------------------------------------
                if image is not None:
                    log.writerow([f"{now - t_start:.2f}", label, f"{speed:.2f}",
                                  f"{speed_target:.2f}", f"{throttle:.3f}", f"{steer:+.3f}",
                                  "" if lane_error is None else f"{lane_error:+.3f}",
                                  yellow_rows,
                                  "" if not np.isfinite(front) else f"{front:.2f}"])

                # ---- PRINT a status line a few times per second ---------------
                if now - t_print > 1.0 / PRINT_HZ:
                    fps = frames_since_print / max(now - t_print, 1e-3) if t_print else 0
                    frames_since_print, t_print = 0, now
                    err_text = "  --  " if lane_error is None else f"{lane_error:+.3f}"
                    lidar_text = "--" if not np.isfinite(front) else f"{front:.2f}"
                    print(f"{now - t_start:6.1f}s {label:<16} speed={speed:+.2f} "
                          f"target={speed_target:.2f} throttle={throttle:.3f} "
                          f"steer={steer:+.2f} lane_err={err_text} "
                          f"yellow={yellow_rows}/5 cam={fps:4.1f}fps lidar={lidar_text}"
                          + (f" turned={man_total_turn:3.0f}deg dist={man_total_dist:.2f}m"
                             if state == "MANEUVER" else ""))

            qcar.write(throttle=0, steering=0)   # Always leave the car stopped.
    finally:
        close_device(cameras)
        if lidar:
            lidar.close()
        log_file.close()
    print(f"Stopped. Debug pictures: {DEBUG_DIR}/drive_XX.jpg   Log: run_*.csv")


# =============================================================================
# ENTRY POINT: choose what to run from the command line
# =============================================================================
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="QCar 2 camera lane following")
    parser.add_argument("--camtest", action="store_true",
                        help="save a camera picture and print what is detected (no driving)")
    parser.add_argument("--lidartest", action="store_true",
                        help="print LiDAR readings to find which angle is 'front' (no driving)")
    parser.add_argument("--lidar", action="store_true",
                        help="drive and stop when the LiDAR sees something ahead")
    parser.add_argument("--dry", action="store_true",
                        help="run everything but keep the motor OFF")
    parser.add_argument("--route", default=None,
                        help="comma list of decisions, one per stop line, repeated: S,L,R,RB1,RB2,RB3")
    parser.add_argument("--turntest", default=None,
                        help="do ONE maneuver (S, L, R, RB1, RB2 or RB3) right now, then stop (calibration)")
    parser.add_argument("--time", type=int, default=RUN_TIME,
                        help=f"stop by itself after this many seconds (default {RUN_TIME})")
    args = parser.parse_args()

    print(f"lane_follow.py version {VERSION}")   # Which file is really running on the car?

    if args.lidartest:
        lidar_test()
    elif args.camtest:
        camera_test()
    else:
        route = [x.strip().upper() for x in args.route.split(",")] if args.route else None
        for name in (route or []) + ([args.turntest.upper()] if args.turntest else []):
            if name not in MANEUVERS:
                raise SystemExit(f"Unknown maneuver '{name}'. Use: {', '.join(MANEUVERS)}")
        drive(args.lidar, args.dry, args.time, route,
              args.turntest.upper() if args.turntest else None)
