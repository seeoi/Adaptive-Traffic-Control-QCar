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
VERSION = "1.0.0"

# =============================================================================
# SETTINGS  (the numbers you will most likely want to change are at the top)
# =============================================================================

# ---- Speed -------------------------------------------------------------------
V_MAX = 0.30           # Target speed on straight road, in meters per second.
V_CURVE = 0.20         # Target speed while turning hard (slower is safer).
THROTTLE_BASE = 0.07   # Fixed motor push (0 to 1) whenever the car should move.
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
LOST_TIME = 1.0     # Seconds without seeing the lane before the car stops.

# ---- Stop lines / traffic light (OFF for now) --------------------------------
USE_STOP_LINES = False  # Set True later, when the traffic lights are working.
STOP_FILL = 0.65        # How much of the lane must look white to count as a stop line.
STOP_ROWS = (0.86, 0.96)  # Which image rows are checked for a stop line.
STOP_IGNORE_S = 3.0     # After leaving a stop line, ignore stop lines for this many seconds.

# ---- LiDAR (only used with --lidar) ------------------------------------------
LIDAR_FRONT = 0.0   # LiDAR angle (radians) that points straight ahead. Find it with --lidartest.
LIDAR_DIR = 1       # +1 if LiDAR angles grow toward the LEFT, -1 if toward the RIGHT.
CONE_DEG = 15       # Only look at objects within this many degrees of straight ahead.
D_STOP = 0.30       # Stop if something is closer than this (meters).
D_GO = 0.40         # Start again only when the road is clear farther than this (meters).
MIN_RANGE = 0.15    # Ignore LiDAR points closer than this (that is the car body).

# ---- Other -------------------------------------------------------------------
RUN_TIME = 300      # The program stops by itself after this many seconds.
RATE_HZ = 200       # How often the QCar interface runs.
PRINT_HZ = 5        # How many status lines per second are printed on screen.
SAVE_EVERY = 10     # Save one debug picture every N camera frames.
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
def drive(use_lidar, dry_run):
    """Drive using the camera. Runs until Ctrl+C or RUN_TIME seconds.

    use_lidar - also stop for obstacles seen by the LiDAR.
    dry_run   - do everything EXCEPT turn the motor (throttle is forced to 0).
    """
    os.makedirs(DEBUG_DIR, exist_ok=True)

    # Ctrl+C sets this flag so the loop can end cleanly and stop the motor.
    quit_flag = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: quit_flag.set())

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
    ignore_stop_until = 0.0
    frame_count = 0
    yellow_rows = 0
    t_prev = t_cam = t_start = time.time()
    t_print = 0.0
    frames_since_print = 0

    log_file = open(f"run_{time.strftime('%H%M%S')}.csv", "w", newline="")
    log = csv.writer(log_file)
    log.writerow(["t", "state", "speed", "speed_target", "throttle", "steer",
                  "lane_error", "yellow_rows", "lidar_front_m"])

    print("DRY RUN: motor is OFF." if dry_run else "Driving. Press Ctrl+C to stop.")
    try:
        with qcar:
            while not quit_flag.is_set() and time.time() - t_start < RUN_TIME:
                now = time.time()
                dt = min(max(now - t_prev, 1e-3), 0.05)   # Time since last loop (seconds).
                t_prev = now

                # ---- Read the car's wheel speed (m/s) -------------------------
                qcar.read()
                speed = float(qcar.motorTach)

                # ---- CAMERA: how far are we from the lane center? -------------
                image = grab_frame(cameras)
                lane_error = None
                if image is not None:
                    frame_count += 1
                    frames_since_print += 1
                    lane_error, stop_seen, yellow_rows = detector.update(image)
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

                    if USE_STOP_LINES and state == "DRIVE" and stop_seen and now > ignore_stop_until:
                        state = "STOP_LINE"
                        print("Stop line seen - waiting for green")

                    # Save a debug picture every SAVE_EVERY frames (they loop over 60 files).
                    if SAVE_EVERY and frame_count % SAVE_EVERY == 0:
                        cv2.imwrite(f"{DEBUG_DIR}/drive_{(frame_count // SAVE_EVERY) % 60:02d}.jpg",
                                    detector.draw(image, f"{state} steer={steer:+.2f}"))

                # ---- STEERING: move toward the target, but not too fast -------
                max_change = STEER_RATE * dt
                steer += float(np.clip(steer_target - steer, -max_change, max_change))

                # ---- STATE MACHINE: decide the target speed -------------------
                if state == "STOP_LINE":
                    if light_is_green():
                        state = "DRIVE"
                        ignore_stop_until = now + STOP_IGNORE_S
                        print("Green - go")
                    speed_target = 0.0
                else:
                    lane_lost = (now - last_lane_time) > LOST_TIME
                    state = "LOST" if lane_lost else "DRIVE"
                    if lane_lost:
                        speed_target = 0.0          # Cannot see the lane -> stop.
                    elif abs(steer) > 0.25:
                        speed_target = V_CURVE      # Turning hard -> slow down.
                    else:
                        speed_target = V_MAX        # Straight road -> normal speed.

                # ---- LIDAR has the last word: obstacle ahead -> stop ----------
                front = lidar.front if lidar else np.inf
                if lidar:
                    # "Hysteresis": stop at D_STOP but only restart past D_GO.
                    # This prevents rapid stop/go flickering.
                    if front < D_STOP:
                        blocked = True
                    elif front > D_GO:
                        blocked = False
                    if blocked:
                        speed_target = 0.0
                        state += "+OBSTACLE"

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
                    log.writerow([f"{now - t_start:.2f}", state, f"{speed:.2f}",
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
                    print(f"{now - t_start:6.1f}s {state:<16} speed={speed:+.2f} "
                          f"target={speed_target:.2f} throttle={throttle:.3f} "
                          f"steer={steer:+.2f} lane_err={err_text} "
                          f"yellow={yellow_rows}/5 cam={fps:4.1f}fps lidar={lidar_text}")

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
    args = parser.parse_args()

    print(f"lane_follow.py version {VERSION}")   # Which file is really running on the car?

    if args.lidartest:
        lidar_test()
    elif args.camtest:
        camera_test()
    else:
        drive(args.lidar, args.dry)
