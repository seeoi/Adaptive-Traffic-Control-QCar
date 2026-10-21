"""
OLD SCRIPT (kept for reference only - do NOT use for driving).
Map + LiDAR route following (Stanley steering + PI speed). Needs a good LiDAR calibration.
Problem found: ignores the painted lines, and the speed controller can output a NEGATIVE throttle (reverse).
Originally named: TestGimmik.py
"""
"""
QCar 2: follow an SDCS roadmap path and stop at a red light.

v2 - fixes for poor path tracking:
  * Pose comes from Quanser's EKF (GPS + gyro + wheel speed, 200 Hz) instead of
    raw LiDAR GPS (~10-15 Hz, jumpy). Raw GPS alone makes the car weave.
  * Stanley steering is referenced to the FRONT axle (0.2 m ahead of center),
    like Quanser's vehicle_control.py. Using the center makes it cut corners.
  * Calibration pose and start pose are separate (matches Quanser's example).
  * Start heading comes from the roadmap node instead of a hard-coded 0.
  * Every loop is logged to CSV + a tracking-error summary prints at the end,
    so you can tell a code problem from a car/localization problem.
  * --straight test mode checks the car's steering trim.

Usage (on the QCar):
  sudo PYTHONPATH=$PYTHONPATH python3 qcar_intersection.py --calibrate   # car at CALIBRATION_POSE
  sudo PYTHONPATH=$PYTHONPATH python3 qcar_intersection.py --car 1       # car at first node
  sudo PYTHONPATH=$PYTHONPATH python3 qcar_intersection.py --straight    # wheels-on-floor trim test
"""
import argparse, csv, signal, threading, time
import numpy as np

from pal.products.qcar import QCar, QCarGPS
from hal.products.mats import SDCSRoadMap
try:
    from hal.products.qcar import QCarEKF
except ImportError:
    QCarEKF = None
    print("WARNING: QCarEKF not found - using raw GPS, expect weaving.")

# ----------------------------------------------------------------- CONFIG ---
CARS = {
    1: dict(nodes=[10, 4, 20, 10], stop_line=(0.15, 1.10), approach="NS"),
    2: dict(nodes=[2, 4, 14, 20, 22, 10], stop_line=(-0.90, 0.25), approach="EW"),
}
CALIBRATION_POSE = [0.0, 2.0, -np.pi / 2]  # Quanser default: x, y (m), heading (rad)
V_CRUISE    = 0.5     # m/s
K_STANLEY   = 1.0     # lower (0.5) if it weaves, raise (1.5) if it drifts wide
AXLE_OFFSET = 0.2     # m, center -> front axle
STEER_TRIM  = 0.0     # rad, set from the --straight test if the car pulls
STOP_ZONE   = 0.60    # m
STOP_DIST   = 0.10    # m
RUN_TIME    = 120     # s
RATE_HZ     = 200
MAX_STEER   = np.pi / 6


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi

# ------------------------------------------------------------ CONTROLLERS ---
class SpeedController:
    """PI on wheel speed -> throttle."""
    def __init__(self, kp=0.1, ki=1.0, u_max=0.3):
        self.kp, self.ki, self.u_max, self.ei = kp, ki, u_max, 0.0

    def update(self, v, v_ref, dt):
        e = v_ref - v
        self.ei = np.clip(self.ei + e * dt, -self.u_max, self.u_max)
        return float(np.clip(self.kp * e + self.ki * self.ei, -self.u_max, self.u_max))


class StanleyController:
    """Stanley lateral control; projects the front axle onto the current path segment."""
    def __init__(self, wp, k=K_STANLEY):
        self.wp, self.N, self.k, self.i, self.ect = wp, wp.shape[1], k, 0, 0.0

    def update(self, p, th, v):
        # advance past segments the front axle has already passed
        while self.i < self.N - 2:
            a, b = self.wp[:, self.i], self.wp[:, self.i + 1]
            L = np.linalg.norm(b - a)
            if L < 1e-6 or np.dot(p - a, b - a) / L >= L:
                self.i += 1
            else:
                break
        a, b = self.wp[:, self.i], self.wp[:, self.i + 1]
        u = (b - a) / max(np.linalg.norm(b - a), 1e-6)
        tangent = np.arctan2(u[1], u[0])
        self.ect = u[0] * (p[1] - a[1]) - u[1] * (p[0] - a[0])  # + = left of path
        delta = wrap(wrap(tangent - th) + np.arctan2(-self.k * self.ect, max(v, 0.1)))
        return float(np.clip(delta, -MAX_STEER, MAX_STEER))

    @property
    def done(self):
        return self.i >= self.N - 2

# ---------------------------------------------------------- TRAFFIC LIGHT ---
class FixedTimeLight:
    """Software light. Replace with your adaptive algorithm / V2I client."""
    def __init__(self, g_ns=8.0, g_ew=8.0, all_red=2.0):
        self.t0, self.g_ns, self.g_ew, self.ar = time.time(), g_ns, g_ew, all_red

    def is_green(self, approach):
        t = (time.time() - self.t0) % (self.g_ns + self.g_ew + 2 * self.ar)
        if approach == "NS":
            return t < self.g_ns
        return self.g_ns + self.ar <= t < self.g_ns + self.ar + self.g_ew

# ------------------------------------------------------------------ TESTS ---
def calibrate_only():
    print(f"Calibrating with car at {CALIBRATION_POSE}. Keep it still...")
    with QCarGPS(initialPose=CALIBRATION_POSE, calibrate=True):
        pass
    print("Done. Move the car to its first route node and run with --car N.")


def straight_test():
    """Zero steering, slow roll for 3 s. Car should go straight on the floor."""
    qcar = QCar(readMode=1, frequency=RATE_HZ)
    with qcar:
        t0 = time.time()
        while time.time() - t0 < 3.0:
            qcar.read()
            qcar.write(throttle=0.08, steering=STEER_TRIM)
        qcar.write(throttle=0, steering=0)
    print("If it curved, adjust STEER_TRIM by ~0.02 rad (+ = left) and repeat.")

# ------------------------------------------------------------------- MAIN ---
def run(car_id):
    cfg = CARS[car_id]
    roadmap = SDCSRoadMap(leftHandTraffic=False)
    wp = roadmap.generate_path(cfg["nodes"])
    x0 = np.asarray(roadmap.get_node_pose(cfg["nodes"][0]), dtype=float).squeeze()
    stop_line = np.array(cfg["stop_line"])

    light, speed_ctl, steer_ctl = FixedTimeLight(), SpeedController(), StanleyController(wp)
    stop_flag = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop_flag.set())

    qcar = QCar(readMode=1, frequency=RATE_HZ)
    gps = QCarGPS(initialPose=CALIBRATION_POSE, calibrate=False)
    ekf = QCarEKF(x_0=x0) if QCarEKF else None

    log_path = f"run_car{car_id}_{time.strftime('%H%M%S')}.csv"
    x, y, th = x0
    delta, ects = 0.0, []

    with qcar, gps, open(log_path, "w", newline="") as f:
        log = csv.writer(f)
        log.writerow(["t", "x", "y", "th", "gps_x", "gps_y", "gps_th",
                      "v", "v_ref", "delta", "ect", "wp_idx"])
        t_start = t_prev = time.time()
        crossed = False
        while not stop_flag.is_set() and time.time() - t_start < RUN_TIME:
            t = time.time(); dt = max(t - t_prev, 1e-3); t_prev = t

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

            heading = np.array([np.cos(th), np.sin(th)])
            p_front = np.array([x, y]) + AXLE_OFFSET * heading

            # --- intersection logic (front bumper vs. stop line) ------------
            d = float(np.linalg.norm(stop_line - p_front))
            ahead = np.dot(stop_line - p_front, heading) > 0
            if not ahead and d < 0.3:
                crossed = True
            v_ref = V_CRUISE
            if ahead and not crossed and d < STOP_ZONE and not light.is_green(cfg["approach"]):
                v_ref = 0.0 if d < STOP_DIST else V_CRUISE * (d - STOP_DIST) / (STOP_ZONE - STOP_DIST)
            if steer_ctl.done:
                v_ref = 0.0

            u = speed_ctl.update(v, v_ref, dt)
            delta = steer_ctl.update(p_front, th, v)
            braking = v_ref < v - 0.05
            leds = np.array([0, 0, 0, 0, int(braking), int(braking), 1, 1])
            qcar.write(throttle=u, steering=delta + STEER_TRIM, LEDs=leds)

            ects.append(abs(steer_ctl.ect))
            g = y_gps if y_gps is not None else [np.nan] * 3
            log.writerow([f"{t - t_start:.3f}", f"{x:.3f}", f"{y:.3f}", f"{th:.3f}",
                          *(f"{q:.3f}" for q in g), f"{v:.3f}", f"{v_ref:.2f}",
                          f"{delta:.3f}", f"{steer_ctl.ect:.3f}", steer_ctl.i])

            if steer_ctl.done and v < 0.02:
                break
        qcar.write(throttle=0, steering=0)

    if ects:
        e = np.array(ects)
        print(f"\nLogged to {log_path}")
        print(f"Tracking error (what the car THINKS): mean {e.mean()*100:.1f} cm, "
              f"max {e.max()*100:.1f} cm")
        print("Under ~5 cm mean but the car visibly left the road -> localization/map/car issue.")
        print("Over ~10 cm mean -> controller tuning issue; send me the CSV.")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--car", type=int, default=1, choices=CARS.keys())
    ap.add_argument("--calibrate", action="store_true")
    ap.add_argument("--straight", action="store_true")
    a = ap.parse_args()
    if a.calibrate:
        calibrate_only()
    elif a.straight:
        straight_test()
    else:
        run(a.car)