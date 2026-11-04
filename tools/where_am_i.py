"""
where_am_i.py - Ask the car WHERE it is on the road map (x, y, heading) and which map node is closest.

The car does NOT move and the motor is NOT used. Run it on the CAR:
    export PYTHONPATH=~/Documents/Quanser/libraries/python
    python3 where_am_i.py              # prints the position 5 times per second for 20 seconds
    python3 where_am_i.py --calibrate  # re-records the reference LiDAR scan (see below)

HOW THE QCar KNOWS ITS POSITION
-------------------------------
The LiDAR looks at the walls and furniture of the ROOM (not at the painted road). On the calibration step the
car sits at a KNOWN place and saves what the LiDAR sees. Later, by comparing a new LiDAR scan with the saved
one, the car works out its own x, y and heading on the map.

WHEN TO USE --calibrate
-----------------------
Only if the positions look wrong, or if furniture was moved. Put the car EXACTLY at CALIBRATION_POSE
(x = 0.0 m, y = 2.0 m, heading -90 deg = our notes: right lane of the road arm that leads from the
traffic-light intersection to the roundabout, facing the intersection), keep it still, then run --calibrate.

HOW TO TEST
-----------
1. Put the car on a map node you know (see map_nodes.txt from dump_map.py), facing the way traffic goes.
2. Run this script. Compare "x, y, heading" with that node's numbers. An error under about 0.10 m and 10 deg is good.
3. Push the car by hand to another node and check again.
"""
import argparse
import time

import numpy as np

from pal.products.qcar import QCarGPS
from hal.products.mats import SDCSRoadMap

CALIBRATION_POSE = [0.0, 2.0, -np.pi / 2]   # x (m), y (m), heading (rad)


def load_nodes(roadmap):
    """Return {node_id: (x, y, heading)} for every node the map has."""
    nodes = {}
    for node_id in range(0, 80):
        try:
            pose = np.asarray(roadmap.get_node_pose(node_id), dtype=float).squeeze()
            nodes[node_id] = (float(pose[0]), float(pose[1]), float(pose[2]))
        except Exception:
            pass                      # This id does not exist: skip it.
    return nodes


def calibrate():
    print(f"Calibrating. The car must be at x={CALIBRATION_POSE[0]}, y={CALIBRATION_POSE[1]}, "
          f"heading={np.degrees(CALIBRATION_POSE[2]):.0f} deg and NOT move.")
    with QCarGPS(initialPose=CALIBRATION_POSE, calibrate=True):
        pass
    print("Done. Now run without --calibrate to check the position.")


def watch(seconds=20):
    nodes = load_nodes(SDCSRoadMap(leftHandTraffic=False))
    print(f"{len(nodes)} map nodes loaded.")
    gps = QCarGPS(initialPose=CALIBRATION_POSE, calibrate=False)
    t_end = time.time() + seconds
    with gps:
        while time.time() < t_end:
            if gps.readGPS():
                x, y = float(gps.position[0]), float(gps.position[1])
                heading = float(gps.orientation[2])
                # Closest node = the one with the smallest distance to (x, y).
                best = min(nodes, key=lambda n: (nodes[n][0] - x) ** 2 + (nodes[n][1] - y) ** 2)
                bx, by, bh = nodes[best]
                dist = float(np.hypot(bx - x, by - y))
                dh = np.degrees((heading - bh + np.pi) % (2 * np.pi) - np.pi)
                print(f"x={x:+.2f} y={y:+.2f} heading={np.degrees(heading):+6.1f} deg   "
                      f"closest node {best}: {dist:.2f} m away, heading differs {dh:+.0f} deg")
            else:
                print("no LiDAR position this time (waiting)...")
            time.sleep(0.2)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Show the car position on the road map (no driving)")
    parser.add_argument("--calibrate", action="store_true", help="record the reference scan (car at the calibration pose)")
    parser.add_argument("--time", type=int, default=20, help="how many seconds to watch")
    args = parser.parse_args()
    calibrate() if args.calibrate else watch(args.time)
