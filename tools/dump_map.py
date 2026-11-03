"""
dump_map.py - Print the nodes of the SDCS road map (id, x, y, heading).

WHY: the map knows where every intersection and roundabout is. With this list we can
turn a route like "go straight, turn left, take the 2nd roundabout exit" into distances
and turning angles that the car can follow with the camera, wheel speed and gyro.

The car does NOT move and the motor is NOT used. Run it on the CAR:
    PYTHONPATH=~/Documents/Quanser/libraries/python python3 dump_map.py > map_nodes.txt
Then copy the result to the laptop (run this on the LAPTOP):
    scp nvidia@192.168.2.183:~/map_nodes.txt .
"""
import numpy as np
from hal.products.mats import SDCSRoadMap

roadmap = SDCSRoadMap(leftHandTraffic=False)

# ---- 1) Every node: where it is and where it points ---------------------------------
print("node_id, x_m, y_m, heading_rad")
found = 0
for node_id in range(0, 60):
    try:
        pose = np.asarray(roadmap.get_node_pose(node_id), dtype=float).squeeze()
        print(f"{node_id}, {pose[0]:.3f}, {pose[1]:.3f}, {pose[2]:.3f}")
        found += 1
    except Exception:
        pass                      # This id does not exist on this map: skip it.
print(f"# {found} nodes found")

# ---- 2) Edges (which node connects to which), if the library exposes them -----------
for name in ("nodes", "edges"):
    items = getattr(roadmap, name, None)
    if items is None:
        print(f"# roadmap.{name}: not available")
        continue
    try:
        print(f"# roadmap.{name}: {len(items)} items")
        for item in list(items)[:80]:
            print(f"#   {repr(item)[:160]}")
    except Exception as error:
        print(f"# roadmap.{name}: could not be listed ({error})")

# ---- 3) Example: how a list of node IDs becomes a path of waypoints -----------------
try:
    route = [10, 4, 20, 10]       # Example route from Quanser.
    waypoints = roadmap.generate_path(route)
    print(f"# route {route} -> {np.asarray(waypoints).shape[1]} waypoints")
except Exception as error:
    print(f"# generate_path failed: {error}")
