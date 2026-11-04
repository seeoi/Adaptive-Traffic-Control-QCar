# Handoff: where the project is and what to build next (2026-11-04)

Read this first. Then read README.md, CHANGELOG.md and src/lane_follow.py (v1.2.2).

## Goal of the project
Senior Design: "Improving Adaptive Algorithms of Traffic Control with Pseudo Smart City".
Several Quanser QCar 2 cars drive at the same time on the Quanser Self-Driving Car Studio mat (TAMIU lab).
Traffic lights run on a Raspberry Pi (previous team's Python/Flask/REST/V2I code, NOT in this repo yet).
Final experiment: find out which traffic-light system makes traffic flow best. Cars with LiDAR and cars without
LiDAR must both work (hybrid fleet).

## What the student wants from the next step (his words, summarized)
A car that drives NATURALLY (smooth, no stop-and-go) and ALWAYS KNOWS where to go. At every intersection it
chooses LEFT, STRAIGHT or RIGHT from a route, stays in the correct lane, and exits in the correct lane of the
road it chose. The same for every curve and for the roundabout (choose exit 1, 2 or 3, then loop forever).
Dashed-line zones and the roundabout have NO yellow center line.

## Facts about the mat (Quanser SDCS mat picture, student's annotations)
- There are NO white stop lines. At intersections there are only small corner marks (like brackets) on the
  vertical road near the middle: two pairs, one north and one south of the central 4-way crossing.
  Cars wait at those marks (the student marked them with blue X).
- The yellow center line is NOT continuous: it is cut at crossings and does not exist inside the roundabout.
  Roundabout entry/exit are marked by white dashed arcs and a dashed circle (top right of the picture).
- White dashed lines on the left lane and bottom-left where the road splits into two lanes.
- Right-hand traffic. `SDCSRoadMap(leftHandTraffic=False)` from `hal.products.mats` holds the node poses and
  `generate_path(list_of_nodes)`. Example route from Quanser: [10, 4, 20, 10].
- Node poses (car 7 = qcar-67815): see CHANGELOG 2.0.0. Map picture scale: about 166 px per m,
  px_x = 371 + 166 x, px_y = 45 + 166 (4.497 - y) on Quanser's 787x1000 mat picture.

## Hardware and software facts (measured on car 7 at 192.168.2.183, user/password nvidia)
- `pal.products.qcar`: QCar (motorTach m/s, gyroscope[2] rad/s, write(throttle, steering)), QCarCameras, QCarLidar,
  QCarGPS (LiDAR vs saved ROOM scan), `hal.products.qcar.QCarEKF`.
- Negative throttle = reverse. Throttle is kept in 0..0.15. Steering limit 0.5 rad.
- Camera 820x410, pink tint, so yellow is found relative to the road color (LAB). The camera sometimes stops
  (cam=0.0fps); v1.0.2 watchdog stops the car after 0.5 s. Low battery makes it worse.
- Measured turning: steering -0.40 at ~0.2 m/s gave about 11 deg/s and radius about 1.1 m (tach distance).
  Full lock (-0.50) is not much tighter. The tach distance may be mis-scaled: NOT verified with a tape measure
  (we asked the student to measure 1 m of `--turntest S`; no answer yet).
- Gyro works (a straight 1 m run read 6 deg).
- The PI speed loop cannot brake (throttle >= 0). `THROTTLE_BASE` 0.08 pushes the car to ~0.27 m/s alone
  and to 0.4 m/s in some places. v1.2.1 smooths the speed target and limits acceleration (untested on the car).
- LiDAR-based room localization (`old/map_lidar_follow.py`) was 2 m / 180 deg off; calibration pose
  [0, 2, -90 deg]. The lab changes (furniture moves, people, other cars), so it is NOT trusted as the base.
- `tools/where_am_i.py` (v1.2.2) prints the LiDAR/map position and the closest node. Not run yet.

## What was tried for turning and why it is not enough
v1.1.x: route list + "maneuvers" (drive a distance, turn with the gyro until N degrees, hand over to the camera
when the yellow line is seen again). Trigger = "yellow line ended" (`ROUTE_TRIGGER`), `--arm-m` delays the route
until an odometer distance. Results on the real car:
- It works as a mechanism, but each corner needs hand calibration, the turn came out too wide, the car entered
  at 0.37 m/s, the camera did not see the line again, and the car kept turning right.
- The trigger also fires on curves that hide the line. Starting a test where there is no yellow never triggers.
- Not tested: `L`, `S` through a real intersection, `RB1..RB3`, the traffic-light hook, LiDAR obstacle stop.

## Design recommended for the next version (WRITTEN as src/map_drive.py v2.0.0, not yet tested on the car)
1. Pose estimate that does not depend on the room: wheel speed + gyro (QCarEKF or own) starting from a known
   map node and heading. Camera landmarks correct it (yellow line ended / corner marks / roundabout
   entry) so error does not grow over a lap.
2. Route = list of map nodes. Path from `SDCSRoadMap.generate_path`. Follow it continuously (Stanley or pure
   pursuit) with a smooth speed profile that depends on curvature; no stop-and-go. Camera lane error is used as a
   small correction where the yellow line exists.
3. LiDAR is only for seeing cars/obstacles (front now; sides/back later), so cars without LiDAR still drive.
4. A hook for the traffic light (`light_is_green()` exists, fake 8 s/8 s) and for V2I later.
5. Keep the rules: code and comments in English with very easy comments, VERSION + CHANGELOG bump on every
   change, GitHub `main` is the only source of truth, test steps for the student must be simple and work
   offline (his laptop is on the car Wi-Fi with no internet).

## How testing works (important for planning)
One person, one laptop (Windows PowerShell), the car on Wi-Fi `Quanser_UVS_209-5G`, one test takes about 3 minutes.
`powershell -ExecutionPolicy Bypass -File .\run.ps1 <options>` copies `src/lane_follow.py` and runs it.
Ctrl+C stops the car (never Ctrl+Z). The student pastes the terminal text back. Many turns are expensive, so
prefer designs that give a lot of information per run (log pose, path error, and so on) and fail safe.

## Open questions to ask the student first
1. `map_nodes.txt` from `tools/dump_map.py` (node ids and coordinates).
2. Which node and heading is the usual start? A photo of the car in place.
3. Real distance of `--turntest S` (tape measure) to check the tach scale.
4. His route drawing on the mat: which intersections need L, S, R, which roundabout exit.
