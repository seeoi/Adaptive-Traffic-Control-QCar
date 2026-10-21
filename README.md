# Adaptive Traffic Control with QCars (Senior Design)

Goal: an adaptive traffic-light system for a small smart-city testbed, using Quanser
QCar 2 vehicles, a Raspberry Pi traffic-light controller and (later) external cameras.

**Current milestone: make ONE QCar drive around the whole track reliably.**
Traffic lights, V2I, multiple cars and external cameras come after that.

## Folder guide

| Path | What it is |
| --- | --- |
| `src/lane_follow.py` | **The script to use.** Follows the yellow center line with the front camera. |
| `old/` | Earlier attempts, kept only for reference. Do not drive with them. |

## Quick start (about 5 minutes)

1. Connect your laptop to the QCar's Wi-Fi network. The car we used is `nvidia@qcar-67815` at `192.168.2.183` (user and password: `nvidia`).
2. Copy the script from your laptop to the car (run this on the LAPTOP):
   `scp src/lane_follow.py nvidia@192.168.2.183:~/`
3. Open an SSH session on the car: `ssh nvidia@192.168.2.183`
4. On the car, once per terminal:
   `export PYTHONPATH=~/Documents/Quanser/libraries/python`
5. Run the tests in this order (the car does not move in 1 and 2):
   1. `python3 lane_follow.py --camtest` - saves `debug/cam_lanes.jpg`. Orange dots must sit on the yellow line.
      If it prints **NO CAMERA IMAGES ARRIVED**, the problem is the camera, not the code.
   2. `python3 lane_follow.py --dry` - runs everything with the motor OFF. Move the car by hand and check that `yellow=` shows 3/5 or more.
   3. `python3 lane_follow.py` - drives. Put the car in the RIGHT lane with the yellow line on its LEFT. **Ctrl+C stops it.**

Do **not** use `sudo`: it fails with a privileges error on our cars.
To stop a stuck program from another terminal: `pkill -f lane_follow.py`.
To download debug pictures to the laptop: `scp -r nvidia@192.168.2.183:~/debug .`

## Tuning (all numbers are at the top of `src/lane_follow.py`)

| Problem | Change |
| --- | --- |
| Car is too slow or does not start | Raise `THROTTLE_BASE` by 0.01 |
| Car zig-zags | Lower `KP` (try 0.6) |
| Car goes wide in curves | Raise `KP` |
| Wheels turn the wrong way | Set `STEER_SIGN = -1` |
| Car drifts to one side on a straight road | Adjust `STEER_TRIM` by 0.02 at a time |
| Yellow line is not detected | Lower `YELLOW_DELTA` (12 -> 8) |

## Known problems and lessons learned

* **Fast forward/backward jerking:** the old speed controllers could output a negative throttle,
  and on the QCar negative throttle = reverse. `lane_follow.py` clamps the throttle to 0..`MAX_THROTTLE`.
* **Camera image has a pink tint**, so fixed color ranges for yellow do not work. The code compares
  pixels against the road color instead.
* **Old LiDAR scripts** (`old/`) use a saved room scan. The last position check was about 2 m and 180 degrees
  off, so the stored calibration is probably wrong. Calibration pose: x = 0, y = 2, heading -90 degrees
  (road arm of the traffic-light intersection that leads to the roundabout, right lane, facing the intersection).
  Recalibrate if the mat or furniture moves.
* The previous team's code (Raspberry Pi + Flask + V2I) is **not in this repo yet**. Add it in its own folder.

## Versions (how we keep the code in order)

* **One source of truth:** the `main` branch on GitHub. Edit on the laptop, never directly on the car.
* **Before you start:** `git pull`.
* **Every change:** update `VERSION` in `src/lane_follow.py`, add a line to `CHANGELOG.md`, then
  `git add -A`, `git commit -m "what changed and why"`, `git push`.
* **Put it on the car:** `scp src/lane_follow.py nvidia@192.168.2.183:~/`. The script prints
  `lane_follow.py version X.Y.Z` when it starts - check it matches the changelog.
* **After a real test on the car:** write the result in `CHANGELOG.md` (worked / what failed).
* **Milestones get a tag:** `git tag v1.1.0` then `git push --tags`
  (for example, when the car completes the full track).
* **Never** edit a file with nano on the car: that change never reaches GitHub.

## Roadmap

- [ ] One QCar completes the full track with the camera only
- [ ] Add LiDAR obstacle stop (`--lidar`) and test it
- [ ] Review and import the previous team's code (Flask server on the Raspberry Pi)
- [ ] Install and configure the physical traffic lights
- [ ] Replace the fake light in `light_is_green()` with the real controller / V2I message
- [ ] Adaptive timing: choose which lane gets green AND for how long (wait time, occupancy)
- [ ] Multiple QCars sharing one traffic light
- [ ] External cameras to detect cars without V2I
