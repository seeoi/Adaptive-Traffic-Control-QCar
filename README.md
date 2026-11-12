# Adaptive Traffic Control with QCars (Senior Design)

Goal: an adaptive traffic-light system for a small smart-city testbed, using Quanser
QCar 2 vehicles, a Raspberry Pi traffic-light controller and (later) external cameras.

**Current milestone: make ONE QCar drive around the whole track reliably.**
Traffic lights, V2I, multiple cars and external cameras come after that.

## Folder guide

| Path | What it is |
| --- | --- |
| `src/map_drive.py` | **New (2.0.0): drives a ROUTE of map nodes** using the map + wheels + gyro + camera. Run with `map.ps1`. |
| `src/lane_follow.py` | Follows the yellow center line with the front camera only (also used by `map_drive.py`). |
| `tools/` | Helper scripts that do not drive the car: `dump_map.py` (map nodes), `where_am_i.py` (car position on the map). |
| `map.ps1` | One command from the laptop: copies both scripts to the car and runs `map_drive.py`. |
| `run.ps1` | One command from the laptop: copies `lane_follow.py` to the car and runs it. |
| `old/` | Earlier attempts, kept only for reference. Do not drive with them. |

## Quick start (about 5 minutes)

1. Connect your laptop to the cars' Wi-Fi network (`Quanser_UVS_209-5G`, no internet). Read the car's IP on its
   screen: it can change after a restart. Cars we used: car 6 (`qcar-67815`) at `192.168.2.183`, car 5 = `nvidia@qcar-67798`
   at `192.168.2.217` (user and password: `nvidia`). Type the FULL IP.
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

**Faster way:** `run.ps1` does steps 2 to 5 in one command. Put the car's IP at the top of the file, then run
(from the repo folder, in PowerShell on the laptop):
`powershell -ExecutionPolicy Bypass -File .\run.ps1 --camtest`   (or `--dry`, or `--time 60`)

Do **not** use `sudo`: it fails with a privileges error on our cars.
Stop with **Ctrl+C**, never Ctrl+Z (Ctrl+Z only pauses it and leaves the camera locked: "GPIO is in use").
To stop a stuck program from another terminal: `pkill -9 -f lane_follow.py`.
To download debug pictures to the laptop: `scp -r nvidia@192.168.2.183:~/debug .`

**Safety:** If the SSH connection drops (Wi-Fi), the car may KEEP DRIVING until `--time` runs out, because Ctrl+C no
longer reaches it. Always use a short `--time` and stay next to the car, ready to pick it up.

## Map driving (v2.0.0) - first tests, in this order

All from the repo folder in PowerShell, laptop on the car Wi-Fi. Map node numbers: see `--graph`.
1. `powershell -ExecutionPolicy Bypass -File .\map.ps1 --graph` - lists the nodes and where each can go.
2. `... .\map.ps1 --odotest` - motor OFF. Push the car straight exactly 1.00 m (tape): `TACH_SCALE = 1.00 / distance`.
   Then push it around a LEFT corner: the turn must be POSITIVE (else `GYRO_SIGN = -1`). Write both numbers at the
   top of `src/map_drive.py`.
3. `... .\map.ps1 --route 9,13 --dry` - motor OFF. Car on node 9 (lane coming from the west into the central
   crossing). Push it through the LEFT turn to node 13; `off=` should stay within about 10 cm.
4. `... .\map.ps1 --route 9,13` - the same, driving.
5. Then a loop, for example `--route 10,4,20,10 --laps 3`.

## Full track: route and maneuvers (v1.1.0, needs calibration)

`python3 lane_follow.py --route S,L,R,RB2` - at every place where the yellow line ends the car does the next
decision of the list when the **yellow line ends** (our mat has no stop lines at the intersections; S straight, L left, R right, RB1/RB2/RB3 roundabout exit) and then goes back to the camera.
The list repeats forever.

**Calibrate ONE maneuver at a time** (the numbers in `MANEUVERS` are guesses):
1. Put the car on the lane, stopped right at the stop line of the intersection.
2. `python3 lane_follow.py --turntest L` (or `S`, `R`, `RB2`...). The car does only that maneuver and stops.
3. Look where it ended and read the printed line, for example `drove 1.20 m, turned 88 deg`.
4. Wrong? Change that maneuver in `MANEUVERS`: too short/long -> the meters; ends too far inside/outside the
   curve -> the steering number (bigger number = tighter turn); turned too little/much -> the degrees.
5. Repeat until it ends in the right lane facing the right way, 3 times in a row. Then test `--route`.

## Tuning (all numbers are at the top of `src/lane_follow.py`)

| Problem | Change |
| --- | --- |
| Car is too slow or does not start | Raise `THROTTLE_BASE` by 0.01 |
| Car zig-zags | Lower `KP` (try 0.6) |
| Car goes wide in curves | Raise `KP` |
| Wheels turn the wrong way | Set `STEER_SIGN = -1` |
| Car drifts to one side on a straight road | Adjust `STEER_TRIM` by 0.02 at a time |
| Yellow line is not detected | Lower `YELLOW_DELTA` (12 -> 8) |
| Car stops in curves (`LOST` in the status line) | Raise `LOST_TIME` (now 3.0) |
| Car is slow but steady | Raise `THROTTLE_BASE` by 0.01, then `V_MAX` by 0.05 |

## Known problems and lessons learned

* **Fast forward/backward jerking:** the old speed controllers could output a negative throttle,
  and on the QCar negative throttle = reverse. `lane_follow.py` clamps the throttle to 0..`MAX_THROTTLE`.
* **Camera sometimes stops** (`cam=0.0fps`, `CAMERA_STOPPED`): the car now stops by itself. After a crash on exit
  (`Segmentation fault`) or a Ctrl+Z the camera can stay locked: run `pkill -9 -f lane_follow.py`, and if it
  still fails, restart the car. Low battery (below about 10.5 V) can also drop the camera or the Wi-Fi.
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

**Design decision:** the car navigates with the CAMERA + wheel speed + gyro. LiDAR is only used to stop for
obstacles. Reason: we want to compare cars WITH and WITHOUT LiDAR later, so driving must not depend on LiDAR
localization (the old map scripts did, and their calibration was wrong).

Why the current script stops at intersections and roundabouts: it only knows "follow the yellow line", and
there is no yellow line there. It also has no idea WHERE it wants to go. The fix is a **route** (a list of
decisions) plus **maneuvers** (short moves done with wheel distance and gyro heading, then back to the camera).

- [x] 1. Follow the yellow line around the track (`lane_follow.py`, tested on car 5)
- [ ] 2. Stop at stop lines and wait for the (fake) light - code written in 1.1.0, NOT tested on the mat
- [ ] 3. Route list + intersection maneuvers: left / straight / right - code written, needs calibration
- [ ] 4. Roundabout exits 1, 2, 3 and route loop - code written, needs calibration
- [ ] 5. LiDAR obstacle stop (`--lidar`) tested
- [ ] 6. Review the previous team's code (Flask server on the Raspberry Pi) and install the physical traffic lights
- [ ] 7. Replace the fake light in `light_is_green()` with the real controller / V2I message
- [ ] 8. Read the traffic light with the camera
- [ ] 9. Multiple QCars sharing the track and the lights
- [ ] 10. Hybrid test: some cars with LiDAR, some without
- [ ] 11. Experiments: which setup gives the best traffic flow; adaptive timing (wait time, occupancy)
- [ ] 12. External cameras to detect cars without V2I


## Using ANOTHER car (quick procedure)

1. Laptop with internet: `git clone https://github.com/seeoi/Adaptive-Traffic-Control-QCar.git` (or `git pull`).
2. Turn the car on, read its IP on the car's screen, connect the laptop to the car Wi-Fi (`Quanser_UVS_209`).
3. Open PowerShell in the repo folder and tell `map.ps1` the car's IP (only for this window):
   `$env:QCAR_IP = "192.168.2.183"`   (password for every ssh/scp prompt: `nvidia`)
4. If the car has no profile yet (the program says "has no measured profile"): measure it once and send the output:
   - `powershell -ExecutionPolicy Bypass -File .\map.ps1 --odotest`   (push the car exactly 1.00 m by hand)
   - `powershell -ExecutionPolicy Bypass -File .\map.ps1 --circletest`   (needs ~2 x 2 m of free floor)
   Car 6 (`qcar-67815`) and car 7 (`qcar-67807`) already have profiles.
5. LiDAR calibration (picture: `docs/mapa1_calibrar.png`; once per car, again if furniture moves): put the car at x=0, y=2.0, facing -90 deg (central
   street, lane going DOWN toward the crossing, about 16 cm past the upper corner marks), nobody near it, then
   `powershell -ExecutionPolicy Bypass -File .\map.ps1 --gpscal`
6. Check it (picture of nodes 22 and 9: `docs/mapa2_salida.png`; motor off): `powershell -ExecutionPolicy Bypass -File .\map.ps1 --whereami` - put the car on node 22 and
   node 9: "cm away" should be under ~8 cm.
7. Drive: car on node 22 (left street, lane going down), then
   `powershell -ExecutionPolicy Bypass -File .\map.ps1 --route 22,9,13,19,17,20,22 --vmax 0.2 --time 90 --gps`
   and, when that is good, `powershell -ExecutionPolicy Bypass -File .\map.ps1 --random 22 --vmax 0.2 --gps`
   (Ctrl+C to stop).

### `--gpscal` says "Unable to download model .../QCarScanMatching/qcar2CaptureScan..."
That car is missing Quanser's LiDAR-position files (seen on car 9, 2026-11-11). Copy them from a car that has them
(car 3 = 192.168.2.168 keeps them in `~/Documents/Quanser/libraries/resources/applications/QCarScanMatching`) to the
folder the error names (here `/home/resources/applications/`):

    scp -r nvidia@192.168.2.168:~/Documents/Quanser/libraries/resources/applications/QCarScanMatching .
    scp -r QCarScanMatching nvidia@<car IP>:~/
    ssh nvidia@<car IP>
    sudo mkdir -p /home/resources/applications
    sudo cp -r ~/QCarScanMatching /home/resources/applications/
    sudo chmod -R a+rX /home/resources
    exit
