# Changelog

Every change to the code gets one entry here, newest on top.
The version number is also stored in `VERSION` at the top of `src/lane_follow.py`
and printed when the script starts, so you can tell which version is on the car.

Version numbers: `MAJOR.MINOR.PATCH`
- PATCH (1.0.1): small fix or a tuned number
- MINOR (1.1.0): new feature, still works the same way
- MAJOR (2.0.0): big change in how the car drives

## 1.1.0 - 2026-11-02
- NEW route and maneuvers: `--route S,L,R,RB2` makes the car do one decision per stop line
  (S straight, L left, R right, RB1/RB2/RB3 roundabout exit 1/2/3) and repeat the list forever.
  A maneuver = drive a distance (wheel speed) + turn an angle (gyroscope), then back to the camera.
- NEW `--turntest L` does ONE maneuver right away (for calibration) and prints how far it drove and
  how many degrees it turned.
- Stop lines are used automatically when a route is given. The car now waits at least `MIN_STOP_S` (1 s).
- Safety: a maneuver is cut after `MANEUVER_MAX_S` (12 s); if the camera dies the car stops.
- Status: only simulated. The maneuver numbers in `MANEUVERS` are FIRST GUESSES, NOT calibrated on the real
  mat. Stop-line detection has never been tested on the real mat either.

## 1.0.3 - 2026-11-02
- Slightly faster: `V_MAX` 0.30 -> 0.35 and `THROTTLE_BASE` 0.07 -> 0.08. NOT yet tested at this speed:
  if the car zig-zags, lower `KP` (0.9 -> 0.7); if it is still too fast, go back one step.
- New `tools/dump_map.py`: prints the road-map nodes (no driving). Needed to plan intersection
  and roundabout maneuvers.

## 1.0.2 - 2026-11-02
- Tested on a REAL QCar (car 5): one full lap of the lab track, smooth and inside the lane,
  at about 0.22 to 0.28 m/s. Camera test found the yellow line in 5 of 5 scan rows.
- `LOST_TIME` raised from 1.0 to 3.0 s: in a curve the line can disappear for a moment and the
  car used to stop there.
- Safety: camera watchdog. If the camera sends no picture for `CAMERA_TIMEOUT` (0.5 s) the car stops
  at once and prints a warning; after `CAMERA_GIVE_UP` (5 s) the run ends. Before, a dead camera
  only showed up as `LOST` after the lane timer ran out.
- New `run.ps1`: copies the script to the car and runs it with one command from the laptop
  (set to car 7, 192.168.2.183; the IP is at the top of the file).
- Known problem: the camera sometimes stops sending pictures, and one run ended with a
  segmentation fault when closing. After that the camera can fail until the car is restarted.
  Possible causes: camera cable, low battery voltage, or how the camera is released on exit.

## 1.0.1 - 2026-10-21
- Safety: the program now also stops the motor cleanly when the SSH connection closes
  (SIGHUP) or it is terminated (SIGTERM). Before, only Ctrl+C did that.
- Safety: default run time lowered from 300 s to 60 s, and new `--time SECONDS` option.
  If the Wi-Fi drops you cannot press Ctrl+C, so this timer is what stops the car.
- Status: simulated tests only. NOT yet tested on a real QCar.

## 1.0.0 - 2026-10-21
- First clean version, based on the old `TestGimmik4.py`.
- Throttle can never be negative (fixes the fast forward/backward jerking).
- Fixed base throttle plus a small correction, so the car starts moving right away.
- Smoother steering (filter + limit on how fast the wheels can turn).
- `--dry` mode (everything runs, motor off) and a live status line on screen.
- Stop lines and LiDAR are OFF by default for the first full-track test.
- Status: tested only with a simulated car. NOT yet tested on a real QCar.
