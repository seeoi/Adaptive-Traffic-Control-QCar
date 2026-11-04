# Changelog

Every change to the code gets one entry here, newest on top.
The version number is also stored in `VERSION` at the top of `src/lane_follow.py`
and printed when the script starts, so you can tell which version is on the car.

Version numbers: `MAJOR.MINOR.PATCH`
- PATCH (1.0.1): small fix or a tuned number
- MINOR (1.1.0): new feature, still works the same way
- MAJOR (2.0.0): big change in how the car drives

## 1.2.2 - 2026-11-04
- New `tools/where_am_i.py`: prints the car position (x, y, heading) from the LiDAR/map and the closest map node.
  No driving. Use it to check if the old LiDAR calibration is right, before building map-based driving.

## 1.2.1 - 2026-11-04
- Smoother, more natural driving. The logs showed `target` jumping between 0.35 and 0.20 almost every line,
  which made the motor push on and off. Now the target speed goes down SMOOTHLY with the steering angle
  (`CURVE_START`, `CURVE_FULL`) and the speed command can only change slowly (`ACCEL` 0.25, `DECEL` 0.60 m/s^2).
  Stops (lane lost, camera dead, obstacle) are still immediate.
- Status: only simulated.

## 1.2.0 - 2026-11-02
- New `--arm-m METERS`: the route is ignored until the odometer reaches this distance. Before that, every place
  where the yellow line ends is only reported (`*** Yellow line ended at odo=X m`). This lets the route start
  at the right corner of a longer drive. The status line now also shows `odo=` (meters driven).

## 1.1.6 - 2026-11-02
- Status line shows `turned=..deg dist=..m` during a maneuver, to see how the turn really goes.
- Real tests of `--route R` (v1.1.5): the `yellow_end` trigger fired. The right turn was entered at 0.37 m/s
  with full lock, the car slowed to 0.05-0.1 m/s, never saw the yellow line again (0/5) and kept turning right.
  Starting a route test with NO yellow line in view never triggers (use `--turntest` there).

## 1.1.5 - 2026-11-02
- Our mat has NO white stop lines at the intersection. New default route trigger `ROUTE_TRIGGER = "yellow_end"`:
  when the yellow line was seen well and then disappears for 0.3 s, the car starts the next maneuver of the route.
  After a maneuver it ignores new triggers for 6 s. (`"stop_line"` is still available.)
- The car no longer waits for the (fake) green light before a route maneuver.
- Status: only simulated.

## 1.1.4 - 2026-11-02
- Real test of `--route R` (car 7): the car followed the curve, the yellow line ended (`yellow=0/5`) and the
  car NEVER reported `Stop line seen`, so it never started the right turn. Cause: the stop-line check needs the
  yellow line to be visible too, and at the stop line the yellow has already ended. Not fixed yet: we need a
  camera picture taken at the stop line.
- Debug pictures: one every 0.25 s (before: ~15 per second), 80 files = the last 20 s, with the time on them.

## 1.1.3 - 2026-11-02
- Real tests of `--turntest R` (car 7, twice, v1.1.2): the camera took over at 81 deg after 1.64 m both times.
  The turn is too WIDE (radius about 1.1 m): at about 7 s the car crossed to the left of the yellow line of
  the new road. Run 1 then followed the left curve well; run 2 had lost the line (yellow 1/5) and drove straight.
  Fix: `R` now starts the turn almost at once (0.05 m instead of 0.15) and uses full lock (-0.50 instead of -0.40).
- Note: after the turn the speed went up to 0.41 m/s (target 0.20). The motor push (`THROTTLE_BASE`) is too big for
  slow targets; not changed yet. If it is too fast, go back to `V_MAX` 0.30 and `THROTTLE_BASE` 0.07.

## 1.1.2 - 2026-11-02
- Real test of `--turntest R` (car 7): the right turn was good, but it kept full right steering until the
  gyro counted 90 deg (9.6 s, 1.93 m) even though the camera saw the yellow line from 7.6 s, so it
  turned right again when the road already curved left. Fix: in a turn, once at least half of the angle is
  done and the camera sees the yellow line in 4+ of 5 rows for 0.25 s, the camera takes over
  (`ARC_MIN_FRACTION`, `HANDOVER_ROWS`, `HANDOVER_S`).
- `--turntest` now keeps following the line for 4 s after the maneuver (`TURNTEST_FOLLOW_S`), so you can see
  the camera handover.
- Measured: with steering -0.40 at 0.2 m/s the car turns about 11 deg/s (about 1 m radius).

## 1.1.1 - 2026-11-02
- Real test of `--turntest S` (car 7): it drove 1.00 m and stopped as designed (the test ends after one
  maneuver), but drifted LEFT (turned 11 deg in 1 m) because the wheels were held at 0. Gyro works.
  Fix: in "straight" steps the car keeps following the yellow line while the camera sees it
  (3+ of 5 rows); it holds the wheels at 0 only when it does not see the line.

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
