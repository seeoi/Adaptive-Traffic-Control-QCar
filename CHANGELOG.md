# Changelog

Every change to the code gets one entry here, newest on top.
The version number is also stored in `VERSION` at the top of `src/lane_follow.py`
and printed when the script starts, so you can tell which version is on the car.

Version numbers: `MAJOR.MINOR.PATCH`
- PATCH (1.0.1): small fix or a tuned number
- MINOR (1.1.0): new feature, still works the same way
- MAJOR (2.0.0): big change in how the car drives

## 2026-11-09 - FIRST FULL LOOP WITH THE ROUNDABOUT (2.3.1, car 6)
- `--route 22,9,13,19,17,20,22 --vmax 0.2`: the whole 13.5 m loop in 68 s - left street, curve, LEFT at the intersection,
  central street, into the roundabout, around it, out to the top street, back down the left street - and it stopped at
  the start ("lo hizo muy bien"). Only issue: slightly wide (outside) in the roundabout; the steering is close to its left
  limit there (+0.49 of 0.50). The start check did not run (the camera saw the line in only 3 of 5 rows).

## 2.3.2 - 2026-11-09
- Start check now accepts 3 of 5 rows (it needed 4).

## 2.3.1 - 2026-11-09
- Test of 2.3.0 (photo): in the first LEFT curve after node 22 the car was too wide, on the white dashed line, while the
  estimate said 1 cm. With the camera now off in curves and the start straight only ~1 m long, nothing corrected the
  hand-placed start position any more (before, the camera did it on that straight).
- New START CHECK: while the car stands still for the gyro (now 1.5 s), the camera measures how far the car is from the
  lane center, and the start position is corrected (up to 12 cm). The camera window on straights is also shorter
  (0.3 m behind to 0.5 m ahead), so it can help on short straights.

## 2.3.0 - 2026-11-09
- Loop test of 2.2.1 (2 laps): 1st roundabout: entering it cut into the white island a bit, then went wide outside;
  2nd lap: drove on the yellow line near node 22 and went out on the right after the left curve; the 2nd pass through the
  roundabout was good. The SSH display dropped at 34 s but the car kept driving (see the warning in the README).
- Pattern over all tests: the parts driven by the MAP ONLY (intersection 1.5 m, roundabout 2.5 m) are accurate; the
  5-10 cm errors appear where the camera is in charge in curves and merges (it guesses the lane center from the yellow
  line, which is wrong in curves and near the yellow island triangles).
- The camera is now used only on almost straight road (`CAM_MAX_CURV` 0.35 1/m, checked 0.5 m behind to 0.7 m ahead),
  both for steering and for correcting the position. Curves are driven by the map + wheels + gyro.

## 2.2.1 - 2026-11-09
- First full loop with the roundabout (`--route 22,9,13,19,17,20,22 --laps 2 --vmax 0.2`, 2.2.0): the left street, the
  curve and the LEFT turn at the intersection were fine; it entered the roundabout and went around it, but about 15 cm
  too far OUT (photo: at the outer edge of the mat) and was stopped there. In the log the estimate said it was on the
  path the whole time inside the roundabout (2.5 m with no camera), so the error came from BEFORE: going up the
  central street and the diagonal to the roundabout the camera pulled the estimate up to 10 cm to the side. Near the
  roundabout there are yellow traffic-island triangles that can fool the yellow-line detector.
- Camera corrections are now OFF inside `NO_CAMERA_ZONES` (a 1.3 m circle around the roundabout), and a camera reading
  must agree with the estimate within 10 cm (`CAM_GATE` 0.15 -> 0.10).
- Roundabout facts from the map: center (1.58, 3.82) m, lane radius 0.68 m (car 6 can turn 0.60 m to the left).

## 2.2.0 - 2026-11-09
- `--circletest` on car 6 (slow, full lock): LEFT circle 0.60 m, RIGHT circle 0.84 m. Asking for 0.6 instead of 0.5
  gave the same circles (the car's own limit is 0.5). So the steering is NOT symmetric: real angle =
  0.70 * command + 0.054 rad (pointed a little to the left at "0"), and right turns can never be tighter than 0.84 m.
  That is why the right turn always ended wide (on the yellow line).
- Steering now uses this model instead of the single `STEER_GAIN` 1.7: `STEER_SCALE` 0.70, `STEER_BIAS` 0.054.
- Corners are rounded per direction: left at least `MIN_RADIUS_LEFT` 0.66 m, right at least `MIN_RADIUS_RIGHT` 0.92 m.
- Good news for the roundabout (left turns, about 0.6 m): the car CAN turn that tight to the left.
- Not tested on the car yet.

## 2.1.2 - 2026-11-09
- Photo after `22,9,0` with 2.1.1: the car ended over the yellow line, about 10 cm to its LEFT of the lane (the outside
  of the right turn), same as before. The shorter look-ahead did not change it: the steering is at its limit in this
  turn, so the car simply cannot turn tighter. Next: find out if the steering can go further than 0.5 rad.
- `--circletest --lock 0.6` drives the circles with a chosen steering command, to compare the real radius at 0.5 and 0.6.

## 2.1.1 - 2026-11-09
- Real test of 2.1.0 `--route 22,9,0`: arrived exactly at the destination, but during the right turn it cut the corner
  on the RIGHT. (Only 2 cm of reshaping was needed for this corner.) Earlier, in left curves, it touched the yellow
  line (also the inside).
- Cause: pure pursuit "cuts corners" more when it looks farther ahead. Look-ahead shortened: 0.30 + 0.60*speed ->
  0.22 + 0.50*speed (at 0.2 m/s: 0.42 m -> 0.32 m), so the car follows the curve closer and starts the turn a bit
  later (more "open" before the turn). Not tested on the car yet.
- Also added an optional `CORNER_SHIFT` (move curves to the outside); it is OFF because on paper it made the ends of
  the turns tighter than the car can drive.

## 2.1.0 - 2026-11-09
- The logs (map_003619/003823/004225.csv) showed the real problem of the right turn: at full lock (-0.50) car 6
  drives a circle of only 0.80-0.84 m radius, but the map's right turn at the central intersection is about 0.72 m.
  The car cannot follow it, so it goes wide, and the outside of a right turn is the yellow line. In straight parts the
  camera and the estimate agree within a few cm, so the camera was not the cause.
- New: before driving, every map corner tighter than `MIN_RADIUS` (0.90 m) is replaced by an arc of that radius
  touching the same entry and exit road (start the turn a bit earlier, cut the corner a bit). For the 90 deg right
  turn the middle of the turn moves about 7 cm to the inside. The program prints how far the path was moved.
  The roundabout (about 0.6 m) is NOT changed this way; it will need its own solution.
- `TIGHT_EXTRA` (2.0.8) set to 0, the reshaped corner replaces it.
- New `--circletest`: drives slow full-lock circles left and right and prints the real radius (needs 2 x 2 m of
  free floor).

## 2.0.10 - 2026-11-09
- Undid 2.0.9: in the simulator, trusting the camera less made the car WORSE (12-15 cm instead of 8-10 cm), even with
  a biased camera. Back to `CAM_STEER_MIX` 0.7 and `FIX_LATERAL` 2.5 until the real logs (map_*.csv) show the cause.

## 2.0.9 - 2026-11-09
- Real test of 2.0.8 `--route 22,9,0`: the right turn ended MORE to the left (closer to the yellow line), not less.
  Pattern in all runs: in the curve before the intersection the car rides left (touches the yellow line) while the
  camera is in charge, and after the blind turn it ends left by about the same amount. Likely cause: the camera's idea
  of the lane center is pulled toward the yellow line, and the camera both steers the car (70 %) and moves the
  position estimate, so the error is carried into the blind turn.
- The map now has more say: `CAM_STEER_MIX` 0.7 -> 0.3, `FIX_LATERAL` 2.5 -> 0.8 (the wheel sensor and the gyro
  proved accurate, so the estimate needs only a gentle camera correction). Not tested on the car yet.

## 2.0.8 - 2026-11-09
- Real test `--route 22,9,0 --vmax 0.2` (2.0.7): the RIGHT turn worked, but it was a bit wide: the steering stayed at
  the limit (-0.50) through the whole turn and the front-left wheel just touched the yellow line.
- Tight corners (curvature over 1/m) now get 0.15 m more look-ahead, so the car starts turning a little earlier.
  Not tested on the car yet.

## 2026-11-09 - 2.0.7 works STRAIGHT through the intersection
- `--route 22,9,7 --vmax 0.2` on car 6: went straight across the intersection and stopped exactly at node 7
  ("esta vez sí fue derecho y paró exactamente donde debía"). After 1.5 m with no yellow line the camera found the
  east street's line again and the estimate was only 3-6 cm off. In the curve the estimate is still 5-8 cm to the left.

## 2.0.7 - 2026-11-09
- Real test `--route 22,9,7` (straight through the intersection): fine until the curve ended, then in the
  intersection (no yellow line) the car drove at about 8-10 deg to the RIGHT while it believed it was going
  straight (estimate heading 0..+2 deg, 1 cm off), and left the track on the right.
- Gyro checked by hand: one full turn = 359.3 deg, so the gyro is accurate. The heading error came from the camera
  correction: the heading was nudged by the camera's sideways reading, so a small fixed offset in that reading
  (a few cm) became a heading error of several degrees (offset / 0.4 m).
- Fix: the camera no longer changes the heading that way. The heading comes from the gyro; it is only aligned
  with the road on STRAIGHT road while the camera sees the car centered and the wheels are almost straight
  (`ALIGN_HEADING`). Simulated with a 4 cm camera offset: within 10 cm. Not tested on the car yet.

## 2026-11-09 - FIRST SUCCESSFUL MAP DRIVE (2.0.6, car 6)
- `--route 22,9,13 --vmax 0.2` with the motor: drove down the left street, the curve, crossed the intersection and
  turned LEFT into the correct lane, stopping at node 13 ("lo hizo perfecto"). Speed steady at 0.18-0.22 m/s
  (push 0.067-0.078), estimate within +-5 cm of the path the whole way, loop 130-150 Hz with the camera.
- Small issue: in the first (left) curve the car touched the yellow line on its left (estimate +3..+5 cm left).

## 2.0.6 / lane_follow 1.2.3 - 2026-11-09
- `--motortest` again with `READ_MODE = 0`: the wheel speed now reacts within 0.1 s (0.06 -> 0.24 m/s,
  0.07 -> 0.31 m/s, 0.08 -> 0.37 m/s, push 0 -> stopped at once; wheels in the air). The 2 s lag is gone.
- `lane_follow.py` 1.2.3: same fix (`readMode=0`). Its old speed overshoots were probably the same lag.

## 2.0.6 - 2026-11-09
- `--motortest` (car 6, wheels in the air) found the real cause of the stop-and-surge: every change of the motor push
  showed up in the wheel speed exactly 2.0-2.1 s LATER (0.06 at 2.0 s -> speed at 4.1 s; 0.07 at 5.0 -> 7.1;
  0.08 at 8.0 -> 10.0; 0 at 11.0 -> 13.1). The steps were sharp, so the readings were simply OLD: with
  `readMode=1` the QCar keeps a queue of readings and our loop read them slower than they arrive (camera work,
  setup time), so it fell 2 s behind. The gyro was late the same way, which also broke the turns.
- Fix: `READ_MODE = 0` (read the latest values every time). `lane_follow.py` probably has the same problem
  (not changed yet).
- Also measured: push 0.06 -> 0.22 m/s, 0.07 -> 0.29 m/s, 0.08 -> 0.36 m/s (wheels in the air).

## 2.0.5 - 2026-11-09
- Motor test of 2.0.4 (`--route 22,9,13 --vmax 0.2`): still stop-and-surge. The log shows the wheel speed staying at
  0 for about 1 s while the push rose to 0.15, then the car surging to 0.6-0.68 m/s while the push was already 0.
  So the motor (or the wheel sensor) reacts LATE, and a strong speed controller makes it worse. At the intersection the
  car went off the track to the right.
- Throttle is back to what worked in lane_follow 1.0.2: fixed push `THROTTLE_BASE` 0.07 + small correction
  (`KP_V` 0.10, `KI_V` 0.30), max push 0.11, no "stuck" boost; the motor is cut only above target + 0.15 m/s.
- The main loop now runs at a steady 200 Hz (before it ran as fast as possible and flooded the car with commands,
  which may add delay). The status line shows `loop=..Hz`.
- New `--motortest` (car ON A BOX, wheels in the air): fixed pushes 0, 0.06, 0.07, 0.08, 0 and prints the wheel
  speed every 0.1 s, to measure the dead zone and the delay.

## 2.0.4 - 2026-11-09
- First MOTOR test, `--route 22,9,13 --vmax 0.25` on car 6: the street and the curve were fine (camera locked,
  estimate within +-7 cm), but at the intersection it went wrong. Problems seen in the log:
  1. Speed ran away: up to 0.53 m/s with a 0.25 limit, then the car stopped dead for 1-1.5 s, then surged again.
  2. During a surge at the intersection (no yellow line) the estimate drifted to 19 cm off the path.
  3. In a curve the "along the path" camera correction pinned the estimate in place (x, y frozen while the
     wheels read 0.5 m/s), so the car lost track of where it was.
- Fixes: removed the along-path correction (the wheel sensor is accurate to 1 %); much smaller feed-forward
  throttle; the motor is cut when the car is 0.06 m/s faster than wanted; a small extra push when the car is stuck;
  the car slows to 0.15 m/s when its estimate is more than 10 cm from the path. The status line shows `thr=`.
- Check the battery: stop-and-go like this can also be low voltage.

## 2.0.3 - 2026-11-09
- Real test `--route 22,9,13 --dry` on car 6 (pushed by hand, wheels steering themselves): WORKED. The camera
  locked on the yellow line along the left street and the curve (estimate within +-4 cm), then the car crossed
  the intersection and turned LEFT into the correct lane, ending at node 13 (student: "lo hizo bien").
  In the left turn the steering hit the limit (0.50) and the estimate went 9 cm wide, then the camera pulled it
  back to 0-1 cm. So that turn is about as tight as the car can do.
- New `--vmax M/S` option to set the top speed for a test (for example `--vmax 0.25` for the first motor run).

## 2.0.2 - 2026-11-09
- First real test, `--route 9,13 --dry` on car 6 (pushed by hand, wheels steering by themselves): the left turn
  had the right shape, but the car ended about 10 cm too far LEFT (close to the yellow line) while the estimate
  said +2..6 cm. Likely cause: the car was placed a little behind node 9 or turned a few degrees, and on this
  short route the camera saw no yellow line until the turn was almost done (yellow 0/5 until s = 0.9 m), so it
  could not fix the start error in time.
- The camera now has more say when it sees the line: `CAM_STEER_MIX` 0.5 -> 0.7, `CAM_GATE` 0.12 -> 0.15 m,
  `FIX_LATERAL` and `FIX_HEADING` 1.5 -> 2.5.
- Test advice: start routes on a road WITH a yellow line before the intersection (for example `22,9,13`), so
  the camera fixes the start position before the turn.

## 2.0.1 - 2026-11-09
- Calibrated on car 6 (qcar-67815) with `--odotest`: pushed 1.00 m -> wheel sensor 1.01 m, so `TACH_SCALE = 1.0` is right.
  A left turn read +92 deg, so `GYRO_SIGN = +1` is right.
- Because the wheel sensor is accurate, the wide turns seen before are real: steering 0.40 gave about a 1.06 m
  circle instead of the 0.60 m the bicycle model predicts. `STEER_GAIN` 1.0 -> 1.7 to make up for it.
  Smallest possible circle is then about 0.85 m (steering limit 0.5).

## 2.0.0 - 2026-11-04 (new program: `src/map_drive.py`)
- NEW map-based driving. The route is a list of map nodes (`--route 9,13`); the Quanser road map turns it into a
  path along the correct lane through intersections and the roundabout. The car tracks its own position with
  wheel speed + gyro (dead reckoning) from the first node, and the camera corrects that estimate (sideways,
  heading, and along the path in curves) whenever it sees the yellow line. Steering = pure pursuit on the
  path, mixed 50/50 with the camera where the line is visible. Speed slows down BEFORE curves (read from the
  map) with smooth acceleration. `--lidar` stops for objects ahead (not needed for position).
- `--graph` prints every node and where it can go (LEFT / STRAIGHT / RIGHT). `--odotest` (motor off) measures
  the wheel-sensor scale and the gyro sign by pushing the car. `--dry` keeps the motor off.
- New `map.ps1`: copies `lane_follow.py` + `map_drive.py` to the car and runs `map_drive.py`.
- Map facts (from `tools/dump_map.py` on car 7 = qcar-67815): 24 nodes, 42 edges. Central 4-way crossing near
  (0.13, 0.97) m; from node 9 (coming from the west): LEFT = 13, STRAIGHT = 7, RIGHT = 0. Roundabout = nodes
  16-19, counter-clockwise. Corner marks on the mat match nodes 11/12 and 0/1.
- Simulated only: within 6-9 cm of the lane with correct calibration; 11-22 cm if the wheel sensor is 10% off,
  so `TACH_SCALE` must be measured first. NOT tested on the real car.
- `lane_follow.py` is unchanged (still 1.2.1) and still works on its own.

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
