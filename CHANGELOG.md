# Changelog

Every change to the code gets one entry here, newest on top.
The version number is also stored in `VERSION` at the top of `src/lane_follow.py`
and printed when the script starts, so you can tell which version is on the car.

Version numbers: `MAJOR.MINOR.PATCH`
- PATCH (1.0.1): small fix or a tuned number
- MINOR (1.1.0): new feature, still works the same way
- MAJOR (2.0.0): big change in how the car drives

## 1.0.0 - 2026-10-21
- First clean version, based on the old `TestGimmik4.py`.
- Throttle can never be negative (fixes the fast forward/backward jerking).
- Fixed base throttle plus a small correction, so the car starts moving right away.
- Smoother steering (filter + limit on how fast the wheels can turn).
- `--dry` mode (everything runs, motor off) and a live status line on screen.
- Stop lines and LiDAR are OFF by default for the first full-track test.
- Status: tested only with a simulated car. NOT yet tested on a real QCar.
