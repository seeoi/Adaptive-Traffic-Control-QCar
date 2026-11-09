# Road map nodes (Quanser SDCS mat, `SDCSRoadMap(leftHandTraffic=False)`)

Printed by `map.ps1 --graph` on car 6 (2026-11-09). x to the right, y up (roundabout = top right), heading in degrees
(0 = right/east, +90 = up/north). A route is a list of these numbers; the car starts on the FIRST node.
LEFT / STRAIGHT / RIGHT is computed from the node headings, so it is only a hint (roundabout labels are rough).

| Node | x (m) | y (m) | Heading | Where it can go next | Where it is |
| --- | --- | --- | --- | --- | --- |
| 0 | +0.00 | +0.13 | -90 | 2 (LEFT) | central street, going down, south of the central crossing (lower corner mark) |
| 1 | +0.27 | +0.08 | +90 | 7 (RIGHT), 8 (LEFT), 13 (STRAIGHT) | central street, going up into the central crossing |
| 2 | +1.13 | -1.08 | 0 | 4 (LEFT) | bottom street, going right |
| 3 | +1.13 | -0.81 | 180 | 1 (RIGHT) | bottom street, going left |
| 4 | +2.25 | +0.08 | +90 | 6 (LEFT), 14 (STRAIGHT) | right street, going up |
| 5 | +1.98 | +0.08 | -90 | 3 (RIGHT) | right street, going down |
| 6 | +1.01 | +1.10 | 180 | 0 (LEFT), 8 (STRAIGHT), 13 (RIGHT) | street east of the central crossing, going left into it |
| 7 | +1.24 | +0.83 | 0 | 5 (RIGHT), 14 (LEFT) | street east of the central crossing, going right |
| 8 | -0.75 | +1.10 | 180 | 10 (U-TURN), 23 (RIGHT) | street west of the central crossing, going left |
| 9 | -0.75 | +0.83 | 0 | 0 (RIGHT), 7 (STRAIGHT), 13 (LEFT) | street west of the central crossing, going right into it |
| 10 | -1.28 | -0.46 | -42 | 1 (LEFT), 2 (STRAIGHT) | bottom-left diagonal, going down-right |
| 11 | +0.00 | +2.16 | -90 | 12 (STRAIGHT) | central street, going down (upper corner mark) |
| 12 | +0.00 | +1.85 | -90 | 0 (STRAIGHT), 7 (LEFT), 8 (RIGHT) | central street, going down into the central crossing |
| 13 | +0.27 | +1.85 | +90 | 19 (RIGHT) | central street, going up, north of the central crossing |
| 14 | +2.25 | +2.97 | +90 | 16 (U-TURN), 20 (LEFT) | right street, going up toward the roundabout |
| 15 | +1.98 | +1.85 | -90 | 5 (STRAIGHT), 6 (RIGHT) | right street, going down |
| 16 | +0.91 | +3.71 | -81 | 17 (LEFT), 18 (RIGHT) | roundabout, west side |
| 17 | +1.47 | +3.15 | -9 | 15, 16, 20 | roundabout, south side |
| 18 | +0.62 | +3.07 | -138 | 11 (LEFT) | roundabout exit toward the central street |
| 19 | +0.79 | +2.86 | +42 | 17 (RIGHT) | roundabout entry from the central street |
| 20 | +0.00 | +4.50 | 180 | 22 (LEFT) | top street, going left |
| 21 | +0.00 | +4.23 | 0 | 16 (RIGHT) | top street, going right (toward the roundabout) |
| 22 | -1.98 | +2.97 | -90 | 9 (LEFT), 10 (LEFT) | left street, going down |
| 23 | -1.72 | +2.97 | +90 | 21 (RIGHT) | left street, going up |

Examples: central crossing from node 9: LEFT `9,13`, STRAIGHT `9,7`, RIGHT `9,0`.
Quanser's example loop: `10,4,20,10`.
