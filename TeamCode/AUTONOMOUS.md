# AutoPath drive candidate

This candidate adapts the visualizer export to the project's installed Pedro 2.1.2 and Ivy core
1.1.1. It belongs in the existing `org.firstinspires.ftc.teamcode` package. It does not use the
Pedro 3 `PoseFactory`, `Paths` API, or Ivy Pedro 3 `follow()` adapter, and needs no dependency upgrade.

It drives and pauses. There are currently **no mechanism commands, ball sensors, confirmed
collection events, or scoring actions**. The arrival messages mark where those commands will
be connected after the hardware exists. The pauses are provisional, not proof that balls loaded.

## Candidate coordinates

All distances are inches. Headings below are degrees; the implementation converts to radians.
Coordinates assume the robot's localization origin is its geometric center, with its front
pointing along positive X at zero heading. This is a planning candidate in the left/red-side
orientation. Confirm that the visualizer, localizer, field origin, and loading zone match it.

| Step | Center X | Center Y | Heading |
| --- | ---: | ---: | ---: |
| Start, rear touching bottom wall | 56.0000 | 7.5000 | 90 |
| Straight departure | 56.0000 | 14.0000 | 90 |
| Original waypoint 1 | 42.6964 | 18.8631 | 135 |
| Flower staging | 24.0000 | 46.7669 | 180 |
| Flower approach | 13.7199 | 46.7669 | 180 |
| Straight retreat from flower | 24.0000 | 46.7669 | 180 |
| Loading staging | 24.0000 | 106.7500 | 180 |
| Loading approach | 13.5000 | 106.7500 | 180 |
| Straight retreat from loading | 24.0000 | 106.7500 | 180 |
| Return corridor | 24.0000 | 24.0000 | 180 |
| Inset return endpoint | 56.3562 | 12.5000 | 90 |

The start remains at the perimeter; wall clearance applies after the initial departure.
The original bottom-left stop is treated as the garden and has been replaced with a proposed
loading-zone stop farther up the left wall. **106.75 is an assumed loading center in this
coordinate frame, not a measured field location.** No human feeding is assumed to have happened.

Original export retained for reference:

| Point | X | Y | Heading |
| --- | ---: | ---: | ---: |
| Start | 56 | 8 | 90 |
| Point 1 | 42.6964 | 18.8631 | 135 |
| Flower point | 5.2199 | 46.7669 | 180 |
| Corner point | 4.8767 | 4.2428 | 270 |
| Finish | 56.3562 | 4.3380 | 90 |

## Gap settings and measurements

- `ROBOT_WIDTH = 17`, `ROBOT_LENGTH = 15` describe the approximate outer chassis footprint.
- `WALL_GAP = 1` is the nominal wall-envelope margin after departure. The software samples the
  rectangle along every straight segment using Pedro's actual interpolated heading. This is
  not a guarantee against localization error or unmodeled obstacles.
- `FLOWER_GAP = 1` is measured horizontally from the configured mechanism tip to the configured
  lower-opening face: `flower center X = FLOWER_FACE_X + FLOWER_TIP_REACH + FLOWER_GAP`.
- **`FLOWER_FACE_X = 5.2199` is only a planning reference taken from the original target point.**
  Measure the actual lower retrieval opening's field-facing surface and replace it. A robot
  center from an export does not establish where a flower surface is. Until measured, the
  one-inch physical gap is unverified.
- `FLOWER_TIP_REACH` and `INTAKE_TIP_REACH` are measured **from the robot center**, not from its
  bumper. Both default to 7.5, meaning flush with the front of the current chassis. Increase
  them when the mechanisms extend farther. The target poses move back accordingly.
- `LOADING_FRONT_GAP = 6` is an initial adjustable runway between the left wall and the intake
  tip; real feeding needs physical tests. The lower opening is a POLLEN retrieval target.
  Introduced NECTAR must first touch tile inside the loading zone before contacting a robot;
  entry also depends on the applicable NECTAR unlock rules.

Official field/setup references:
https://ftc-resources.firstinspires.org/ftc/field
https://ftc-resources.firstinspires.org/ftc/game

## Motion and completion

The candidate uses 35% transit power and 20% approach power, an 8-second timeout per driving
segment, and a 29-second overall limit. These values are starting points, not on-robot tuning.
A path ending or timing out internally does not automatically advance the routine: the
controller also requires settled position, heading, and speed. A leg timeout aborts subsequent
movement. STOP/exit cancels scheduling, breaks following, and independently attempts to zero
all four drive motors. Encoder reads use AUTO bulk caching in this independent OpMode.

Per-path constraints explicitly use the Pedro 2.1.2 constructor's actual semantics:
`t-value, end timeout in milliseconds, braking strength, braking start`. The existing global
constructor's third argument is 100 (braking strength); this candidate instead supplies local
constraints with braking strength 1, without modifying other code or tuning constants.

Existing encoder conversions, motor directions, velocities, mass and PID coefficients are
still placeholders that need physical calibration. The localizer's wheel geometry is separate
from the outer chassis footprint; the 17-inch body width should not blindly replace it.
The code does not perform a complete field-object collision check or validate mechanism height.

## Installation and verification

Intended destinations within `FtcRobotController`:

- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/AutoPath.java`
- `TeamCode/src/test/java/org/firstinspires/ftc/teamcode/AutoPathTest.java`
- `TeamCode/AUTONOMOUS.md`

The accompanying patch adds these files atomically and does not change existing source files.
The saved candidate has been compiled against the existing TeamCode source and cached SDK
dependencies. Thirteen new autonomous tests use real Pedro geometry and the real Ivy scheduler
with simulated drive feedback; the nineteen existing vision tests are also run. This is Java
compilation and hardware-free testing, not an Android APK build or a real robot/camera test.

After installation and with Gradle/Android dependencies available:

```sh
./gradlew :TeamCode:testDebugUnitTest
```
