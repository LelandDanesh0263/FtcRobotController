# Optional shared vision

Vision is disabled by default: `BaseRobot` can be used without configuring or connecting a camera.
The existing robot core still needs its normal FTC hardware; `new Vision()` itself needs none.
This layer publishes observations and never drives motors, aims the turret, or starts scoring.

## What is implemented

- Separate optional camera configurations with GENERAL, FIELD, or TURRET roles.
- AprilTag processing, including the FTC SDK's BIOBUZZ hive-cell clusters.
- Optional color-blob candidate processors. These do not confirm an object's identity or distance.
- A shared immutable observation API, with timestamps, camera source, alliance, image bounds,
  and camera-relative pose when the detector actually provides one.
- Freshness filtering, complete-frame replacement, processor disabling, error reporting,
  and cleanup of every opened camera.
- Extension points for custom OpenCV or trained object detectors for flowers, hives, and robots.
  No model or validated recognition pipeline for those objects has been selected or installed.

## Enable a single camera later

Set `Vision.ENABLE_VISION = true` before constructing `BaseRobot` / pressing INIT.
The default camera name and resolution use the existing `Vision.WEBCAM_NAME`, `STREAM_WIDTH`,
and `STREAM_HEIGHT` settings. These settings are applied at initialization, not dynamically.
`MAX_OBSERVATION_AGE_SECONDS` can be adjusted while running; its initial value is a starting
point that must be checked against the eventual camera frame rate and control requirements.

## Multiple cameras, when hardware is chosen

Example configuration only; replace webcam names with the Driver Station configuration names:

```java
VisionCameraConfig fieldCamera = new VisionCameraConfig.Builder(
        "field", "YOUR_FIELD_CAMERA", VisionObservation.CameraRole.FIELD)
        .aprilTags(false)
        .build();
VisionCameraConfig turretCamera = new VisionCameraConfig.Builder(
        "turret", "YOUR_TURRET_CAMERA", VisionObservation.CameraRole.TURRET)
        .aprilTags(true)
        .build();
BaseRobot robot = new BaseRobot(hardwareMap, telemetry, gamepad1, gamepad2,
        fieldCamera, turretCamera);
```

Explicit camera configurations opt in independently of `ENABLE_VISION`. Missing or disabled
cameras produce empty observations and a status. Duplicate camera IDs or attempts to open
one webcam twice are rejected before opening hardware. Each webcam gets its own portal;
previews are disabled. Simultaneous cameras need real CPU/USB-bandwidth testing.

## Read results from any robot code

Call `robot.update()` once each loop, then use:

```java
robot.vision.getObservations();
robot.vision.getObservations(VisionObservation.Kind.POLLEN);
robot.vision.getObservations(VisionObservation.Kind.ROBOT);
robot.vision.getObservationsFromCamera("turret");
robot.vision.getHiveCells(VisionObservation.Alliance.RED);
```

An empty result can mean disabled vision, missing camera, no target, expired frame,
or a failed detector. Use `getCameraStatuses()` and `getErrors()` for diagnosis.

Each observation belongs to one camera and one frame. Different cameras may see the same object;
do not count the combined list as unique objects. Temporal tracking and cross-camera fusion
are future work. A turret camera's pose is camera-relative; using it as robot/field coordinates
will require calibrated mounting transforms and turret angle at frame-capture time.

## Add color candidates or an object detector

Use `.colorCandidate(new VisionCameraConfig.ColorProfile(...))` to select a color range,
candidate kind, alliance/color, and minimum contour area. Pollen is yellow; nectar is red or
blue. Color thresholds and shape validation must be tuned on actual images. A blue region
alone could be a ball, a cell, or another object; results carry `COLOR_CANDIDATE` evidence.
No confidence or range is invented from a color region.

For validated flower/hive/robot recognition, extend `VisionCameraConfig.Detector`, implement
`detect(Mat frame, long captureTimeNanos)`, and register an instance with `.detector(...)`.
Return the complete list for that frame, including an empty list when nothing is detected.
Use the supplied camera identity, role, and capture timestamp in every observation. Optional
`initialize(...)` supports loading a model/calibration. Recognition failures clear that frame
and appear in telemetry. Override `release()` to release model/native resources. Cleanup
is synchronized with processing and called once; later frames cannot use released resources.
Processor keys are `apriltags`, `color/<profile ID>`, and `object/<detector ID>`.
Use `setProcessorEnabled(cameraId, key, enabled)` to toggle processing and clear old results.

## BIOBUZZ cell identity

`BioBuzzTargets` isolates game-specific IDs from the general vision layer:

| Cell | Tag IDs |
| --- | --- |
| Red, opposite audience | 30-33 |
| Red, audience side | 34-37 |
| Blue, audience side | 38-41 |
| Blue, opposite audience | 42-45 |

The SDK's cluster pose refers to a cluster origin; an individual tag pose has a different
origin. `poseOrigin` makes this distinction explicit. Cluster `tagIds` describes its known
membership, not proof that all four tags were individually visible. Cluster coverage is not
confidence, and seeing a cell does not establish that it is upward-facing or scoring-ready.
Legacy `getAprilTagData(id)` and `getClosestAprilTag()` only return individual-tag poses;
use `getHiveCells(...)` for the clustered BIOBUZZ scoring targets.

Source: BIOBUZZ Competition Manual TU10, sections 9.6-9.9:
https://ftc-resources.firstinspires.org/ftc/game/cm-html/BIOBUZZ%20Competition%20Manual%20-%20TU10.htm

Hardware-free regression tests are in `src/test/java/.../subsystems/VisionFrameworkTest.java`.
With Gradle/Android dependencies available, run `:TeamCode:testDebugUnitTest`.
Camera calibration, native image processing, throughput, and on-robot control still require
camera images and physical testing.
