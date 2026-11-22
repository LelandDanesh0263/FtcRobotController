package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.*;
import org.firstinspires.ftc.teamcode.util.VisionObservation.*;
import org.firstinspires.ftc.vision.apriltag.*;
import org.junit.Test;
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import static org.junit.Assert.*;

/** Hardware-free checks for observations, camera separation, and BIOBUZZ tag semantics. */
public class VisionFrameworkTest {
    private static final long T = 1_000_000_000L;

    private static VisionObservation observation(String camera, long time) {
        return new VisionObservation.Builder(camera, CameraRole.FIELD, Kind.ROBOT, Evidence.OBJECT_DETECTOR, time).build();
    }
    private static VisionSnapshot.Frame frame(String camera, long time) {
        return new VisionSnapshot.Frame(time, Collections.singletonList(observation(camera, time)));
    }

    @Test public void startsWithNoHardwareAndNoDetections() {
        Vision vision = new Vision();
        vision.periodic();
        assertTrue(vision.getObservations().isEmpty());
        assertTrue(vision.getCameraStatuses().isEmpty());
        assertFalse(vision.isCameraInitialized());
        assertSame(VisionData.NO_TARGET, vision.getClosestAprilTag());
        vision.stop();
        vision.stop();
    }

    @Test public void disabledAndMissingCamerasDoNotOpenHardware() {
        VisionCameraConfig disabled = new VisionCameraConfig.Builder("field", "fieldCam", CameraRole.FIELD).enabled(false).build();
        VisionCameraConfig missing = new VisionCameraConfig.Builder("turret", "turretCam", CameraRole.TURRET).build();
        Vision vision = new Vision(null, disabled, missing);
        assertEquals("DISABLED", vision.getCameraStatuses().get("field"));
        assertEquals("MISSING", vision.getCameraStatuses().get("turret"));
        vision.periodic();
        assertTrue(vision.getObservations().isEmpty());
        vision.stop();
    }

    @Test public void rejectsDuplicateCameraIdentitiesBeforeOpeningHardware() {
        VisionCameraConfig first = new VisionCameraConfig.Builder("same", "first", CameraRole.FIELD).build();
        VisionCameraConfig second = new VisionCameraConfig.Builder("same", "second", CameraRole.TURRET).build();
        assertThrows(IllegalArgumentException.class, () -> new Vision(null, first, second));
    }

    @Test public void rejectsOpeningOneWebcamTwice() {
        VisionCameraConfig first = new VisionCameraConfig.Builder("field", "same", CameraRole.FIELD).build();
        VisionCameraConfig second = new VisionCameraConfig.Builder("turret", "same", CameraRole.TURRET).build();
        assertThrows(IllegalArgumentException.class, () -> new Vision(null, first, second));
    }

    @Test public void keepsCamerasAndProcessorsSeparate() {
        VisionSnapshot store = new VisionSnapshot();
        store.publish("field", "objects", frame("field", T));
        store.publish("field", "tags", frame("field", T));
        store.publish("turret", "tags", frame("turret", T));
        assertEquals(3, store.getObservations(T, .25).size());
        store.clearProcessor("field", "objects");
        assertEquals(2, store.getObservations(T, .25).size());
        store.clearCamera("turret");
        assertEquals(1, store.getObservations(T, .25).size());
    }

    @Test public void emptyFrameImmediatelyRemovesAnEarlierDetection() {
        VisionSnapshot store = new VisionSnapshot();
        store.publish("field", "objects", frame("field", T));
        store.publish("field", "objects", new VisionSnapshot.Frame(T + 1, Collections.emptyList()));
        assertTrue(store.getObservations(T + 1, .25).isEmpty());
    }

    @Test public void ignoresRepeatedAndOutOfOrderFrames() {
        VisionSnapshot store = new VisionSnapshot();
        assertTrue(store.publish("field", "objects", frame("field", T)));
        assertFalse(store.publish("field", "objects", frame("field", T)));
        assertFalse(store.publish("field", "objects", frame("field", T - 1)));
        assertEquals(1, store.getObservations(T, .25).size());
    }

    @Test public void expiresStoppedStreamsAndRejectsFutureTimestamps() {
        VisionSnapshot store = new VisionSnapshot();
        store.publish("field", "objects", frame("field", T));
        assertEquals(1, store.getObservations(T + 250_000_000L, .25).size());
        assertTrue(store.getObservations(T + 250_000_001L, .25).isEmpty());
        assertTrue(store.getObservations(T - 1, .25).isEmpty());
    }

    @Test public void validatesTimestampCameraAndAgeInputs() {
        VisionSnapshot store = new VisionSnapshot();
        assertThrows(IllegalArgumentException.class, () -> store.publish("turret", "objects", frame("field", T)));
        assertThrows(IllegalArgumentException.class, () -> new VisionSnapshot.Frame(T + 1, Collections.singletonList(observation("field", T))));
        assertThrows(IllegalArgumentException.class, () -> store.getObservations(T, Double.NaN));
        assertThrows(IllegalArgumentException.class, () -> store.getObservations(T, -1));
    }

    @Test public void snapshotsAreDefensiveAndImmutable() {
        List<VisionObservation> input = new ArrayList<>(Collections.singletonList(observation("field", T)));
        VisionSnapshot.Frame frame = new VisionSnapshot.Frame(T, input);
        input.clear();
        assertEquals(1, frame.observations.size());
        assertThrows(UnsupportedOperationException.class, () -> frame.observations.clear());
        VisionSnapshot store = new VisionSnapshot(); store.publish("field", "objects", frame);
        assertThrows(UnsupportedOperationException.class, () -> store.getObservations(T, 1).clear());
    }

    @Test public void colorCandidatesHaveNoInventedConfidenceOrDistance() {
        VisionObservation candidate = new VisionObservation.Builder("field", CameraRole.FIELD, Kind.POLLEN, Evidence.COLOR_CANDIDATE, T).build();
        assertTrue(Double.isNaN(candidate.confidence));
        assertFalse(candidate.pose.hasPose());
        assertEquals(PoseOrigin.NONE, candidate.poseOrigin);
    }

    @Test public void mapsEveryOfficialCellTagAndLeavesUnknownTagsUnknown() {
        for (BioBuzzTargets.Cell cell : BioBuzzTargets.Cell.values()) {
            for (int id : cell.tagIds()) assertSame(cell, BioBuzzTargets.forTagId(id));
            assertSame(cell, BioBuzzTargets.forSdkClusterName(cell.sdkClusterName));
        }
        assertNull(BioBuzzTargets.forTagId(29));
        assertNull(BioBuzzTargets.forTagId(46));
        assertNull(BioBuzzTargets.forSdkClusterName("unknown"));
    }

    @Test public void recognizesSdkHiveClustersWithoutPretendingCoverageIsConfidence() {
        AprilTagClusterMetadata metadata = AprilTagGameDatabase.getBioBuzzTagLibrary().lookupCluster(30);
        AprilTagClusterDetection cluster = new AprilTagClusterDetection(25, metadata, DistanceUnit.INCH, null, null, null, T);
        List<VisionObservation> observations = Vision.describeAprilTags("turret", CameraRole.TURRET, T, Collections.singletonList(cluster));
        VisionObservation result = observations.get(0);
        assertEquals(Kind.HIVE_CELL, result.kind);
        assertEquals(Alliance.RED, result.alliance);
        assertEquals("red/opposite-audience", result.targetId);
        assertEquals(Arrays.asList(30, 31, 32, 33), result.tagIds);
        assertEquals(25, result.clusterCoveragePercent);
        assertTrue(Double.isNaN(result.confidence));
        assertFalse(result.pose.hasPose());
    }

    @Test public void keepsSingleTagAndClusterPoseOriginsDistinct() {
        AprilTagPoseFtc pose = new AprilTagPoseFtc(0, 42, 0, 0, 0, 0, 42, 0, 0);
        AprilTagSingleDetection tag = new AprilTagSingleDetection(34, 0, 100, null, null, null, pose, null, null, T, DistanceUnit.INCH);
        AprilTagClusterMetadata metadata = AprilTagGameDatabase.getBioBuzzTagLibrary().lookupCluster(34);
        AprilTagClusterDetection cluster = new AprilTagClusterDetection(100, metadata, DistanceUnit.INCH, pose, null, null, T);
        List<VisionObservation> observations = Vision.describeAprilTags("turret", CameraRole.TURRET, T, Arrays.asList(tag, cluster));
        assertEquals(PoseOrigin.SINGLE_TAG, observations.get(0).poseOrigin);
        assertEquals(34, observations.get(0).pose.id);
        assertEquals(PoseOrigin.TAG_CLUSTER, observations.get(1).poseOrigin);
        assertEquals(-1, observations.get(1).pose.id);
        assertEquals(42, observations.get(1).pose.rangeInches, 0);
    }

    @Test public void unknownClustersRemainGeneric() {
        AprilTagClusterDetection cluster = new AprilTagClusterDetection(50, null, DistanceUnit.INCH, null, null, null, T);
        VisionObservation result = Vision.describeAprilTags("field", CameraRole.FIELD, T, Collections.singletonList(cluster)).get(0);
        assertEquals(Kind.APRIL_TAG_CLUSTER, result.kind);
        assertEquals(Alliance.UNKNOWN, result.alliance);
        assertTrue(result.tagIds.isEmpty());
    }

    @Test public void detectorFailuresPublishAnEmptyFrameAndExposeAnError() {
        FakeDetector detector = new FakeDetector();
        detector.processFrame(null, T);
        assertEquals(1, detector.getLatestFrame().observations.size());
        detector.fail = true;
        detector.processFrame(null, T + 1);
        assertTrue(detector.getLatestFrame().observations.isEmpty());
        assertFalse(detector.getLastError().isEmpty());
        detector.fail = false;
        detector.processFrame(null, T + 2);
        assertEquals("", detector.getLastError());
        assertEquals(1, detector.getLatestFrame().observations.size());
    }

    @Test public void rejectsDetectorsAssignedToAnotherCamera() {
        assertThrows(IllegalArgumentException.class, () -> new VisionCameraConfig.Builder("turret", "cam", CameraRole.TURRET).detector(new FakeDetector()));
    }

    @Test public void cleanupRunsOnceAndClosedDetectorsIgnoreLaterFrames() {
        FakeDetector detector = new FakeDetector();
        detector.processFrame(null, T);
        detector.close(); detector.close();
        detector.processFrame(null, T + 1);
        assertEquals(1, detector.releaseCount);
        assertNull(detector.getLatestFrame());
    }

    @Test public void invalidFrameTimestampsDoNotLeaveEarlierResultsActive() {
        FakeDetector detector = new FakeDetector();
        detector.processFrame(null, T);
        detector.processFrame(null, 0);
        assertNull(detector.getLatestFrame());
        assertFalse(detector.getLastError().isEmpty());
    }

    private static final class FakeDetector extends VisionCameraConfig.Detector {
        boolean fail;
        int releaseCount;
        FakeDetector() { super("fake", "field", CameraRole.FIELD); }
        @Override protected List<VisionObservation> detect(Mat frame, long timestamp) {
            if (fail) throw new IllegalStateException("simulated detector failure");
            return Collections.singletonList(observation(cameraId, timestamp));
        }
        @Override protected void release() { releaseCount++; }
    }
}
