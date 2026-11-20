package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.VisionData;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

import java.util.Collections;
import java.util.List;

/** Shared vision for OpModes using an optional FTC VisionPortal webcam. */
@Config
public class Vision implements Subsystem {
    // Applied when an OpMode initializes.
    public static String WEBCAM_NAME = "Webcam 1";
    public static int STREAM_WIDTH = 640;
    public static int STREAM_HEIGHT = 480;
    public static boolean ENABLE_APRIL_TAGS = true;
    public static boolean ENABLE_COLOR_BLOBS = false;

    private VisionPortal portal;
    private AprilTagProcessor aprilTags;
    private ColorBlobLocatorProcessor colorBlobs;

    public Vision(HardwareMap hardwareMap) {
        this(hardwareMap, WEBCAM_NAME);
    }

    public Vision(HardwareMap hardwareMap, String webcamName) {
        this(hardwareMap, webcamName, ColorRange.BLUE);
    }

    public Vision(HardwareMap hardwareMap, String webcamName, ColorRange blobColor) {
        try {
            WebcamName webcam = hardwareMap.tryGet(WebcamName.class, webcamName);
            if (webcam == null) {
                return;
            }

            aprilTags = new AprilTagProcessor.Builder()
                    .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                    .setDrawTagID(true)
                    .setDrawTagOutline(true)
                    .build();
            colorBlobs = new ColorBlobLocatorProcessor.Builder()
                    .setTargetColorRange(blobColor)
                    .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                    .setRoi(ImageRegion.entireFrame())
                    .setDrawContours(true)
                    .setBlurSize(5)
                    .build();

            portal = new VisionPortal.Builder()
                    .setCamera(webcam)
                    .setCameraResolution(new Size(STREAM_WIDTH, STREAM_HEIGHT))
                    .addProcessors(aprilTags, colorBlobs)
                    .build();
            setAprilTagProcessorEnabled(ENABLE_APRIL_TAGS);
            setColorBlobProcessorEnabled(ENABLE_COLOR_BLOBS);
        } catch (RuntimeException e) {
            RobotLog.ee("Vision", e, "Could not initialize webcam '%s'", webcamName);
            stop();
        }
    }

    public boolean isCameraInitialized() {
        return portal != null && portal.getCameraState() == VisionPortal.CameraState.STREAMING;
    }

    public List<AprilTagDetection> getAprilTagDetections() {
        return isProcessing(aprilTags) ? aprilTags.getDetections() : Collections.emptyList();
    }

    public boolean isTagVisible(int tagId) {
        return getAprilTagData(tagId).targetFound;
    }

    /** Finds only a single tag with this exact ID, including tags without a pose. */
    public VisionData getAprilTagData(int tagId) {
        for (AprilTagDetection detection : getAprilTagDetections()) {
            if (detection instanceof AprilTagSingleDetection) {
                AprilTagSingleDetection tag = (AprilTagSingleDetection) detection;
                if (tag.id == tagId) {
                    return snapshot(tag);
                }
            }
        }
        return VisionData.NO_TARGET;
    }

    /** Finds the nearest individual tag with a valid range. */
    public VisionData getClosestAprilTag() {
        AprilTagSingleDetection closest = null;
        double closestRange = Double.POSITIVE_INFINITY;
        for (AprilTagDetection detection : getAprilTagDetections()) {
            if (detection instanceof AprilTagSingleDetection && detection.ftcPose != null) {
                double range = detection.ftcPose.range;
                if (range >= 0 && range < closestRange) {
                    closest = (AprilTagSingleDetection) detection;
                    closestRange = range;
                }
            }
        }
        return closest == null ? VisionData.NO_TARGET : snapshot(closest);
    }

    private static VisionData snapshot(AprilTagSingleDetection tag) {
        String name = tag.metadata == null ? "Unknown" : tag.metadata.name;
        AprilTagPoseFtc pose = tag.ftcPose;
        if (pose == null) {
            return new VisionData(true, tag.id, name, tag.frameAcquisitionNanoTime,
                    Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN,
                    Double.NaN, Double.NaN, Double.NaN);
        }
        return new VisionData(true, tag.id, name, tag.frameAcquisitionNanoTime,
                pose.range, pose.bearing, pose.elevation, pose.yaw, pose.pitch, pose.roll,
                pose.x, pose.y, pose.z);
    }

    public List<ColorBlobLocatorProcessor.Blob> getDetectedBlobs() {
        return isProcessing(colorBlobs) ? colorBlobs.getBlobs() : Collections.emptyList();
    }

    public boolean hasDetectedBlobs() {
        return !getDetectedBlobs().isEmpty();
    }

    public VisionPortal getVisionPortal() {
        return portal;
    }

    public AprilTagProcessor getAprilTagProcessor() {
        return aprilTags;
    }

    public ColorBlobLocatorProcessor getColorBlobProcessor() {
        return colorBlobs;
    }

    private boolean isProcessing(VisionProcessor processor) {
        return isCameraInitialized() && portal.getProcessorEnabled(processor);
    }

    public void setAprilTagProcessorEnabled(boolean enabled) {
        if (portal != null) {
            portal.setProcessorEnabled(aprilTags, enabled);
        }
    }

    public void setColorBlobProcessorEnabled(boolean enabled) {
        if (portal != null) {
            portal.setProcessorEnabled(colorBlobs, enabled);
        }
    }

    @Override
    public void stop() {
        VisionPortal activePortal = portal;
        portal = null;
        aprilTags = null;
        colorBlobs = null;
        if (activePortal != null) {
            try {
                activePortal.close();
            } catch (RuntimeException e) {
                RobotLog.ee("Vision", e, "Could not close camera");
            }
        }
    }
}
