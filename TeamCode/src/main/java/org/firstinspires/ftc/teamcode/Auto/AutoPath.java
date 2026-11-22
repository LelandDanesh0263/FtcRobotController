package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.LongSupplier;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.sequential;

/** Pedro 2.1.2 / Ivy core 1.1.1 drive candidate. Mechanism commands are not installed. */
@Config
@Autonomous(name = "AutoPath - Drive Candidate", group = "Autonomous")
public class AutoPath extends LinearOpMode {
    // Inches, with the localization origin at the geometric center of the robot.
    // Match these coordinates to your actual field and visualizer orientation before driving.
    public static double ROBOT_WIDTH = 17.0;
    public static double ROBOT_LENGTH = 15.0;
    public static double FIELD_SIZE = 141.5;
    public static double WALL_GAP = 1.0;

    // Planning reference from the exported flower point, NOT a measured flower surface.
    // Replace FACE_X with the measured field-facing surface of the lower retrieval opening.
    public static double FLOWER_FACE_X = 5.2199;
    public static double FLOWER_CENTER_Y = 46.7669;
    public static double FLOWER_GAP = 1.0;
    // Center-to-tip distances. Defaults assume both tips are flush with the 15-inch chassis.
    public static double FLOWER_TIP_REACH = 7.5;
    public static double INTAKE_TIP_REACH = 7.5;
    public static double LOADING_FRONT_GAP = 6.0;
    // Red loading-zone center in this assumed field orientation; confirm on your field.
    public static double LOADING_CENTER_Y = 106.75;

    // Initial test values, not physically tuned speed or timing guarantees.
    public static double TRANSIT_POWER = 0.35;
    public static double APPROACH_POWER = 0.20;
    public static double LEG_TIMEOUT_SECONDS = 8.0;
    public static double AUTO_TIMEOUT_SECONDS = 29.0;
    public static double POSITION_TOLERANCE = 0.5;
    public static double HEADING_TOLERANCE_DEGREES = 2.0;
    public static double END_SPEED = 2.0;
    public static double FLOWER_PAUSE_SECONDS = 1.0;
    public static double LOADING_PAUSE_SECONDS = 2.0;

    @Override public void runOpMode() throws InterruptedException {
        Scheduler.reset();
        Follower follower = null;
        List<DcMotor> driveMotors = new ArrayList<>();
        Controller controller = null;
        try {
            // Snapshot Dashboard values at INIT; changing geometry during a run cannot move targets.
            Settings settings = new Settings();
            Plan plan = plan(settings);
            validateWallClearance(plan, settings);
            for (String name : new String[]{"frontLeft", "frontRight", "backLeft", "backRight"}) {
                DcMotor motor = hardwareMap.get(DcMotor.class, name);
                driveMotors.add(motor);
                motor.setPower(0);
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            // BaseRobot uses MANUAL caching; this independent OpMode needs fresh encoder reads too.
            for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
                hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
            }
            follower = Constants.createFollower(hardwareMap);
            follower.setConstraints(constraints(settings));
            follower.setStartingPose(plan.start);
            final Follower driveFollower = follower;
            final String[] event = {"Mechanisms pending; this candidate only drives and pauses"};
            controller = new Controller(new Drive() {
                @Override public void follow(Leg leg) {
                    driveFollower.setMaxPower(leg.power);
                    driveFollower.followPath(leg.path, true);
                }
                @Override public Pose pose() { return driveFollower.getPose(); }
                @Override public boolean busy() { return driveFollower.isBusy(); }
                @Override public double speed() { return driveFollower.getVelocity().getMagnitude(); }
                @Override public void stop() { driveFollower.breakFollowing(); }
            }, settings, System::nanoTime, message -> event[0] = message);
            Command routine = controller.routine(plan);
            telemetry.setMsTransmissionInterval(100);
            telemetry.addLine("DRIVE CANDIDATE: field geometry and localizer still need physical validation");
            telemetry.addData("Start center", "(%.2f, %.2f), %.1f deg; rear against bottom wall",
                    plan.start.getX(), plan.start.getY(), Math.toDegrees(plan.start.getHeading()));
            telemetry.addData("Flower center stop", "(%.2f, %.2f), 180 deg", plan.flower.getX(), plan.flower.getY());
            telemetry.addData("Loading center stop", "(%.2f, %.2f), 180 deg", plan.loading.getX(), plan.loading.getY());
            telemetry.addData("Actions", event[0]);
            telemetry.update();
            waitForStart();
            if (isStopRequested()) return;
            long started = System.nanoTime();
            Scheduler.schedule(routine);
            while (opModeIsActive() && Scheduler.isScheduled(routine) && controller.fault.isEmpty()) {
                if (secondsSince(started, System.nanoTime()) >= settings.autoTimeout) {
                    controller.abort("Autonomous time limit reached");
                    break;
                }
                follower.update();
                Scheduler.execute();
                Pose pose = follower.getPose();
                telemetry.addData("Leg", "%d/%d %s", controller.activeLeg + 1, plan.legs.size(), controller.legName);
                telemetry.addData("Pose", "x %.2f, y %.2f, heading %.1f deg", pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
                telemetry.addData("Elapsed seconds", "%.1f", secondsSince(started, System.nanoTime()));
                telemetry.addData("Action", event[0]);
                telemetry.addData("Fault", controller.fault);
                telemetry.update();
                idle();
            }
            telemetry.addData("Result", !controller.fault.isEmpty() ? controller.fault
                    : controller.finished ? "Drive route complete; no collection/scoring performed" : "Stopped");
            telemetry.update();
        } finally {
            // Cancellation and follower cleanup cannot prevent the independent motor-zero attempts.
            try {
                Scheduler.reset();
            } finally {
                try {
                    if (controller != null) controller.drive.stop();
                    else if (follower != null) follower.breakFollowing();
                } finally {
                    for (DcMotor motor : driveMotors) {
                        try { motor.setPower(0); }
                        catch (RuntimeException e) { RobotLog.ee("AutoPath", e, "Could not stop a drive motor"); }
                    }
                }
            }
        }
    }

    static final class Settings {
        final double width = positive(ROBOT_WIDTH, "Robot width");
        final double length = positive(ROBOT_LENGTH, "Robot length");
        final double field = positive(FIELD_SIZE, "Field size");
        final double wallGap = positive(WALL_GAP, "Wall gap");
        final double flowerFace = positive(FLOWER_FACE_X, "Flower face X");
        final double flowerY = positive(FLOWER_CENTER_Y, "Flower Y");
        final double flowerGap = positive(FLOWER_GAP, "Flower gap");
        final double flowerReach = positive(FLOWER_TIP_REACH, "Flower mechanism reach");
        final double intakeReach = positive(INTAKE_TIP_REACH, "Intake reach");
        final double loadingGap = positive(LOADING_FRONT_GAP, "Loading front gap");
        final double loadingY = positive(LOADING_CENTER_Y, "Loading Y");
        final double transitPower = power(TRANSIT_POWER);
        final double approachPower = power(APPROACH_POWER);
        final double legTimeout = positive(LEG_TIMEOUT_SECONDS, "Leg timeout");
        final double autoTimeout = positive(AUTO_TIMEOUT_SECONDS, "Auto timeout");
        final double positionTolerance = positive(POSITION_TOLERANCE, "Position tolerance");
        final double headingTolerance = Math.toRadians(positive(HEADING_TOLERANCE_DEGREES, "Heading tolerance"));
        final double endSpeed = positive(END_SPEED, "End speed");
        final double flowerPause = nonnegative(FLOWER_PAUSE_SECONDS, "Flower pause");
        final double loadingPause = nonnegative(LOADING_PAUSE_SECONDS, "Loading pause");
        Settings() {
            if (flowerReach < length / 2 || intakeReach < length / 2) {
                throw new IllegalArgumentException("Front tip reaches are measured from robot center and must reach at least the chassis front");
            }
        }
        // Include configured front protrusions in the conservative outline during travel.
        double frontReach() { return Math.max(length / 2, Math.max(flowerReach, intakeReach)); }
        double turnRadius() { return Math.hypot(Math.max(length / 2, frontReach()), width / 2); }
    }

    static final class Leg {
        final String name;
        final Pose end;
        final Path path;
        final double power, pause;
        final String arrival;
        Leg(String name, Pose start, Pose end, double power, double pause, String arrival, Settings s) {
            this.name = name; this.end = end; this.power = power; this.pause = pause; this.arrival = arrival;
            path = new Path(new BezierLine(start, end), constraints(s));
            path.setLinearHeadingInterpolation(start.getHeading(), end.getHeading());
        }
    }

    static final class Plan {
        final Pose start, flower, loading;
        final List<Leg> legs;
        Plan(Pose start, Pose flower, Pose loading, List<Leg> legs) {
            this.start = start; this.flower = flower; this.loading = loading;
            this.legs = Collections.unmodifiableList(new ArrayList<>(legs));
        }
    }

    static Plan plan(Settings s) {
        Pose start = degrees(56, s.length / 2, 90);
        Pose depart = degrees(56, Math.max(14, s.turnRadius() + s.wallGap), 90);
        Pose waypoint1 = degrees(42.6964, 18.8631, 135);
        Pose flower = degrees(s.flowerFace + s.flowerReach + s.flowerGap, s.flowerY, 180);
        Pose loading = degrees(s.intakeReach + s.loadingGap, s.loadingY, 180);
        // Retreat straight from each target before rotating or travelling along the wall.
        double corridorX = Math.max(24, Math.max(flower.getX(), loading.getX()) + 6);
        corridorX = Math.max(corridorX, s.flowerFace + s.turnRadius() + s.wallGap);
        Pose flowerStage = degrees(corridorX, s.flowerY, 180);
        Pose loadingStage = degrees(corridorX, s.loadingY, 180);
        Pose returnStage = degrees(corridorX, 24, 180);
        Pose finish = degrees(56.3562, Math.max(12.5, s.turnRadius() + s.wallGap), 90);
        List<Leg> legs = new ArrayList<>();
        legs.add(new Leg("Leave starting wall", start, depart, s.approachPower, 0, "", s));
        legs.add(new Leg("Exported waypoint 1", depart, waypoint1, s.transitPower, 0,
                "Waypoint 1 reached; future scoring command belongs here", s));
        legs.add(new Leg("Align with flower", waypoint1, flowerStage, s.transitPower, 0, "", s));
        legs.add(new Leg("Approach flower", flowerStage, flower, s.approachPower, s.flowerPause,
                "Flower pause only; lower-opening collection mechanism pending", s));
        legs.add(new Leg("Back away from flower", flower, flowerStage, s.approachPower, 0, "", s));
        legs.add(new Leg("Travel to loading zone", flowerStage, loadingStage, s.transitPower, 0, "", s));
        legs.add(new Leg("Approach loading zone", loadingStage, loading, s.approachPower, s.loadingPause,
                "Loading pause only; intake and ball detection pending", s));
        legs.add(new Leg("Back away from loading zone", loading, loadingStage, s.approachPower, 0, "", s));
        legs.add(new Leg("Return through left corridor", loadingStage, returnStage, s.transitPower, 0, "", s));
        legs.add(new Leg("Return near exported finish", returnStage, finish, s.transitPower, 0, "", s));
        return new Plan(start, flower, loading, legs);
    }

    static PathConstraints constraints(Settings s) {
        // Pedro 2.1.2 arguments: t-value, end timeout (ms), braking strength, braking start.
        PathConstraints c = new PathConstraints(0.995, 1000.0, 1.0, 1.0);
        c.setVelocityConstraint(s.endSpeed);
        c.setTranslationalConstraint(s.positionTolerance);
        c.setHeadingConstraint(s.headingTolerance);
        return c;
    }

    /** Nominal sampled wall-envelope check, not a field-object or localization-error guarantee. */
    static void validateWallClearance(Plan plan, Settings s) {
        for (int legIndex = 0; legIndex < plan.legs.size(); legIndex++) {
            Leg leg = plan.legs.get(legIndex);
            // The first straight departure starts touching the wall as required for MATCH setup.
            double minimum = legIndex == 0 ? 0 : s.wallGap;
            for (int sample = 0; sample <= 400; sample++) {
                double t = sample / 400.0;
                Pose point = leg.path.getPoint(t);
                double heading = leg.path.getHeadingGoal(t);
                double gap = boundaryClearance(point.getX(), point.getY(), heading, s);
                if (gap + 1e-6 < minimum) {
                    throw new IllegalArgumentException(leg.name + ": insufficient wall clearance; revise geometry");
                }
            }
        }
    }

    static double boundaryClearance(double x, double y, double heading, Settings s) {
        double gap = Double.POSITIVE_INFINITY;
        for (double localX : new double[]{-s.length / 2, s.frontReach()}) {
            for (double localY : new double[]{-s.width / 2, s.width / 2}) {
                double px = x + localX * Math.cos(heading) - localY * Math.sin(heading);
                double py = y + localX * Math.sin(heading) + localY * Math.cos(heading);
                gap = Math.min(gap, Math.min(Math.min(px, py), Math.min(s.field - px, s.field - py)));
            }
        }
        return gap;
    }

    interface Drive {
        void follow(Leg leg);
        Pose pose();
        boolean busy();
        double speed();
        void stop();
    }

    /** Small Pedro 2 adapter; uses Ivy core, not the installed Pedro 3 follow adapter. */
    static final class Controller {
        final Drive drive;
        final Settings settings;
        final LongSupplier clock;
        final Consumer<String> arrival;
        String fault = "", legName = "Waiting";
        int activeLeg = -1;
        boolean finished;
        private long legStarted, pauseStarted;

        Controller(Drive drive, Settings settings, LongSupplier clock, Consumer<String> arrival) {
            this.drive = drive; this.settings = settings; this.clock = clock; this.arrival = arrival;
        }
        Command routine(Plan plan) {
            List<Command> commands = new ArrayList<>();
            for (int i = 0; i < plan.legs.size(); i++) {
                final int index = i;
                final Leg leg = plan.legs.get(i);
                commands.add(new CommandBuilder().requiring(drive).setStart(() -> {
                    if (!fault.isEmpty()) return;
                    activeLeg = index; legName = leg.name; legStarted = clock.getAsLong();
                    drive.follow(leg);
                }).setDone(() -> legDone(leg)).setEnd(condition -> {
                    if (condition != com.pedropathing.ivy.behaviors.EndCondition.NATURALLY) drive.stop();
                }));
                if (!leg.arrival.isEmpty()) {
                    commands.add(instant(() -> { if (fault.isEmpty()) arrival.accept(leg.arrival); }));
                }
                if (leg.pause > 0) {
                    commands.add(new CommandBuilder().requiring(drive)
                            .setStart(() -> pauseStarted = clock.getAsLong())
                            .setDone(() -> !fault.isEmpty() || secondsSince(pauseStarted, clock.getAsLong()) >= leg.pause));
                }
            }
            commands.add(instant(() -> finished = fault.isEmpty()));
            return sequential(commands.toArray(new Command[0]));
        }
        private boolean legDone(Leg leg) {
            if (!fault.isEmpty()) return true;
            Pose p = drive.pose();
            double speed = drive.speed();
            if (p == null || !Double.isFinite(p.getX()) || !Double.isFinite(p.getY())
                    || !Double.isFinite(p.getHeading()) || !Double.isFinite(speed)) {
                abort("Invalid localization or velocity"); return true;
            }
            double distance = Math.hypot(p.getX() - leg.end.getX(), p.getY() - leg.end.getY());
            double headingError = Math.abs(angleDifference(p.getHeading(), leg.end.getHeading()));
            if (!drive.busy() && distance <= settings.positionTolerance
                    && headingError <= settings.headingTolerance && Math.abs(speed) <= settings.endSpeed) return true;
            if (secondsSince(legStarted, clock.getAsLong()) >= settings.legTimeout) {
                abort("Timed out: " + leg.name); return true;
            }
            return false;
        }
        void abort(String message) {
            if (!fault.isEmpty()) return;
            fault = message; drive.stop();
        }
    }

    static Pose degrees(double x, double y, double heading) { return new Pose(x, y, Math.toRadians(heading)); }
    static double angleDifference(double first, double second) { return Math.atan2(Math.sin(first - second), Math.cos(first - second)); }
    static double secondsSince(long then, long now) { return (now - then) / 1e9; }
    private static double positive(double value, String name) {
        if (!Double.isFinite(value) || value <= 0) throw new IllegalArgumentException(name + " must be finite and positive");
        return value;
    }
    private static double nonnegative(double value, String name) {
        if (!Double.isFinite(value) || value < 0) throw new IllegalArgumentException(name + " must be finite and nonnegative");
        return value;
    }
    private static double power(double value) {
        positive(value, "Power");
        if (value > 1) throw new IllegalArgumentException("Power must be <= 1");
        return value;
    }
}
