package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.paths.PathConstraints;

import org.firstinspires.ftc.teamcode.Auto.AutoPath;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import static org.junit.Assert.*;

/** Real Pedro geometry and Ivy scheduling, with simulated drive feedback and an explicit clock. */
public class AutoPathTest {
    private long now;

    @Before public void setUp() {
        Scheduler.reset();
        now = 1_000_000_000L;
        AutoPath.ROBOT_WIDTH = 17; AutoPath.ROBOT_LENGTH = 15; AutoPath.FIELD_SIZE = 141.5;
        AutoPath.WALL_GAP = 1; AutoPath.FLOWER_FACE_X = 5.2199; AutoPath.FLOWER_CENTER_Y = 46.7669;
        AutoPath.FLOWER_GAP = 1; AutoPath.FLOWER_TIP_REACH = 7.5; AutoPath.INTAKE_TIP_REACH = 7.5;
        AutoPath.LOADING_FRONT_GAP = 6; AutoPath.LOADING_CENTER_Y = 106.75;
        AutoPath.TRANSIT_POWER = .35; AutoPath.APPROACH_POWER = .2;
        AutoPath.LEG_TIMEOUT_SECONDS = 8; AutoPath.AUTO_TIMEOUT_SECONDS = 29;
        AutoPath.POSITION_TOLERANCE = .5; AutoPath.HEADING_TOLERANCE_DEGREES = 2; AutoPath.END_SPEED = 2;
        AutoPath.FLOWER_PAUSE_SECONDS = 0; AutoPath.LOADING_PAUSE_SECONDS = 0;
    }
    @After public void tearDown() {
        Scheduler.reset();
        AutoPath.FLOWER_PAUSE_SECONDS = 1; AutoPath.LOADING_PAUSE_SECONDS = 2;
    }

    @Test public void routeStartsAtWallAndItsDepartureHasConstantHeading() {
        AutoPath.Settings s = new AutoPath.Settings();
        AutoPath.Plan plan = AutoPath.plan(s);
        assertEquals(10, plan.legs.size());
        assertEquals(0, AutoPath.boundaryClearance(plan.start.getX(), plan.start.getY(), plan.start.getHeading(), s), 1e-9);
        for (int i = 0; i <= 100; i++) {
            assertEquals(Math.PI / 2, plan.legs.get(0).path.getHeadingGoal(i / 100.0), 1e-9);
        }
        AutoPath.validateWallClearance(plan, s);
    }

    @Test public void originalNearWallPointsIntersectTheRobotOutline() {
        AutoPath.Settings s = new AutoPath.Settings();
        assertTrue(AutoPath.boundaryClearance(5.2199, 46.7669, Math.PI, s) < 0);
        assertTrue(AutoPath.boundaryClearance(4.8767, 4.2428, 3 * Math.PI / 2, s) < 0);
        assertTrue(AutoPath.boundaryClearance(56.3562, 4.338, Math.PI / 2, s) < 0);
    }

    @Test public void successivePathsHaveContinuousPositionsAndHeadings() {
        AutoPath.Plan plan = AutoPath.plan(new AutoPath.Settings());
        for (int i = 1; i < plan.legs.size(); i++) {
            AutoPath.Leg previous = plan.legs.get(i - 1), next = plan.legs.get(i);
            Pose start = next.path.getPoint(0);
            assertEquals(previous.end.getX(), start.getX(), 1e-9);
            assertEquals(previous.end.getY(), start.getY(), 1e-9);
            assertEquals(0, AutoPath.angleDifference(previous.end.getHeading(), next.path.getHeadingGoal(0)), 1e-9);
        }
    }

    @Test public void flowerAndLoadingGapsUseTheMechanismTip() {
        AutoPath.Settings s = new AutoPath.Settings();
        AutoPath.Plan plan = AutoPath.plan(s);
        assertEquals(1, plan.flower.getX() - s.flowerReach - s.flowerFace, 1e-9);
        assertEquals(6, plan.loading.getX() - s.intakeReach, 1e-9);
        assertEquals(Math.PI, plan.flower.getHeading(), 1e-9);
        assertEquals(Math.PI, plan.loading.getHeading(), 1e-9);
        assertEquals(s.flowerY, plan.legs.get(4).end.getY(), 1e-9);
        assertEquals(Math.PI, plan.legs.get(4).end.getHeading(), 1e-9);
        assertTrue(plan.legs.get(4).end.getX() > plan.flower.getX());
    }

    @Test public void extendingATipMovesItsStopBackWithoutChangingTheGap() {
        AutoPath.FLOWER_TIP_REACH = 10;
        AutoPath.INTAKE_TIP_REACH = 9;
        AutoPath.Settings s = new AutoPath.Settings();
        AutoPath.Plan plan = AutoPath.plan(s);
        assertEquals(1, plan.flower.getX() - 10 - s.flowerFace, 1e-9);
        assertEquals(6, plan.loading.getX() - 9, 1e-9);
        AutoPath.validateWallClearance(plan, s);
    }

    @Test public void perPathConstraintsUsePedro2ArgumentSemantics() {
        AutoPath.Settings s = new AutoPath.Settings();
        PathConstraints c = AutoPath.constraints(s);
        assertEquals(1, c.getBrakingStrength(), 0);
        assertEquals(1000, c.getTimeoutConstraint(), 0);
        assertEquals(s.positionTolerance, c.getTranslationalConstraint(), 0);
        assertEquals(s.headingTolerance, c.getHeadingConstraint(), 0);
        assertEquals(s.endSpeed, c.getVelocityConstraint(), 0);
    }

    @Test public void schedulerCompletesEveryLegInOrderWithSettledFeedback() {
        AutoPath.Settings s = new AutoPath.Settings();
        AutoPath.Plan plan = AutoPath.plan(s);
        FakeDrive drive = new FakeDrive();
        List<String> arrivals = new ArrayList<>();
        AutoPath.Controller controller = controller(drive, s, arrivals);
        Command routine = controller.routine(plan);
        Scheduler.schedule(routine);
        tick(100);
        assertTrue(controller.finished);
        assertEquals("", controller.fault);
        assertFalse(Scheduler.isScheduled(routine));
        assertEquals(plan.legs.size(), drive.followed.size());
        for (int i = 0; i < plan.legs.size(); i++) assertSame(plan.legs.get(i), drive.followed.get(i));
        assertEquals(3, arrivals.size());
    }

    @Test public void aFollowerEndingFarFromTargetCannotSkipAhead() {
        AutoPath.Settings s = new AutoPath.Settings();
        FakeDrive drive = new FakeDrive(); drive.snapToEnd = false;
        AutoPath.Controller controller = controller(drive, s, new ArrayList<>());
        Scheduler.schedule(controller.routine(AutoPath.plan(s)));
        tick(10);
        assertEquals(1, drive.followed.size());
        assertFalse(controller.finished);
        now += 8_000_000_000L;
        tick(100);
        assertTrue(controller.fault.startsWith("Timed out:"));
        assertEquals(1, drive.followed.size());
        assertTrue(drive.stopCount > 0);
        assertFalse(controller.finished);
    }

    @Test public void busyHeadingAndVelocityMustAllSettleBeforeAdvancing() {
        AutoPath.Settings s = new AutoPath.Settings();
        FakeDrive drive = new FakeDrive(); drive.busy = true;
        AutoPath.Controller controller = controller(drive, s, new ArrayList<>());
        Scheduler.schedule(controller.routine(AutoPath.plan(s)));
        tick(5); assertEquals(1, drive.followed.size());
        Pose end = drive.followed.get(0).end;
        drive.busy = false; drive.position = end.withHeading(end.getHeading() + Math.toRadians(10));
        tick(5); assertEquals(1, drive.followed.size());
        drive.position = end; drive.velocity = 3;
        tick(5); assertEquals(1, drive.followed.size());
        drive.velocity = 0;
        tick(5); assertTrue(drive.followed.size() > 1);
    }

    @Test public void invalidLocalizationStopsBeforeStartingAnotherLeg() {
        AutoPath.Settings s = new AutoPath.Settings();
        FakeDrive drive = new FakeDrive(); drive.snapToEnd = false;
        drive.position = new Pose(Double.NaN, 0, 0);
        List<String> arrivals = new ArrayList<>();
        AutoPath.Controller controller = controller(drive, s, arrivals);
        Scheduler.schedule(controller.routine(AutoPath.plan(s)));
        tick(100);
        assertEquals("Invalid localization or velocity", controller.fault);
        assertEquals(1, drive.followed.size());
        assertTrue(arrivals.isEmpty());
        assertTrue(drive.stopCount > 0);
    }

    @Test public void pauseIsNonblockingAndUsesTheSuppliedClock() {
        AutoPath.Settings s = new AutoPath.Settings();
        Pose start = new Pose(20, 20, 0), end = new Pose(30, 20, 0);
        AutoPath.Leg leg = new AutoPath.Leg("Pause test", start, end, .2, 1, "At target", s);
        AutoPath.Plan plan = new AutoPath.Plan(start, end, end, Collections.singletonList(leg));
        FakeDrive drive = new FakeDrive();
        AutoPath.Controller controller = controller(drive, s, new ArrayList<>());
        Scheduler.schedule(controller.routine(plan));
        tick(20); assertFalse(controller.finished);
        now += 999_999_999L;
        tick(20); assertFalse(controller.finished);
        now += 1;
        tick(20); assertTrue(controller.finished);
    }

    @Test public void cancellationStopsTheActiveDriveCommand() {
        AutoPath.Settings s = new AutoPath.Settings();
        FakeDrive drive = new FakeDrive(); drive.busy = true;
        AutoPath.Controller controller = controller(drive, s, new ArrayList<>());
        Command routine = controller.routine(AutoPath.plan(s));
        Scheduler.schedule(routine); tick(2);
        Scheduler.cancel(routine);
        assertTrue(drive.stopCount > 0);
        assertFalse(Scheduler.isScheduled(routine));
        assertFalse(controller.finished);
    }

    @Test public void unsafeSettingsAreRejectedBeforeHardwareInitialization() {
        AutoPath.TRANSIT_POWER = Double.NaN;
        assertThrows(IllegalArgumentException.class, AutoPath.Settings::new);
        AutoPath.TRANSIT_POWER = 1.1;
        assertThrows(IllegalArgumentException.class, AutoPath.Settings::new);
        AutoPath.TRANSIT_POWER = .35;
        AutoPath.FLOWER_TIP_REACH = 7;
        assertThrows(IllegalArgumentException.class, AutoPath.Settings::new);
        AutoPath.FLOWER_TIP_REACH = 7.5;
        AutoPath.LOADING_CENTER_Y = 150;
        AutoPath.Settings s = new AutoPath.Settings();
        assertThrows(IllegalArgumentException.class, () -> AutoPath.validateWallClearance(AutoPath.plan(s), s));
    }

    private AutoPath.Controller controller(FakeDrive drive, AutoPath.Settings s, List<String> arrivals) {
        return new AutoPath.Controller(drive, s, () -> now, arrivals::add);
    }
    private static void tick(int count) { for (int i = 0; i < count; i++) Scheduler.execute(); }

    private static final class FakeDrive implements AutoPath.Drive {
        final List<AutoPath.Leg> followed = new ArrayList<>();
        Pose position = new Pose();
        boolean snapToEnd = true, busy;
        double velocity;
        int stopCount;
        @Override public void follow(AutoPath.Leg leg) { followed.add(leg); if (snapToEnd) position = leg.end; }
        @Override public Pose pose() { return position; }
        @Override public boolean busy() { return busy; }
        @Override public double speed() { return velocity; }
        @Override public void stop() { stopCount++; busy = false; }
    }
}
