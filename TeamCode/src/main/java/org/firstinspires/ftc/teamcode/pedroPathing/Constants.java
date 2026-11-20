package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {

    // 1. Path-Following & Physics Rules (Tuned later via AutoTune / FTC Dashboard)
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(13.0) // Estimated robot weight in kg (update once built)
            .forwardZeroPowerAcceleration(-30.0)
            .lateralZeroPowerAcceleration(-60.0)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.01, 0))
            .headingPIDFCoefficients(new PIDFCoefficients(1.0, 0, 0.05, 0))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.01, 0, 0.0005, 0.6, 0));

    // 2. Mecanum Drivetrain Motor Names & Directions
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1.0)
            .rightFrontMotorName("frontRight")
            .rightRearMotorName("backRight")
            .leftRearMotorName("backLeft")
            .leftFrontMotorName("frontLeft")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(60.0) // Forward velocity constant (calibrated during tuning)
            .yVelocity(45.0); // Strafe velocity constant (calibrated during tuning)

    // 3. Localizer Placeholder (Uses drive motor encoders until you pick Pinpoint / OTOS / Dead Wheels)
    public static DriveEncoderConstants localizerConstants = new DriveEncoderConstants()
            .leftFrontMotorName("frontLeft")
            .rightFrontMotorName("frontRight")
            .leftRearMotorName("backLeft")
            .rightRearMotorName("backRight")
            .forwardTicksToInches(0.01)
            .strafeTicksToInches(0.01)
            .turnTicksToInches(0.01)
            .robotWidth(15.0)
            .robotLength(15.0)
            .leftFrontEncoderDirection(Encoder.REVERSE)
            .leftRearEncoderDirection(Encoder.REVERSE)
            .rightFrontEncoderDirection(Encoder.FORWARD)
            .rightRearEncoderDirection(Encoder.FORWARD);

    // 4. Default Speed & Braking Constraints
    public static PathConstraints pathConstraints = new PathConstraints(
            0.995, // Path completion translation threshold
            100.0, // Max velocity
            100.0, // Max acceleration
            1.0    // Braking multiplier
    );

    // 5. Factory Method called by your Subsystems, Auto OpModes, and Tuning.java
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .driveEncoderLocalizer(localizerConstants) // TODO: Swap to .pinpointLocalizer(...) or .otosLocalizer(...) later
                .pathConstraints(pathConstraints)
                .build();
    }
}