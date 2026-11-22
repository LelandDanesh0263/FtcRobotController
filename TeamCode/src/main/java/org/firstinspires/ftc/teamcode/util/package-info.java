/**
 * Reusable FTC control math and utilities. Configure hardware and choose measured limits in
 * subsystems and OpModes.
 *
 * <h2>Control</h2>
 * <ul>
 *   <li>Use {@link org.firstinspires.ftc.teamcode.util.PIDFController} for P, PI, PD, or PID by
 *       setting unused gains to zero. Sample a motion profile for the next position, velocity,
 *       and acceleration. Add {@link org.firstinspires.ftc.teamcode.util.FeedforwardMath} for
 *       friction, velocity, acceleration, and either constant lift gravity or angle-dependent
 *       arm gravity. Keep feedforward and PID in the same output units.</li>
 *   <li>Use one output controller per mechanism. A flywheel may use Hub velocity control or a
 *       software loop such as PI or hysteretic bang-bang. Set the hood servo from calibrated
 *       presets using {@link org.firstinspires.ftc.teamcode.util.InterpolatingTable}.</li>
 *   <li>Wrap angular error before aligning a turret or chassis. For vision, use fresh snapshots
 *       and calibrate the camera-to-robot transform, including turret angle at frame capture.
 *       Moving Hive tags are aiming targets. Absolute field pose corrections use stationary
 *       landmarks and a time-aligned odometry estimate.</li>
 * </ul>
 *
 * <h2>OpMode tick</h2>
 * <p>Start by clearing hub caches once if using MANUAL caching. BaseRobot.update already
 * does this for its callers. Read sensors and fresh vision observations, update localization,
 * advance the existing Ivy scheduler or subsystem states, and run mechanism control with
 * the new setpoints. Pedro's follower owns drivetrain outputs during an active path. Vision
 * publishes snapshots across threads. Keep actuator commands on the OpMode thread. Use
 * {@link org.firstinspires.ftc.teamcode.util.CachedOutput#forceWrite(double)} for an immediate
 * stop. Use timing and telemetry to find I/O bottlenecks before changing the loop cadence.</p>
 *
 * <h2>Power</h2>
 * <p>Share one periodically sampled voltage across custom actuator loops. Apply voltage
 * compensation once per output. Pedro 2.1.2 exposes its drivetrain option on MecanumConstants.
 * Battery/current reads can be scheduled separately from bulk encoder reads. The FTC SDK's
 * DcMotorEx.setCurrentAlert sets the threshold reported by isOverCurrent. For software
 * derating or jam handling, read motor current and velocity and decide the response in the
 * subsystem. Use measured thresholds and response times. Shed discretionary loads before
 * essential control, and preserve any power needed to hold a lift in place. Use the existing
 * slew-rate limiter and stagger high-load events through the scheduler.</p>
 *
 * <p>Hub channel limits, motor ratings, servo voltage, and the shared main battery circuit
 * determine wiring and fuse choices. Check the current FIRST construction rules and the
 * manufacturer's wiring guide when distributing motors and adding a servo power module.</p>
 */
package org.firstinspires.ftc.teamcode.util;
