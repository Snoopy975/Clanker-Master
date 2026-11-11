package org.firstinspires.ftc.teamcode.Mechanisms;

import static com.pedropathing.api.Paths.line;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import dev.nextftc.robot.Mechanism;

/**
 * NextFTC v2 drivetrain Mechanism backed by Pedro Pathing 3.
 *
 * Pedro's Mecanum drivetrain is the ONLY thing that writes to fl/bl/fr/br, so do not create
 * NextMotor objects for those motors anywhere else.
 *
 * Modes:
 *   MANUAL - field-centric driving (heading from the Pinpoint) with stick shaping, slew limiting,
 *            voltage compensation, optional heading hold, slow mode.
 *   HOLD   - Pedro holds the pose where you let go of the sticks (optional hold assist).
 *   AUTO   - Pedro drives a straight line to a target pose (goTo). Moving the sticks cancels it.
 *
 * Call init(hardwareMap) from your OpMode constructor, then startDrive(gamepad1) in start().
 */
public class Drive implements Mechanism {

    private enum Mode { MANUAL, HOLD, AUTO }

    // ---- Tunables ----
    public static double MAX_ACCEL = 4.0;           // power/sec, translation
    public static double MAX_TURN_ACCEL = 8.0;      // power/sec, rotation
    public static double STRAFE_SCALE = 1.1;
    public static boolean INSTANT_STOP = true;      // releasing the stick cuts power immediately

    public static double DEADBAND = 0.05;
    public static double CURVE_EXPONENT = 2.0;      // 1 linear, 2 squared, 3 cubed
    public static double SLOW_FACTOR = 0.4;

    public static boolean USE_VOLTAGE_COMP = true;
    public static double NOMINAL_VOLTAGE = 12.5;
    public static double VOLTAGE_READ_PERIOD = 0.5; // seconds

    public static boolean USE_HEADING_HOLD = false; // keeps heading while translating, not turning
    public static double HEADING_KP = 1.5;
    public static double HEADING_MAX_CORRECTION = 0.4;
    public static double HEADING_HOLD_DELAY = 0.25;

    public static boolean USE_HOLD_ASSIST = false;  // Pedro holds position when sticks are released
    public static double HOLD_SPEED = 8.0;          // in/s; hold engages below this speed

    /** Pose the localizer starts at. Set before init(), e.g. from your autonomous end pose. */
    public static Pose startPose = Pose.zero();

    // ---- Pedro objects ----
    private Localizer localizer;
    private Drivetrain drivetrain;
    private Follower follower;
    private VoltageSensor voltageSensor;
    private boolean ready = false;

    // ---- State ----
    private Mode mode = Mode.MANUAL;
    private Pose holdPose = Pose.zero();
    private Pose autoTarget = Pose.zero();

    private double targetX, targetY, targetRx;
    private double lastX, lastY, lastRx;
    private boolean slow = false;

    private double headingTarget = 0;
    private boolean headingLocked = false;
    private final ElapsedTime sinceTurn = new ElapsedTime();

    private double voltageScale = 1.0;
    private final ElapsedTime voltageTimer = new ElapsedTime();
    private final ElapsedTime timer = new ElapsedTime();

    /** Call once from the OpMode constructor. */
    public void init(HardwareMap map) {
        localizer = new PinpointLocalizer(map, Constants.localizerConfig);
        drivetrain = new Mecanum(map, Constants.drivetrainConfig);
        follower = new Follower(localizer, drivetrain, new Foresight(Constants.foresightConfig));

        localizer.setPose(startPose);
        localizer.update();

        if (map.voltageSensor.iterator().hasNext()) {
            voltageSensor = map.voltageSensor.iterator().next();
        }
        timer.reset();
        voltageTimer.reset();
        sinceTurn.reset();
        ready = true;
    }

    // ---- Public API ----

    /** Schedules the gamepad drive command. Call once in the OpMode's start(). */
    public void startDrive(Gamepad gamepad) {
        Command c = infinite(() -> setInputs(
                -gamepad.left_stick_y,
                gamepad.left_stick_x,
                gamepad.right_stick_x));
        c.schedule();
    }

    /** y = forward, x = strafe right, rx = turn clockwise (raw stick values). */
    public void setInputs(double y, double x, double rx) {
        targetY = shape(y);
        targetX = shape(x);
        targetRx = shape(rx);
    }

    public void setSlowMode(boolean on) {
        slow = on;
    }

    /** Makes the robot's current facing "forward" for field-centric driving. */
    public void resetHeading() {
        if (!ready) return;
        Pose p = localizer.pose();
        localizer.setPose(new Pose(p.x(), p.y(), 0));
        headingLocked = false;
    }

    /** Drive a straight line to the target pose with Pedro, then hold it. Sticks cancel. */
    public void goTo(Pose target) {
        if (!ready) return;
        localizer.update();
        autoTarget = target;
        follower.follow(line(localizer.pose(), target).constant(target.heading()));
        mode = Mode.AUTO;
    }

    public Pose getPose() {
        return ready ? localizer.pose() : Pose.zero();
    }

    // ---- Internals ----

    private double shape(double v) {
        if (Math.abs(v) < DEADBAND) return 0;
        double scaled = (Math.abs(v) - DEADBAND) / (1 - DEADBAND);
        return Math.signum(v) * Math.pow(scaled, CURVE_EXPONENT);
    }

    private double slew(double target, double last, double dt, double maxAccel) {
        if (INSTANT_STOP && Math.abs(target) < Math.abs(last)) return target;
        double step = maxAccel * dt;
        return last + Math.max(-step, Math.min(step, target - last));
    }

    private void resetSlew() {
        lastX = lastY = lastRx = 0;
    }

    @Override
    public void periodic() {
        if (!ready) return;

        double dt = timer.seconds();
        timer.reset();

        boolean sticksActive = targetX != 0 || targetY != 0 || targetRx != 0;

        if (mode == Mode.AUTO) {
            if (sticksActive) {
                mode = Mode.MANUAL;
                resetSlew();
            } else {
                follower.update();
                if (follower.atParametricEnd()) {
                    holdPose = autoTarget;
                    follower.hold(holdPose);
                    mode = Mode.HOLD;
                }
                return;
            }
        }

        if (mode == Mode.HOLD) {
            if (sticksActive) {
                mode = Mode.MANUAL;
                resetSlew();
            } else {
                follower.update();
                return;
            }
        }

        // ---- MANUAL ----
        localizer.update();
        Pose pose = localizer.pose();
        double heading = pose.heading();

        if (USE_VOLTAGE_COMP && voltageSensor != null
                && voltageTimer.seconds() > VOLTAGE_READ_PERIOD) {
            voltageTimer.reset();
            double v = voltageSensor.getVoltage();
            if (v > 5) voltageScale = Math.min(NOMINAL_VOLTAGE / v, 1.25);
        }

        double speedMult = slow ? SLOW_FACTOR : 1.0;
        double y = slew(targetY * speedMult, lastY, dt, MAX_ACCEL);
        double x = slew(targetX * speedMult, lastX, dt, MAX_ACCEL);
        double rxCmd = targetRx * speedMult;

        if (USE_HEADING_HOLD) {
            if (targetRx != 0) {
                sinceTurn.reset();
                headingLocked = false;
            } else if (!headingLocked && sinceTurn.seconds() > HEADING_HOLD_DELAY) {
                headingTarget = heading;
                headingLocked = true;
            }
            boolean translating = Math.abs(x) + Math.abs(y) > 0.05;
            if (headingLocked && translating) {
                double error = AngleUnit.normalizeRadians(headingTarget - heading);
                // heading is CCW-positive, rx is CW-positive, so negate
                rxCmd = Math.max(-HEADING_MAX_CORRECTION,
                        Math.min(HEADING_MAX_CORRECTION, -HEADING_KP * error));
            } else if (!translating) {
                headingLocked = false;
            }
        }

        double rx = slew(rxCmd, lastRx, dt, MAX_TURN_ACCEL);
        lastY = y; lastX = x; lastRx = rx;

        // Rotate stick vector into the robot frame (x = right, y = forward)
        double rotX = (x * Math.cos(-heading) - y * Math.sin(-heading)) * STRAFE_SCALE;
        double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);

        // Pedro DrivePowers: forward, strafe LEFT, turn CCW. Ours are right / clockwise, so negate.
        double forward = rotY * voltageScale;
        double left = -rotX * voltageScale;
        double ccw = -rx * voltageScale;

        double denom = Math.max(Math.abs(forward) + Math.abs(left) + Math.abs(ccw), 1);
        drivetrain.drive(new DrivePowers(forward / denom, left / denom, ccw / denom), true);

        // Hold assist: lock the position once released and nearly stopped
        if (USE_HOLD_ASSIST && !sticksActive
                && localizer.velocity().toVector2D().magnitude() < HOLD_SPEED) {
            holdPose = pose;
            follower.hold(holdPose);
            mode = Mode.HOLD;
        }
    }
}