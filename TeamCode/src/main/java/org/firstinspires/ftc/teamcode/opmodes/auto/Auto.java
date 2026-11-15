package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.List;

/**
 * Vision autonomous (no path following, no localization needed):
 *
 *   FIRE : Limelight AprilTag pipeline. The robot turns until the tag is centered, sets the flywheel
 *          speed from the tag distance, and only feeds a game piece once it is aimed, in range and
 *          the flywheel is at speed.
 *   HUNT : Limelight detector pipeline. The robot drives at the biggest ball it sees while the
 *          intake runs. It spins to search if it sees nothing. Stops when the intake is full.
 *
 * Limelight setup (web UI at limelight.local:5801):
 *   pipeline 0 = AprilTag (fiducial) pipeline
 *   pipeline 1 = Neural Detector (or Color) pipeline trained on the balls
 *
 * Everything in the Tunables block is a PLACEHOLDER to tune on the real robot.
 */
@Autonomous(name = "VisionAuto", group = "Auto")
public class VisionAuto extends LinearOpMode {

    // ---- Tunables ----
    public static int TAG_PIPELINE = 0;
    public static int BALL_PIPELINE = 1;
    public static int TAG_ID = -1;                  // -1 = any tag
    public static String BALL_CLASS = "";           // "" = any detector class

    public static double AUTO_SECONDS = 30;
    public static double END_BUFFER = 1.0;          // stop doing new things this close to the end

    // aiming
    public static double AIM_KP = 0.02;             // power per degree of tag error
    public static double AIM_MAX_TURN = 0.35;
    public static double AIM_TOLERANCE_DEG = 2.0;
    public static double MIN_FIRE_DIST_M = 0.8;     // fire window (camera to tag)
    public static double MAX_FIRE_DIST_M = 3.0;
    public static double SEARCH_TURN = 0.25;

    // flywheel (ticks/sec). Tune FLY_BASE and FLY_PER_METER from test shots.
    public static String FLY_LEFT = "outL";
    public static String FLY_RIGHT = "outR";
    public static double FLY_BASE = 1100;
    public static double FLY_PER_METER = 250;
    public static double FLY_MIN = 600;
    public static double FLY_MAX = 2000;
    public static double FLY_TOLERANCE = 60;
    public static double READY_HOLD_S = 0.15;       // must be aimed + at speed this long to fire

    // feeding
    public static String FEEDER_NAME = "feeder";    // CRServo that pushes a piece into the flywheel
    public static double FEED_POWER = 1.0;
    public static double FEED_TIME_S = 0.35;
    public static double FEED_GAP_S = 0.35;
    public static int SHOTS_PER_CYCLE = 4;
    public static double FIRE_TIMEOUT_S = 6.0;

    // ball hunting
    public static double HUNT_FORWARD = 0.4;
    public static double HUNT_TURN_KP = 0.02;
    public static double HUNT_MAX_TURN = 0.4;
    public static double HUNT_TURN_ONLY_DEG = 18;   // if the ball is further off than this, rotate first
    public static double LOST_GRACE_S = 0.5;        // keep driving this long after the ball leaves view
    public static double HUNT_TIMEOUT_S = 12.0;

    // ---- Hardware ----
    private Limelight3A limelight;
    private Mecanum drivetrain;
    private final Intake intake = new Intake();
    private DcMotorEx flyL, flyR;
    private CRServo feeder;

    private int activePipeline = -1;
    private double flyTarget = 0;
    private int searchDir = 1;

    private final ElapsedTime match = new ElapsedTime();

    @Override
    public void runOpMode() {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        drivetrain = new Mecanum(hardwareMap, Constants.drivetrainConfig);
        intake.init(hardwareMap);

        flyL = hardwareMap.get(DcMotorEx.class, FLY_LEFT);
        flyR = hardwareMap.get(DcMotorEx.class, FLY_RIGHT);
        flyR.setDirection(DcMotorSimple.Direction.REVERSE); // flip if the wheels fight each other
        flyL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flyR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        feeder = hardwareMap.tryGet(CRServo.class, FEEDER_NAME);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        setPipeline(TAG_PIPELINE);

        while (opModeInInit()) {
            telemetry.addData("Feeder", feeder == null ? "MISSING (add CRServo '" + FEEDER_NAME + "')" : "ok");
            LLResult r = limelight.getLatestResult();
            telemetry.addData("Limelight", r == null ? "no data" : "valid=" + r.isValid());
            telemetry.update();
            sleep(50);
        }

        match.reset();

        // Preload first, then hunt and fire until time runs out.
        fireCycle();
        while (opModeIsActive() && timeLeft() > END_BUFFER) {
            hunt();
            if (timeLeft() <= END_BUFFER) break;
            fireCycle();
        }

        stopAll();
        limelight.stop();
    }

    // ------------------------------------------------------------------ FIRE

    private void fireCycle() {
        setPipeline(TAG_PIPELINE);
        intake.stop();
        intake.periodic(); // push the stop to the motor now; periodic() is what actually writes power
        int shots = 0;
        ElapsedTime cycle = new ElapsedTime();
        ElapsedTime ready = new ElapsedTime();
        boolean wasReady = false;

        while (opModeIsActive() && shots < SHOTS_PER_CYCLE
                && cycle.seconds() < FIRE_TIMEOUT_S && timeLeft() > END_BUFFER) {

            LLResultTypes.FiducialResult tag = currentTag();

            if (tag == null) {
                // Can't see a tag: spin slowly to find one. Keep the flywheel warm.
                drive(0, 0, searchDir * SEARCH_TURN);
                setFlywheel(Range.clip(FLY_BASE, FLY_MIN, FLY_MAX));
                wasReady = false;
                telemetry.addData("Fire", "searching for tag");
                telemetry.update();
                continue;
            }

            double tx = tag.getTargetXDegrees();
            double dist = tagDistanceM(tag);
            setFlywheel(Range.clip(FLY_BASE + FLY_PER_METER * dist, FLY_MIN, FLY_MAX));

            // Limelight tx is + when the target is to the right; turning CCW is +, so negate.
            double turn = Math.abs(tx) < AIM_TOLERANCE_DEG ? 0 : Range.clip(-tx * AIM_KP, -AIM_MAX_TURN, AIM_MAX_TURN);
            drive(0, 0, turn);

            boolean aimed = Math.abs(tx) < AIM_TOLERANCE_DEG;
            boolean inRange = dist >= MIN_FIRE_DIST_M && dist <= MAX_FIRE_DIST_M;
            boolean atSpeed = flywheelAtSpeed();
            boolean ok = aimed && inRange && atSpeed;

            if (ok && !wasReady) ready.reset();
            wasReady = ok;

            telemetry.addData("Tag tx", "%.1f deg", tx);
            telemetry.addData("Tag dist", "%.2f m", dist);
            telemetry.addData("aimed/range/speed", "%s / %s / %s", aimed, inRange, atSpeed);
            telemetry.addData("Shots", shots);
            telemetry.update();

            if (ok && ready.seconds() >= READY_HOLD_S) {
                drivetrain.stop();
                feedOnce();
                shots++;
                wasReady = false;
            }
        }

        feed(false);
        setFlywheel(0);
        intake.start(); // re-arm the intake and reset its count
    }

    private void feedOnce() {
        ElapsedTime t = new ElapsedTime();
        feed(true);
        while (opModeIsActive() && t.seconds() < FEED_TIME_S) {
            setFlywheel(flyTarget);
        }
        feed(false);
        t.reset();
        while (opModeIsActive() && t.seconds() < FEED_GAP_S) {
            setFlywheel(flyTarget); // let the flywheel recover before the next piece
        }
    }

    private void feed(boolean on) {
        if (feeder != null) feeder.setPower(on ? FEED_POWER : 0);
    }

    private LLResultTypes.FiducialResult currentTag() {
        LLResult r = latest(TAG_PIPELINE);
        if (r == null) return null;
        List<LLResultTypes.FiducialResult> list = r.getFiducialResults();
        for (LLResultTypes.FiducialResult fr : list) {
            if (TAG_ID < 0 || fr.getFiducialId() == TAG_ID) return fr;
        }
        return null;
    }

    /** Straight-line camera-to-tag distance in meters (verify against a tape measure). */
    private double tagDistanceM(LLResultTypes.FiducialResult tag) {
        Position p = tag.getTargetPoseCameraSpace().getPosition().toUnit(DistanceUnit.METER);
        return Math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
    }

    // ------------------------------------------------------------------ HUNT

    private void hunt() {
        setPipeline(BALL_PIPELINE);
        setFlywheel(0);
        intake.start();

        ElapsedTime hunt = new ElapsedTime();
        ElapsedTime sinceSeen = new ElapsedTime();
        double lastTurn = 0;

        while (opModeIsActive() && timeLeft() > END_BUFFER
                && hunt.seconds() < HUNT_TIMEOUT_S && !intake.isFull()) {

            intake.periodic();
            LLResultTypes.DetectorResult ball = bestBall();

            double fwd, turn;
            if (ball != null) {
                sinceSeen.reset();
                double tx = ball.getTargetXDegrees();
                turn = Range.clip(-tx * HUNT_TURN_KP, -HUNT_MAX_TURN, HUNT_MAX_TURN);
                lastTurn = turn;
                // Rotate first if it's far off center, otherwise drive at it while correcting.
                fwd = Math.abs(tx) > HUNT_TURN_ONLY_DEG ? 0 : HUNT_FORWARD;
                telemetry.addData("Ball", "tx=%.1f ty=%.1f area=%.2f",
                        tx, ball.getTargetYDegrees(), ball.getTargetArea());
            } else if (sinceSeen.seconds() < LOST_GRACE_S) {
                // Ball is probably in the camera's blind spot under the intake: keep rolling.
                fwd = HUNT_FORWARD;
                turn = lastTurn * 0.5;
                telemetry.addData("Ball", "lost, coasting");
            } else {
                fwd = 0;
                turn = searchDir * SEARCH_TURN;
                telemetry.addData("Ball", "searching");
            }

            drive(fwd, 0, turn);
            telemetry.addData("Intake", "%s count=%d", intake.getState(), intake.getCount());
            telemetry.update();
        }

        drivetrain.stop();
    }

    private LLResultTypes.DetectorResult bestBall() {
        LLResult r = latest(BALL_PIPELINE);
        if (r == null) return null;
        LLResultTypes.DetectorResult best = null;
        for (LLResultTypes.DetectorResult d : r.getDetectorResults()) {
            if (!BALL_CLASS.isEmpty() && !BALL_CLASS.equals(d.getClassName())) continue;
            if (best == null || d.getTargetArea() > best.getTargetArea()) best = d; // biggest = closest
        }
        return best;
    }

    // ------------------------------------------------------------------ helpers

    private void setPipeline(int p) {
        if (activePipeline != p) {
            limelight.pipelineSwitch(p);
            activePipeline = p;
            sleep(50);
        }
    }

    /** Latest result, but only if it came from the pipeline we asked for (ignores stale frames after a switch). */
    private LLResult latest(int pipeline) {
        LLResult r = limelight.getLatestResult();
        if (r == null || !r.isValid() || r.getPipelineIndex() != pipeline) return null;
        return r;
    }

    private void setFlywheel(double velocity) {
        flyTarget = velocity;
        flyL.setVelocity(velocity);
        flyR.setVelocity(velocity);
    }

    private boolean flywheelAtSpeed() {
        if (flyTarget <= 0) return false;
        double avg = (flyL.getVelocity() + flyR.getVelocity()) / 2.0;
        return Math.abs(avg - flyTarget) < FLY_TOLERANCE;
    }

    /** Robot-relative: forward, strafe LEFT, turn CCW (Pedro convention). */
    private void drive(double forward, double left, double ccw) {
        drivetrain.drive(new DrivePowers(forward, left, ccw), false);
    }

    private double timeLeft() {
        return AUTO_SECONDS - match.seconds();
    }

    private void stopAll() {
        drivetrain.stop();
        setFlywheel(0);
        feed(false);
        intake.stop();
        intake.periodic();
    }
}