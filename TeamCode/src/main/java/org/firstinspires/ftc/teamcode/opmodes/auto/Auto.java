package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.groups.Groups;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.Robot;

import java.util.ArrayList;
import java.util.List;

import dev.nextftc.robot.opmode.NextAutonomous;
import dev.nextftc.robot.opmode.NextOpMode;


/**
 * Vision autonomous, NextFTC command style. No path following or localization.
 *
 *   FIRE : Limelight AprilTag pipeline. Turn until the tag is centered, set the flywheel from tag
 *          distance, and only run the intake into the shooter once aimed, in range and spun up.
 *   HUNT : Limelight detector pipeline. Drive at the biggest ball while the intake runs; spin to
 *          search when nothing is visible; stop when the intake is full.
 *
 * The auto is one big Ivy command: fire, then (hunt, fire) repeated until time runs out.
 *
 * Limelight setup (limelight.local:5801): pipeline 0 = AprilTag, pipeline 1 = ball detector.
 * Every number in Tunables is a PLACEHOLDER to tune on the robot.
 */
@NextAutonomous(name = "Auto")
public class Auto extends NextOpMode {

    // ---- Tunables ----
    public static int TAG_PIPELINE = 0;
    public static int BALL_PIPELINE = 1;
    public static int TAG_ID = -1;                  // -1 = any tag
    public static String BALL_CLASS = "";           // "" = any detector class

    public static double AUTO_SECONDS = 30;
    public static double END_BUFFER = 1.0;          // stop starting new things this close to the end
    public static int MAX_CYCLES = 6;               // upper bound on hunt+fire repeats (time cuts it short)

    // aiming
    public static double AIM_KP = 0.02;             // turn power per degree of tag error
    public static double AIM_MAX_TURN = 0.35;
    public static double AIM_TOLERANCE_DEG = 2.0;
    public static double MIN_FIRE_DIST_M = 0.8;     // fire window (camera to tag)
    public static double MAX_FIRE_DIST_M = 3.0;
    public static double SEARCH_TURN = 0.25;

    // flywheel (open-loop throttle 0..1, so spin-up is judged by time, not measured speed)
    public static double FLY_BASE = 0.55;
    public static double FLY_PER_METER = 0.08;
    public static double FLY_MIN = 0.30;
    public static double FLY_MAX = 1.0;
    public static double FLY_DEADBAND = 0.02;       // ignore setpoint jitter smaller than this
    public static double FLY_SETTLE_S = 1.2;        // setpoint must be stable this long before firing
    public static double READY_HOLD_S = 0.15;       // aimed + in range + settled for this long to fire

    // firing: the intake pushes the piece into the flywheel
    public static double FEED_TIME_S = 0.35;
    public static double FEED_GAP_S = 0.35;
    public static int SHOTS_PER_CYCLE = 4;
    public static double FIRE_TIMEOUT_S = 6.0;

    // ball hunting
    public static double HUNT_FORWARD = 0.4;
    public static double HUNT_TURN_KP = 0.02;
    public static double HUNT_MAX_TURN = 0.4;
    public static double HUNT_TURN_ONLY_DEG = 18;   // ball further off than this: rotate in place first
    public static double LOST_GRACE_S = 0.5;        // keep rolling this long after the ball leaves view
    public static double HUNT_TIMEOUT_S = 12.0;

    private final Robot robot;
    private Limelight3A limelight;

    private final ElapsedTime match = new ElapsedTime();
    private final ElapsedTime cycleTimer = new ElapsedTime();
    private final ElapsedTime flyTimer = new ElapsedTime();
    private final ElapsedTime readyTimer = new ElapsedTime();
    private final ElapsedTime feedTimer = new ElapsedTime();
    private final ElapsedTime huntTimer = new ElapsedTime();
    private final ElapsedTime sinceSeen = new ElapsedTime();

    private int shots = 0;
    private boolean aimed = false, inRange = false, wasReady = false, fireOk = false;
    private double flyCmd = 0;
    private double lastTurn = 0;
    private final double searchDir = 1;

    public Auto(Robot robot) {
        super(robot);
        this.robot = robot;
    }

    @Override
    public void start() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        match.reset();

        List<Command> steps = new ArrayList<>();
        steps.add(fireCycle());                                   // shoot the preload first
        for (int i = 0; i < MAX_CYCLES; i++) {
            steps.add(Commands.conditional(this::timeOk,
                    Groups.sequential(huntCycle(), fireCycle()),
                    Commands.instant(() -> { })));
        }
        steps.add(Commands.instant(this::stopAll));
        Groups.sequential(steps.toArray(new Command[0])).schedule();
    }

    @Override
    public void end() {
        stopAll();
    }

    // ------------------------------------------------------------------ FIRE

    private Command fireCycle() {
        List<Command> steps = new ArrayList<>();
        steps.add(robot.Intake.stop());
        steps.add(Commands.instant(() -> {
            setPipeline(TAG_PIPELINE);
            shots = 0;
            wasReady = false;
            cycleTimer.reset();
        }));
        for (int i = 0; i < SHOTS_PER_CYCLE; i++) {
            steps.add(Commands.conditional(this::canShoot, shootOne(), Commands.instant(() -> { })));
        }
        steps.add(Commands.instant(() -> {
            robot.flywheel.stop();
            flyCmd = 0;
            robot.drive.stop();
            robot.Intake.resetCount();                            // shots also trip the beam break
        }));
        return Groups.sequential(steps.toArray(new Command[0]));
    }

    private Command shootOne() {
        return Groups.sequential(
                Commands.instant(() -> { fireOk = false; wasReady = false; }),
                // aim and spin up until ready (or the cycle times out)
                Commands.infinite(this::aimStep).until(() -> {
                    if (wasReady && readyTimer.seconds() >= READY_HOLD_S) { fireOk = true; return true; }
                    return cycleExpired();
                }),
                Commands.conditional(() -> fireOk,
                        Groups.sequential(
                                Commands.instant(() -> { robot.drive.stop(); feedTimer.reset(); }),
                                robot.Intake.run().until(() -> feedTimer.seconds() >= FEED_TIME_S),
                                robot.Intake.stop(),
                                Commands.instant(() -> { shots++; feedTimer.reset(); }),
                                // let the flywheel recover before the next piece
                                Commands.infinite(() -> { }).until(() -> feedTimer.seconds() >= FEED_GAP_S)),
                        Commands.instant(() -> { })));
    }

    /** One loop of aiming: turn toward the tag, set the flywheel from distance, track "ready". */
    private void aimStep() {
        LLResultTypes.FiducialResult tag = currentTag();
        if (tag == null) {
            // No tag: spin slowly to find one, keep the flywheel warm.
            robot.drive.setPowers(0, 0, searchDir * SEARCH_TURN);
            setFlywheel(FLY_BASE);
            aimed = false;
            inRange = false;
            wasReady = false;
            log("Fire", "searching for tag");
            return;
        }

        double tx = tag.getTargetXDegrees();
        double dist = tagDistanceM(tag);
        setFlywheel(FLY_BASE + FLY_PER_METER * dist);

        // Limelight tx is + when the target is to the right; CCW is +, so negate.
        aimed = Math.abs(tx) < AIM_TOLERANCE_DEG;
        double turn = aimed ? 0 : Range.clip(-tx * AIM_KP, -AIM_MAX_TURN, AIM_MAX_TURN);
        robot.drive.setPowers(0, 0, turn);

        inRange = dist >= MIN_FIRE_DIST_M && dist <= MAX_FIRE_DIST_M;
        boolean ok = aimed && inRange && flyTimer.seconds() >= FLY_SETTLE_S;
        if (ok && !wasReady) readyTimer.reset();
        wasReady = ok;

        log("Tag tx / dist", String.format("%.1f deg / %.2f m", tx, dist));
        log("aimed / range / spun up", aimed + " / " + inRange + " / " + (flyTimer.seconds() >= FLY_SETTLE_S));
        log("Shots", shots);
    }

    private void setFlywheel(double throttle) {
        double t = Range.clip(throttle, FLY_MIN, FLY_MAX);
        if (Math.abs(t - flyCmd) > FLY_DEADBAND) {
            flyCmd = t;
            flyTimer.reset();                                      // new setpoint: wait for it to settle
            robot.flywheel.setThrottle(t);
        }
    }

    private boolean canShoot() { return shots < SHOTS_PER_CYCLE && !cycleExpired(); }

    private boolean cycleExpired() { return cycleTimer.seconds() >= FIRE_TIMEOUT_S || !timeOk(); }

    private LLResultTypes.FiducialResult currentTag() {
        LLResult r = latest(TAG_PIPELINE);
        if (r == null) return null;
        for (LLResultTypes.FiducialResult fr : r.getFiducialResults()) {
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

    private Command huntCycle() {
        return Groups.sequential(
                Commands.instant(() -> {
                    setPipeline(BALL_PIPELINE);
                    robot.flywheel.stop();
                    flyCmd = 0;
                    robot.Intake.resetCount();
                    huntTimer.reset();
                    sinceSeen.reset();
                    lastTurn = 0;
                }),
                // intake runs while the drive chases the ball; both end together
                Groups.parallel(
                        robot.Intake.run().until(this::huntDone),
                        Commands.infinite(this::huntStep).until(this::huntDone)),
                Commands.instant(() -> robot.drive.stop()));
    }

    private boolean huntDone() {
        return robot.Intake.isFull() || huntTimer.seconds() >= HUNT_TIMEOUT_S || !timeOk();
    }

    private void huntStep() {
        LLResultTypes.DetectorResult ball = bestBall();
        double fwd, turn;
        if (ball != null) {
            sinceSeen.reset();
            double tx = ball.getTargetXDegrees();
            turn = Range.clip(-tx * HUNT_TURN_KP, -HUNT_MAX_TURN, HUNT_MAX_TURN);
            lastTurn = turn;
            // Rotate first if it's far off center, otherwise drive at it while correcting.
            fwd = Math.abs(tx) > HUNT_TURN_ONLY_DEG ? 0 : HUNT_FORWARD;
            log("Ball", String.format("tx=%.1f ty=%.1f area=%.2f", tx, ball.getTargetYDegrees(), ball.getTargetArea()));
        } else if (sinceSeen.seconds() < LOST_GRACE_S) {
            // Probably in the camera's blind spot right in front of the intake: keep rolling.
            fwd = HUNT_FORWARD;
            turn = lastTurn * 0.5;
            log("Ball", "lost, coasting");
        } else {
            fwd = 0;
            turn = searchDir * SEARCH_TURN;
            log("Ball", "searching");
        }
        robot.drive.setPowers(fwd, 0, turn);
        log("Intake", robot.Intake.getCount() + "/" + robot.Intake.CAPACITY);
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
        limelight.pipelineSwitch(p);
    }

    /** Latest result, only if it came from the pipeline we want (ignores stale frames right after a switch). */
    private LLResult latest(int pipeline) {
        LLResult r = limelight.getLatestResult();
        if (r == null || !r.isValid() || r.getPipelineIndex() != pipeline) return null;
        return r;
    }

    private boolean timeOk() { return AUTO_SECONDS - match.seconds() > END_BUFFER; }

    private void stopAll() {
        robot.drive.stop();
        robot.flywheel.stop();
        robot.Intake.halt();
    }

    /** One place to adapt if NextOpMode's telemetry field isn't the plain FTC Telemetry. */
    private void log(String key, Object value) {
        telemetry.addData(key, value);
    }
}