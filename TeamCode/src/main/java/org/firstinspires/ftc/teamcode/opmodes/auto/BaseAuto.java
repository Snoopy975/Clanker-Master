package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.groups.Groups;
import com.pedropathing.ivy.pedro.PedroCommands;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.ArrayList;
import java.util.List;

import dev.nextftc.robot.opmode.NextAutonomous;
import dev.nextftc.robot.opmode.NextOpMode;

/**
 * Pedro Pathing + Limelight autonomous.
 *
 *   Pedro   : knows where the robot and the goal are. Drives to the shoot spot facing the goal,
 *             drives the ball-pickup lines, and gives a fallback distance to the goal.
 *   Turret  : the Turret mechanism owns the Limelight. It tracks the AprilTag (and sweeps to search
 *             when it sees none) on its own; this auto only reads its tag distance and "aimed" flag.
 *             The tag decides WHEN to fire: only when the turret is locked on the tag, the robot is
 *             settled, and the flywheel has spun up.
 *
 * Fire, then (pickup, fire) repeated until time runs out.
 *
 * Coordinates are Pedro field coordinates (inches, 0..144, heading 0 = +x). The poses below are
 * PLACEHOLDERS: set GOAL and the START / SHOOT / PICKUPS for each side on the BLUE alliance;
 * the red autos mirror everything left/right. Run one of the four subclasses (Blue/Red x
 * Scoring/Audience side), not this base.
 */
public abstract class BaseAuto extends NextOpMode {

    // ---- Field layout (PLACEHOLDERS, written for the BLUE side) ----
    public static double FIELD = 144;

    // Guessed from the field diagram (blue on the right half, scoring tags along the top, audience
    // tags along the bottom, Pedro origin bottom-left). NOT measured: replace with real numbers.
    public static double GOAL_X = 132, GOAL_Y = 132;                 // used to face the goal + fallback distance

    // Scoring side (starts near the goals)
    public static double[] START_SCORING = {120, 128, 270};          // x, y, heading deg
    public static double[] SHOOT_SCORING = {100, 100};               // x, y (heading is computed from the goal)
    /** Each row: approach x,y,deg then end x,y,deg (drive approach -> end with the intake on). */
    public static double[][] PICKUPS_SCORING = {
            {96, 96, 0, 120, 96, 0},
            {96, 72, 0, 120, 72, 0},
            {96, 48, 0, 120, 48, 0},
    };

    // Audience side (starts on the far side from the goals)
    public static double[] START_AUDIENCE = {96, 12, 90};
    public static double[] SHOOT_AUDIENCE = {96, 48};
    public static double[][] PICKUPS_AUDIENCE = {
            {96, 36, 0, 120, 36, 0},
            {96, 60, 0, 120, 60, 0},
            {96, 84, 0, 120, 84, 0},
    };

    // ---- Timing ----
    public static double AUTO_SECONDS = 30;
    public static double END_BUFFER = 1.0;          // stop starting new things this close to the end
    public static int MAX_CYCLES = 6;               // upper bound on pickup+fire repeats

    public static double TURRET_TOL_DEG = 1.5;      // "locked on the tag"

    // ---- Firing readiness ----
    public static double ROBOT_HEAD_TOL_DEG = 5.0;  // body only has to face the goal roughly; the turret fine-aims
    public static double POS_TOL_IN = 1.5;
    public static double MIN_FIRE_DIST_IN = 30, MAX_FIRE_DIST_IN = 130;
    public static double READY_HOLD_S = 0.15;

    // flywheel (open-loop throttle 0..1 vs distance in inches, so spin-up is judged by time)
    public static double FLY_BASE = 0.45;
    public static double FLY_PER_INCH = 0.004;
    public static double FLY_MIN = 0.30;
    public static double FLY_MAX = 1.0;
    public static double FLY_DEADBAND = 0.02;
    public static double FLY_SETTLE_S = 1.2;

    // firing: the intake pushes the piece into the flywheel
    public static double FEED_TIME_S = 0.35;
    public static double FEED_GAP_S = 0.35;
    public static int SHOTS_PER_CYCLE = 4;
    public static double FIRE_TIMEOUT_S = 6.0;

    private final Robot robot;
    private Follower follower;
    private final PoseFactory rad = PoseFactory.radians();
    private final PoseFactory deg = PoseFactory.degrees();

    private final ElapsedTime match = new ElapsedTime();
    private final ElapsedTime cycleTimer = new ElapsedTime();
    private final ElapsedTime flyTimer = new ElapsedTime();
    private final ElapsedTime readyTimer = new ElapsedTime();
    private final ElapsedTime feedTimer = new ElapsedTime();

    private int shots = 0;
    private boolean fireOk = false, wasReady = false, pickupDone = false;
    private double flyCmd = 0;
    private Pose aimPose = null;

    private final boolean red;
    private final boolean audience;

    protected BaseAuto(Robot robot, boolean red, boolean audience) {
        super(robot);
        this.robot = robot;
        this.red = red;
        this.audience = audience;
    }

    @Override
    public void start() {
        follower = Constants.create(hardwareMap);
        double[] st = audience ? START_AUDIENCE : START_SCORING;
        follower.setPose(P(st[0], st[1], st[2]));
        follower.update();

        match.reset();

        List<Command> steps = new ArrayList<>();
        steps.add(fireCycle());                                   // shoot the preload first
        for (int i = 0; i < MAX_CYCLES; i++) {
            steps.add(Commands.conditional(this::timeOk,
                    Groups.sequential(pickupCycle(i), fireCycle()),
                    Commands.instant(() -> { })));
        }
        steps.add(Commands.instant(this::stopAll));
        Groups.sequential(steps.toArray(new Command[0])).schedule();
    }

    /** Runs every loop: localization + telemetry (the Turret tracks the tag in its own periodic). */
    @Override
    public void periodic() {
        if (follower == null) return;
        follower.update();

        Pose p = follower.pose();
        log("Pose", String.format("x=%.1f y=%.1f h=%.0f deg", p.x(), p.y(), Math.toDegrees(p.heading())));
        log("Turret", robot.turret.getState() + (robot.turret.isTagVisible()
                ? String.format(" tx=%.1f dist=%.1f in", robot.turret.getLastTx(), robot.turret.getTagDistanceIn())
                : " no tag"));
        log("Goal dist by pose (in)", String.format("%.1f", distToGoal(p)));
        log("Shots", shots);
        log("Intake", robot.Intake.getCount() + "/" + robot.Intake.CAPACITY);
    }

    @Override
    public void end() {
        stopAll();
    }

    // ------------------------------------------------------------------ TAG

    /** Distance for the flywheel: the tag's measured distance, or the pose-based one if the tag is lost. */
    private double shotDistIn() {
        return robot.turret.isTagVisible() ? robot.turret.getTagDistanceIn() : distToGoal(follower.pose());
    }

    // ------------------------------------------------------------------ FIRE

    private Command fireCycle() {
        List<Command> steps = new ArrayList<>();
        steps.add(robot.Intake.stop());
        steps.add(Commands.instant(() -> {
            shots = 0;
            wasReady = false;
            cycleTimer.reset();
        }));
        // drive to the shoot spot, body already facing the goal so the tag is in the turret's range
        steps.add(Commands.lazy(() -> followTo(shootPose())).until(() -> !timeOk()));
        for (int i = 0; i < SHOTS_PER_CYCLE; i++) {
            steps.add(Commands.conditional(this::canShoot, shootOne(), Commands.instant(() -> { })));
        }
        steps.add(Commands.instant(() -> {
            robot.flywheel.stop();
            flyCmd = 0;
            robot.Intake.resetCount();                            // shots also trip the beam break
        }));
        return Groups.sequential(steps.toArray(new Command[0]));
    }

    private Command shootOne() {
        return Groups.sequential(
                Commands.instant(() -> { fireOk = false; wasReady = false; }),
                // hold position facing the goal (hold() is instant: the follower keeps holding afterwards) ...
                Commands.lazy(() -> {
                    Pose cur = follower.pose();
                    aimPose = rad.of(cur.x(), cur.y(), headingToGoal(cur));
                    return PedroCommands.hold(follower, aimPose);
                }),
                // ... while the turret locks on and the flywheel spins up
                Commands.waitUntil(() -> {
                    if (isReady()) { fireOk = true; return true; }
                    return cycleExpired();
                }),
                Commands.conditional(() -> fireOk,
                        Groups.sequential(
                                Commands.instant(() -> feedTimer.reset()),
                                robot.Intake.run().until(() -> feedTimer.seconds() >= FEED_TIME_S),
                                robot.Intake.stop(),
                                Commands.instant(() -> { shots++; feedTimer.reset(); }),
                                // let the flywheel recover before the next piece
                                Commands.infinite(() -> { }).until(() -> feedTimer.seconds() >= FEED_GAP_S)),
                        Commands.instant(() -> { })));
    }

    /** Tag seen and centered, robot settled, in range, flywheel spun up, all held for READY_HOLD_S. */
    private boolean isReady() {
        double dist = shotDistIn();
        setFlywheel(FLY_BASE + FLY_PER_INCH * dist);

        if (aimPose == null) return false;
        Pose cur = follower.pose();
        double posErr = Math.hypot(cur.x() - aimPose.x(), cur.y() - aimPose.y());
        double headErr = Math.abs(wrap(cur.heading() - aimPose.heading()));

        boolean aimed = robot.turret.isAimed(TURRET_TOL_DEG);
        boolean settled = posErr < POS_TOL_IN && headErr < Math.toRadians(ROBOT_HEAD_TOL_DEG);
        boolean inRange = dist >= MIN_FIRE_DIST_IN && dist <= MAX_FIRE_DIST_IN;
        boolean spunUp = flyTimer.seconds() >= FLY_SETTLE_S;

        boolean ok = aimed && settled && inRange && spunUp;
        if (ok && !wasReady) readyTimer.reset();
        wasReady = ok;

        log("Ready", "aimed=" + aimed + " settled=" + settled + " range=" + inRange + " spun=" + spunUp);
        return ok && readyTimer.seconds() >= READY_HOLD_S;
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

    // ------------------------------------------------------------------ PICKUP

    private Command pickupCycle(int i) {
        double[][] lines = audience ? PICKUPS_AUDIENCE : PICKUPS_SCORING;
        double[] a = lines[i % lines.length];
        return Groups.sequential(
                Commands.instant(() -> {
                    robot.flywheel.stop();
                    flyCmd = 0;
                    robot.Intake.resetCount();
                    pickupDone = false;
                }),
                // intake runs the whole way; the drive finishes the line (or stops early if full)
                Groups.parallel(
                        robot.Intake.run().until(() -> pickupDone || pickupAbort()),
                        Groups.sequential(
                                Commands.lazy(() -> followTo(P(a[0], a[1], a[2]))).until(this::pickupAbort),
                                Commands.lazy(() -> followTo(P(a[3], a[4], a[5]))).until(this::pickupAbort),
                                Commands.instant(() -> pickupDone = true))));
    }

    private boolean pickupAbort() { return robot.Intake.isFull() || !timeOk(); }

    // ------------------------------------------------------------------ pose helpers

    /** Straight line from wherever the robot is now to the target pose, turning linearly. */
    private Command followTo(Pose target) {
        Pose cur = follower.pose();
        return PedroCommands.follow(follower, Paths.line(cur, target).linear(cur, target));
    }

    /** Shoot spot with the body heading pointed at the goal. */
    private Pose shootPose() {
        double[] sh = audience ? SHOOT_AUDIENCE : SHOOT_SCORING;
        Pose spot = P(sh[0], sh[1], 0);
        return rad.of(spot.x(), spot.y(), headingToGoal(spot));
    }

    private double goalX() { return red ? FIELD - GOAL_X : GOAL_X; }

    /** Body heading (radians) that points the robot front at the goal from the given pose. */
    private double headingToGoal(Pose from) {
        return Math.atan2(GOAL_Y - from.y(), goalX() - from.x());
    }

    private double distToGoal(Pose from) {
        return Math.hypot(goalX() - from.x(), GOAL_Y - from.y());
    }

    /** Pose in degrees, mirrored across the field for the red side. */
    private Pose P(double x, double y, double headingDeg) {
        if (red) { x = FIELD - x; headingDeg = 180 - headingDeg; }
        return deg.of(x, y, headingDeg);
    }

    private static double wrap(double a) {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }

    // ------------------------------------------------------------------ misc

    private boolean timeOk() { return AUTO_SECONDS - match.seconds() > END_BUFFER; }

    private void stopAll() {
        robot.flywheel.stop();
        robot.Intake.halt();
    }

    /** One place to adapt if NextOpMode's telemetry field isn't the plain FTC Telemetry. */
    private void log(String key, Object value) {
        telemetry.addData(key, value);
    }
}
