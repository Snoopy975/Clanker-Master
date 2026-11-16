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
 * BIOBUZZ autonomous: Pedro Pathing for position, the Turret mechanism (Limelight) for aiming.
 *
 * Game facts this is built on (manual TU09):
 *   - The target is the UPWARD-FACING CELL of your own HIVE (HIVE TIP = 20 pts, LEAVE = 3, PARK = 5).
 *     The AprilTag cluster sits on the bottom of each CELL, so the tag the turret locks onto IS the
 *     target cell. Red tags are 30-37, blue 38-45; the cell that is up has its tags facing the floor.
 *   - You start touching the wall, on your own half, with 4 POLLEN preloaded. LEAVE = leave the wall.
 *     PARK = end AUTO at least partly inside your own LOADING ZONE.
 *   - During AUTO stay on your own half (G402). Pollen to collect: your GARDEN (corner strip along
 *     the wall, 4 pollen) and the retrieval slots at the bottom of the FLOWERS on your half.
 *
 * Plan: drive to the shoot spot beside the HIVE, hold, let the turret lock on the tag, fire when
 * aimed / settled / spun up; then (collect, fire) repeated; reserve the last PARK_RESERVE_S to PARK.
 *
 * Coordinates are Pedro map coordinates (inches, origin bottom-left, +x right, +y up, ~141.5 across,
 * read off the visualizer). Everything is written for the BLUE alliance; RED is the same layout
 * rotated 180 degrees about the field centre (the field is rotationally symmetric). Values marked
 * "guess" are NOT measured. Run one of the four subclasses, not this base.
 */
public abstract class BaseAuto extends NextOpMode {

    // ---- Field layout (BLUE alliance; red = rotated 180 about the centre) ----
    public static double FIELD = 141.5;                              // map extent, measured on the visualizer

    // Blue HIVE (frame x 72-95, y 45-96) has a CELL at each end. In this layout (blue; red is rotated) the
    // TOP cell is the one that starts facing up. guess: cell centres, tune with the visualizer.
    public static double[] TOP_CELL = {84, 87};
    public static double[] BOTTOM_CELL = {84, 56};

    // One shoot spot beside each cell. The auto goes to the top one first, checks the turret can see that
    // cell's tag (the up cell's tag faces the floor), and moves to the other side if it can't. guess.
    public static double[] SHOOT_TOP = {112, 92};
    public static double[] SHOOT_BOTTOM = {112, 54};
    public static double CHECK_TAG_S = 1.0;         // how long to wait for a tag before trying the other side
    public static double TIP_ASSUME_S = 1.0;        // tag gone this long after a shot: assume the hive tipped

    // Starts: touching the wall, own half (x > 71), outside the LOADING ZONE and clear of the FLOWERS.
    // x, y, heading deg. guess.
    public static double[] START_AUDIENCE = {118, 9, 90};            // bottom (audience) wall
    public static double[] START_REAR = {100, 132, 270};             // top (rear) wall, beside the GARDEN

    // PARK: partly inside the blue LOADING ZONE (right wall, y 23-46, 11 in deep). guess.
    public static double[] PARK = {126, 35, 0};
    public static double PARK_RESERVE_S = 5.0;                       // time kept free at the end to drive there

    /**
     * Pickup lines: approach x,y,deg then end x,y,deg, driven with the intake on, nose toward the wall
     * the pollen is against. GARDEN = 4 pollen in a line along the top wall from the right corner.
     * FLOWER lines drive at the bottom retrieval slot (3.5 in tall); mechanical fit not verified.
     */
    public static double[] GARDEN = {135, 120, 90, 135, 131, 90};
    public static double[] FLOWER_RIGHT = {118, 95, 0, 130, 95, 0};  // flower on the right wall (y~95)
    public static double[] FLOWER_BOTTOM = {94, 24, 270, 94, 11, 270}; // flower on the bottom wall (x~94)

    public static double[][] PICKUPS_REAR = {GARDEN, FLOWER_RIGHT, FLOWER_BOTTOM};
    public static double[][] PICKUPS_AUDIENCE = {FLOWER_BOTTOM, FLOWER_RIGHT, GARDEN};

    // ---- Timing ----
    public static double AUTO_SECONDS = 30;
    public static double END_BUFFER = 5.0;          // stop starting new things this close to the end (= time to PARK)
    public static int MAX_CYCLES = 6;               // upper bound on pickup+fire repeats

    public static double TURRET_TOL_DEG = 1.5;      // "locked on the tag"

    // ---- Ball hunting (BallCamera: webcam, yellow pollen) once the preset pickup spots are used up ----
    public static double HUNT_FORWARD = 0.4;
    public static double HUNT_TURN_KP = 0.02;       // turn power per degree of bearing
    public static double HUNT_MAX_TURN = 0.4;
    public static double HUNT_TURN_ONLY_DEG = 18;   // ball further off than this: rotate in place first
    public static double HUNT_SEARCH_TURN = 0.25;
    public static double LOST_GRACE_S = 0.5;        // keep rolling this long after the ball leaves view
    public static double HUNT_TIMEOUT_S = 8.0;
    public static double HALF_MARGIN_IN = 4;        // stay this far inside our own half (G402)

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
    private final ElapsedTime huntTimer = new ElapsedTime();
    private final ElapsedTime sinceSeen = new ElapsedTime();
    private double lastTurn = 0;
    private final ElapsedTime checkTimer = new ElapsedTime();
    private final ElapsedTime tagSeenTimer = new ElapsedTime();
    private boolean upTop = true;       // which cell we believe is facing up (top = true in the blue layout)
    private boolean lostCell = false;

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
        double[] st = audience ? START_AUDIENCE : START_REAR;
        follower.setPose(P(st[0], st[1], st[2]));
        follower.update();

        // Only lock onto our own alliance's tags (red 30-37, blue 38-45), never the opponent's hive.
        robot.turret.setTagIdRange(red ? 30 : 38, red ? 37 : 45);

        match.reset();

        List<Command> steps = new ArrayList<>();
        steps.add(fireCycle());                                   // shoot the preload first
        for (int i = 0; i < MAX_CYCLES; i++) {
            steps.add(Commands.conditional(this::timeOk,
                    Groups.sequential(pickupCycle(i), fireCycle()),
                    Commands.instant(() -> { })));
        }
        // LEAVE is automatic (we drove off the wall). PARK: end partly inside our LOADING ZONE.
        steps.add(Commands.instant(() -> { robot.flywheel.stop(); flyCmd = 0; robot.Intake.halt(); }));
        steps.add(Commands.lazy(() -> followTo(P(PARK[0], PARK[1], PARK[2])))
                .until(() -> match.seconds() >= AUTO_SECONDS - 0.2));
        steps.add(Commands.instant(this::stopAll));
        Groups.sequential(steps.toArray(new Command[0])).schedule();
    }

    /** Runs every loop: localization + telemetry (the Turret tracks the tag in its own periodic). */
    @Override
    public void periodic() {
        if (follower == null) return;
        follower.update();
        if (robot.turret.isTagVisible()) tagSeenTimer.reset();

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
        steps.add(goToUpCell());
        steps.add(Commands.conditional(this::seesUpCell, Commands.instant(() -> { }), tryOtherSide()));
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
                Commands.instant(() -> { fireOk = false; wasReady = false; lostCell = false; }),
                // hold position facing the goal (hold() is instant: the follower keeps holding afterwards) ...
                Commands.lazy(() -> {
                    Pose cur = follower.pose();
                    aimPose = rad.of(cur.x(), cur.y(), headingToGoal(cur));
                    return PedroCommands.hold(follower, aimPose);
                }),
                // ... while the turret locks on and the flywheel spins up
                Commands.waitUntil(() -> {
                    if (isReady()) { fireOk = true; return true; }
                    // Tag vanished after we started shooting: the hive probably tipped and the other cell is up.
                    if (shots > 0 && tagSeenTimer.seconds() >= TIP_ASSUME_S) { lostCell = true; return true; }
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
                        Commands.instant(() -> { })),
                // hive tipped: go to the other side where the cell that is now up is
                Commands.conditional(() -> lostCell, tryOtherSide(), Commands.instant(() -> { })));
    }

    /** Drive to the shoot spot of the cell we think is up, then give the turret a moment to see its tag. */
    private Command goToUpCell() {
        return Groups.sequential(
                Commands.lazy(() -> followTo(shootPose())).until(() -> !timeOk()),
                Commands.instant(() -> checkTimer.reset()),
                Commands.waitUntil(() -> seesUpCell() || checkTimer.seconds() >= CHECK_TAG_S));
    }

    /** Switch to the other cell's side and wait for its tag. Used when this side has no tag (not up). */
    private Command tryOtherSide() {
        return Groups.sequential(
                Commands.instant(() -> { upTop = !upTop; wasReady = false; }),
                goToUpCell());
    }

    /** The turret sees a tag that belongs to the cell we expect to be up. */
    private boolean seesUpCell() {
        return robot.turret.isTagVisible() && isTopTag(robot.turret.getTagId()) == upTop;
    }

    /** Which cell a tag belongs to, in the blue layout: top cell = blue 42-45 / red 34-37. */
    private boolean isTopTag(int id) {
        return red ? id >= 34 && id <= 37 : id >= 42 && id <= 45;
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
        double[][] lines = audience ? PICKUPS_AUDIENCE : PICKUPS_REAR;
        if (i >= lines.length) return huntCycle();                // presets used up: look for loose pollen
        double[] a = lines[i];
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

    // ------------------------------------------------------------------ HUNT

    /** Drive at the nearest yellow ball the webcam sees; spin to search; stay on our own half. */
    private Command huntCycle() {
        return Groups.sequential(
                Commands.instant(() -> {
                    robot.flywheel.stop();
                    flyCmd = 0;
                    robot.Intake.resetCount();
                    huntTimer.reset();
                    sinceSeen.reset();
                    lastTurn = 0;
                }),
                Groups.parallel(
                        robot.Intake.run().until(this::huntDone),
                        Commands.infinite(this::huntStep).until(this::huntDone)),
                Commands.instant(() -> follower.hold(follower.pose())));
    }

    private boolean huntDone() {
        return robot.Intake.isFull() || huntTimer.seconds() >= HUNT_TIMEOUT_S || !timeOk()
                || !inOwnHalf(follower.pose());
    }

    private void huntStep() {
        double fwd, turn;
        if (robot.ballCamera.hasBall()) {
            sinceSeen.reset();
            double bearing = robot.ballCamera.getBearingDeg();
            // + bearing = ball to the right; Pedro turns CCW-positive, so negate.
            turn = Range.clip(-bearing * HUNT_TURN_KP, -HUNT_MAX_TURN, HUNT_MAX_TURN);
            lastTurn = turn;
            fwd = Math.abs(bearing) > HUNT_TURN_ONLY_DEG ? 0 : HUNT_FORWARD;
            log("Ball", String.format("bearing=%.1f deg radius=%.0f px", bearing, robot.ballCamera.getRadiusPx()));
        } else if (sinceSeen.seconds() < LOST_GRACE_S) {
            fwd = HUNT_FORWARD;                                    // probably under the intake: keep rolling
            turn = lastTurn * 0.5;
            log("Ball", "lost, coasting");
        } else {
            fwd = 0;
            turn = HUNT_SEARCH_TURN;
            log("Ball", "searching");
        }
        follower.manual(fwd, 0, turn);                             // robot-centric: forward, strafe left, turn CCW
    }

    /** True while the robot is still on its own half of the field (red x < centre, blue x > centre). */
    private boolean inOwnHalf(Pose p) {
        double xBlue = red ? FIELD - p.x() : p.x();                // fold red onto the blue layout
        return xBlue > FIELD / 2 + HALF_MARGIN_IN;
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
        double[] sp = upTop ? SHOOT_TOP : SHOOT_BOTTOM;
        Pose spot = P(sp[0], sp[1], 0);
        return rad.of(spot.x(), spot.y(), headingToGoal(spot));
    }

    private double goalX() { double x = upTop ? TOP_CELL[0] : BOTTOM_CELL[0]; return red ? FIELD - x : x; }

    private double goalY() { double y = upTop ? TOP_CELL[1] : BOTTOM_CELL[1]; return red ? FIELD - y : y; }

    /** Body heading (radians) that points the robot front at the goal from the given pose. */
    private double headingToGoal(Pose from) {
        return Math.atan2(goalY() - from.y(), goalX() - from.x());
    }

    private double distToGoal(Pose from) {
        return Math.hypot(goalX() - from.x(), goalY() - from.y());
    }

    /** Pose in degrees; for red, rotated 180 degrees about the field centre. */
    private Pose P(double x, double y, double headingDeg) {
        if (red) { x = FIELD - x; y = FIELD - y; headingDeg += 180; }
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
