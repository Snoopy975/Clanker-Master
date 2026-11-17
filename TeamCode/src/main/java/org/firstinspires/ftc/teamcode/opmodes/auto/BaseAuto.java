package org.firstinspires.ftc.teamcode.opmodes.auto;

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
 * Plan: drive to the shoot spot in front of our own HIVE, hold, let the turret lock on the tag, fire
 * when aimed / settled / spun up (no tag on that cell: run to the other end). Then GARDEN, pit FLOWER, audience FLOWER,
 * then the SCOOPS, each followed by a fire cycle; the last PARK_RESERVE_S is PARK.
 * Which pickups run is set before the match with AutoOptions (use the "Auto Setup" OpMode).
 *
 * Coordinates are Pedro map coordinates (inches, origin bottom-left, +x right, +y up, ~141.5 across,
 * read off the visualizer). Everything is written for the BLUE alliance; RED is the same layout
 * rotated 180 degrees about the field centre (the field is rotationally symmetric). Values marked
 * "guess" are NOT measured. Run one of the four subclasses, not this base.
 */
public abstract class BaseAuto extends NextOpMode {

    // ---- Field layout (BLUE alliance; red = rotated 180 about the centre) ----
    public static double FIELD = 141.5;                              // map extent, measured on the visualizer

    // Blue HIVE (frame x 72-95, y 45-96) has a CELL at each end, where the pieces are launched. In this
    // layout (blue; red is rotated) the TOP cell is the one that starts facing up (3 staged nectar inside).
    public static double[] TOP_CELL = {84, 87};       // cell centres, read off the map body. guess
    public static double[] BOTTOM_CELL = {84, 56};

    // Shoot spots: the circled areas in FRONT of each hive end on your map (x ~84, just outside the hive).
    // The Limelight reads the cell's tag best from right in front of our own hive, never from between
    // the two hives. The auto goes to the top one first, checks the turret sees that cell's tag (the up
    // cell's tag faces the floor), and moves to the other end if it can't. Check the robot body clears the hive.
    public static double[] SHOOT_TOP = {78.2, 101.5};     // = your red audience-scoop end (63.3, 40.0) rotated
    public static double[] SHOOT_BOTTOM = {81.0, 36.7};   // = your red pit-scoop end (60.5, 104.8) rotated
    public static double CHECK_TAG_S = 1.0;         // how long to wait for a tag before trying the other side
    public static double TIP_ASSUME_S = 1.0;        // tag gone this long after a shot: assume the hive tipped

    // PARK: partly inside the blue LOADING ZONE (right wall, y 23-46, 11 in deep). guess.
    public static double[] PARK = {126, 35, 0};
    public static double PARK_RESERVE_S = 5.0;                       // time kept free at the end to drive there

    // Pickups use your Paths.java (red as built, blue rotated 180). Fixed priority, skipping disabled ones:
    // preload, garden, pit flower, audience flower, then scoops (the side opposite the up cell first; toggles: AutoOptions / Auto Setup).
    private static final class Pick {
        final Pose start; final java.util.function.Supplier<com.pedropathing.paths.Path> path;
        final boolean scoopSlot;       // true: which scoop to run is decided when it is reached
        Pick(Pose start, java.util.function.Supplier<com.pedropathing.paths.Path> path) { this(start, path, false); }
        Pick(Pose start, java.util.function.Supplier<com.pedropathing.paths.Path> path, boolean scoopSlot) {
            this.start = start; this.path = path; this.scoopSlot = scoopSlot;
        }
    }
    private final List<Pick> pickups = new ArrayList<>();
    private Paths paths;
    private boolean pitScoopUsed, audScoopUsed;

    // ---- Timing ----
    public static double AUTO_SECONDS = 30;
    public static double END_BUFFER = 5.0;          // stop starting new things this close to the end (= time to PARK)

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
        paths = new Paths(follower, red, FIELD);
        follower.setPose(paths.startPose(audience));
        upTop = red;      // try the AUDIENCE-side cell first (bottom; folds to "top" in the blue layout for red); no tag -> pit side
        follower.update();

        // Only lock onto our own alliance's tags (red 30-37, blue 38-45), never the opponent's hive.
        robot.turret.setTagIdRange(red ? 30 : 38, red ? 37 : 45);

        match.reset();

        pickups.clear();
        if (AutoOptions.garden) pickups.add(new Pick(paths.gardenStart(), paths::garden));
        if (AutoOptions.flowerPit) pickups.add(new Pick(paths.flowerPitStart(), paths::flowerpitside));
        if (AutoOptions.flowerAudience) pickups.add(new Pick(paths.flowerAudienceStart(), paths::floweraudienceside));
        // Scoops: one slot per enabled scoop; which side runs is decided when the slot is reached (chooseScoop).
        pitScoopUsed = false;
        audScoopUsed = false;
        if (AutoOptions.scoopPit) pickups.add(new Pick(null, null, true));
        if (AutoOptions.scoopAudience) pickups.add(new Pick(null, null, true));
        int cycles = pickups.size();

        List<Command> steps = new ArrayList<>();
        steps.add(fireCycle());                                   // shoot the preload first
        for (int i = 0; i < cycles; i++) {
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
        log("Options", AutoOptions.summary());
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
        Pick pk = pickups.get(i);
        if (pk.scoopSlot) return Commands.lazy(() -> pickupRun(chooseScoop()));
        return pickupRun(pk);
    }

    /**
     * Whichever cell is up is the one we are shooting into; the OPPOSITE side's hive just dumped its balls, so
     * scoop there. Audience cell up -> pit scoop, pit cell up -> audience scoop. If that scoop is disabled or
     * already done, use the other one.
     */
    private Pick chooseScoop() {
        boolean audienceUp = red ? upTop : !upTop;
        boolean pitOk = AutoOptions.scoopPit && !pitScoopUsed;
        boolean audOk = AutoOptions.scoopAudience && !audScoopUsed;
        boolean pit = audienceUp ? pitOk : !audOk;
        if (pit && !pitOk) pit = false;
        if (!pit && !audOk) pit = true;
        if (pit) {
            pitScoopUsed = true;
            return new Pick(paths.scoopPitStart(), paths::scooppitside);
        }
        audScoopUsed = true;
        return new Pick(paths.scoopAudienceStart(), paths::scoopaudienceside);
    }

    private Command pickupRun(final Pick a) {
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
                                Commands.lazy(() -> followTo(a.start)).until(this::pickupAbort),
                                Commands.lazy(() -> PedroCommands.follow(follower, a.path.get())).until(this::pickupAbort),
                                Commands.instant(() -> pickupDone = true))));
    }

    private boolean pickupAbort() { return robot.Intake.isFull() || !timeOk(); }

    // ------------------------------------------------------------------ pose helpers

    /** Straight line from wherever the robot is now to the target pose, turning linearly. */
    // ---- Obstacle avoidance: never drive a straight line through our own HIVE ----
    // HIVE frame in the BLUE layout (red is folded onto it). Margins are how far the robot centre must stay
    // off the frame: sides = robot half-width + slack, ends = small, so the shoot spots in front of the ends
    // stay reachable. Tune these to your robot. Only routes around the outside (away from the centre line)
    // so we never wander onto the other alliance's half (G402). Other field elements are NOT modelled.
    public static double HIVE_X0 = 72, HIVE_X1 = 95, HIVE_Y0 = 45, HIVE_Y1 = 96;
    public static double HIVE_CLEAR_SIDE = 10, HIVE_CLEAR_END = 4;
    public static boolean HIVE_AVOID = true;      // false: drive straight lines, ignoring the hive entirely

    private double[] fold(double x, double y) {
        return red ? new double[]{FIELD - x, FIELD - y} : new double[]{x, y};
    }

    private Command lineTo(Pose a, Pose b) {
        return PedroCommands.follow(follower, com.pedropathing.api.Paths.line(a, b).linear(a, b));
    }

    /** Drive to target, detouring around the hive if the straight line would hit it. */
    private Command followTo(Pose target) {
        Pose cur = follower.pose();
        double[] a = fold(cur.x(), cur.y()), b = fold(target.x(), target.y());
        List<Command> legs = new ArrayList<>();
        Pose from = cur;
        for (double[] w : routeAroundHive(a, b)) {
            double[] r = fold(w[0], w[1]);
            Pose wp = deg.of(r[0], r[1], Math.toDegrees(target.heading()));
            legs.add(lineTo(from, wp));
            from = wp;
        }
        legs.add(lineTo(from, target));
        return legs.size() == 1 ? legs.get(0) : Groups.sequential(legs.toArray(new Command[0]));
    }

    /** Waypoints (blue layout) that keep a->b clear of the hive; empty if the direct line is already clear. */
    private List<double[]> routeAroundHive(double[] a, double[] b) {
        double x0 = HIVE_X0 - HIVE_CLEAR_SIDE, x1 = HIVE_X1 + HIVE_CLEAR_SIDE;
        double y0 = HIVE_Y0 - HIVE_CLEAR_END, y1 = HIVE_Y1 + HIVE_CLEAR_END;
        List<double[]> none = new ArrayList<>();
        if (!HIVE_AVOID) return none;
        if (!hitsRect(a, b, x0, y0, x1, y1)) return none;
        double[] tr = {x1, y1}, br = {x1, y0};            // outer corners (away from the centre line)
        double[][][] options = {{tr}, {br}, {br, tr}, {tr, br}};
        List<double[]> best = null;
        double bestLen = Double.MAX_VALUE;
        for (double[][] opt : options) {
            List<double[]> pts = new ArrayList<>();
            pts.add(a);
            for (double[] w : opt) pts.add(w);
            pts.add(b);
            double len = 0;
            boolean ok = true;
            for (int i = 0; i + 1 < pts.size() && ok; i++) {
                double[] p = pts.get(i), q = pts.get(i + 1);
                if (hitsRect(p, q, x0, y0, x1, y1)) ok = false;
                len += Math.hypot(q[0] - p[0], q[1] - p[1]);
            }
            if (ok && len < bestLen) {
                bestLen = len;
                best = new ArrayList<>();
                for (double[] w : opt) best.add(w);
            }
        }
        return best != null ? best : none;                // no clean detour (e.g. start inside): go direct
    }

    /** Does segment a-b pass through the interior of the rectangle? (Liang-Barsky, strict.) */
    private static boolean hitsRect(double[] a, double[] b, double x0, double y0, double x1, double y1) {
        double e = 1e-6;
        x0 += e; y0 += e; x1 -= e; y1 -= e;
        double dx = b[0] - a[0], dy = b[1] - a[1], t0 = 0, t1 = 1;
        double[] p = {-dx, dx, -dy, dy};
        double[] q = {a[0] - x0, x1 - a[0], a[1] - y0, y1 - a[1]};
        for (int i = 0; i < 4; i++) {
            if (p[i] == 0) {
                if (q[i] < 0) return false;
            } else {
                double r = q[i] / p[i];
                if (p[i] < 0) { if (r > t1) return false; if (r > t0) t0 = r; }
                else { if (r < t0) return false; if (r < t1) t1 = r; }
            }
        }
        return t0 < t1;
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
