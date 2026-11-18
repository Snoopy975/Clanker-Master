package org.firstinspires.ftc.teamcode.mechanisms.turret;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import org.firstinspires.ftc.teamcode.data.Config;

import java.util.List;

import dev.nextftc.hardware.RobotController;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.units.Units;

public class Turret implements Mechanism {

    public Turret() {}

    public final NextMotor motor = new NextMotor(RobotController.controlHub(), Config.turret);

    // ---- Tuning ----
    private static final int APRILTAG_PIPELINE = 0;
    private static final int TARGET_TAG_ID = -1;          // -1 = any tag
    private static final double MOTOR_SIGN = 1.0;

    private static final double TICKS_PER_TURRET_REV = 3895.9; // CHANGE ME (ticks/rev * gear ratio)
    private static final boolean CONTINUOUS = false;
    private static final double MIN_DEG = 0.0;            // cable fully unwound = power-on position
    private static final double MAX_DEG = 360.0;          // cable fully wound, never go past this

    private static final double kP = 0.025;               // power per degree of tx error
    private static final double kD = 0.0015;              // damping (power per deg/sec)
    private static final double MAX_TRACK_POWER = 0.7;
    private static final double kS = 0.05;                // min power to break static friction
    private static final double DEADBAND_DEG = 0.7;

    private static final double SEARCH_POWER = 0.30;
    private static final double SEARCH_CENTER_DEG = 180.0;     // middle of travel = "forward" (CHANGE ME if forward is elsewhere)
    private static final double SEARCH_HALF_RANGE_DEG = 45.0;  // sweep +/- 45 = 90 degrees total
    private static final long LOST_GRACE_MS = 250;

    private static final double UNWIND_POWER = 0.6;
    private static final double UNWIND_TOL_DEG = 8.0;

    public enum State { TRACKING, SEARCHING, UNWINDING }

    private double unwindTargetDeg = 0;

    private Limelight3A limelight;
    private State state = State.SEARCHING;
    private long lastSeenMs = 0;
    private double searchDir = 1.0;
    private double lastTx = 0;
    private double prevErr = 0;
    private long prevErrMs = 0;

    public State getState() { return state; }

    // ---- Read by the autonomous ----
    private int minTagId = -1, maxTagId = -1;

    /** Only track tags with an ID in [min, max] (e.g. red 30-37, blue 38-45). Negative min = any tag. */
    public void setTagIdRange(int min, int max) { minTagId = min; maxTagId = max; }

    private boolean tagVisibleNow = false;
    private double tagDistIn = 0;
    private int tagId = -1;

    /** ID of the tag it is currently tracking (valid while isTagVisible()). */
    public int getTagId() { return tagId; }

    /** A tag was in the Limelight frame on the latest loop. */
    public boolean isTagVisible() { return tagVisibleNow; }

    /** 3D camera-to-tag distance in inches (last seen tag; unaffected by the camera's tilt). */
    public double getTagDistanceIn() { return tagDistIn; }

    /** Tracking a visible tag and its error is within tolDeg. */
    public boolean isAimed(double tolDeg) {
        return state == State.TRACKING && tagVisibleNow && Math.abs(lastTx) < tolDeg;
    }
    public double getLastTx() { return lastTx; }

    /** Turret angle in degrees; 0 = forward at power-on, positive = clockwise. */
    public double getAngleDeg() {
        return MOTOR_SIGN * encoderCounts() * 360.0 / TICKS_PER_TURRET_REV;
    }

    /** Raw encoder counts. NextMotor reports an Angle = counts * anglePerCount, and anglePerCount is left at its 1 rad default. */
    private double encoderCounts() {
        return motor.getEncoderPosition().into(Units.Radians);
    }

    /** NextFTC's Mechanism has no initialize(); the Limelight is set up on the first periodic() instead. */
    private void ensureInit() {
        if (limelight != null) return;
        limelight = RobotController.hardwareMap().get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        limelight.start();
        lastSeenMs = System.currentTimeMillis();
        setTurnPower(0);
    }

    @Override
    public void periodic() {
        ensureInit();
        long now = System.currentTimeMillis();
        tagVisibleNow = false;

        // Unwinding: spin back 360 degrees (same heading) to free the cables, ignoring tags
        if (state == State.UNWINDING) {
            double err = unwindTargetDeg - getAngleDeg();
            if (Math.abs(err) < UNWIND_TOL_DEG) {
                state = State.TRACKING;
                lastSeenMs = now; // give tracking a moment to re-acquire before searching
                prevErrMs = 0;
                setTurnPower(0);
            } else {
                setTurnPower(Math.copySign(UNWIND_POWER, err));
            }
            return;
        }

        Double tx = readTagTx();
        tagVisibleNow = tx != null;

        if (tx != null) {
            lastSeenMs = now;
            lastTx = tx;
            state = State.TRACKING;
            track(tx, now);
            return;
        }

        prevErrMs = 0; // reset derivative when the tag is lost

        if (now - lastSeenMs < LOST_GRACE_MS) {
            setTurnPower(0);
            return;
        }

        if (state != State.SEARCHING) {
            state = State.SEARCHING;
            searchDir = lastTx >= 0 ? 1.0 : -1.0; // continue the way the tag was last heading
        }
        search();
    }

    private void track(double txDeg, long now) {
        if (Math.abs(txDeg) < DEADBAND_DEG) {
            setTurnPower(0);
            prevErr = txDeg;
            prevErrMs = now;
            return;
        }
        double d = 0;
        if (prevErrMs != 0 && now > prevErrMs) {
            d = (txDeg - prevErr) / ((now - prevErrMs) / 1000.0);
        }
        prevErr = txDeg;
        prevErrMs = now;

        double p = kP * txDeg + kD * d;
        p = Math.max(-MAX_TRACK_POWER, Math.min(MAX_TRACK_POWER, p));
        if (Math.abs(p) < kS) p = Math.copySign(kS, p);

        // Target angle is outside 0..360: wrap it (e.g. 370 -> 10) and spin back around to it
        if (!CONTINUOUS) {
            double a = getAngleDeg();
            double desired = a + txDeg; // tx > 0 means the tag is clockwise of us
            if (desired > MAX_DEG) {
                unwindTargetDeg = desired - 360.0;
                state = State.UNWINDING;
                return;
            } else if (desired < MIN_DEG) {
                unwindTargetDeg = desired + 360.0;
                state = State.UNWINDING;
                return;
            }
        }
        setTurnPower(p);
    }

    private void search() {
        // Sweep back and forth within SEARCH_CENTER_DEG +/- SEARCH_HALF_RANGE_DEG.
        // If the turret is outside that window (e.g. after losing a tag far away),
        // this also drives it back into the window first.
        double a = getAngleDeg();
        if (a >= SEARCH_CENTER_DEG + SEARCH_HALF_RANGE_DEG) searchDir = -1.0;
        else if (a <= SEARCH_CENTER_DEG - SEARCH_HALF_RANGE_DEG) searchDir = 1.0;
        setTurnPower(searchDir * SEARCH_POWER);
    }

    private Double readTagTx() {
        if (limelight == null) return null;
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return null;
        if (result.getPipelineIndex() != APRILTAG_PIPELINE) return null; // stale frame

        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        if (tags == null || tags.isEmpty()) return null;

        LLResultTypes.FiducialResult best = null;
        for (LLResultTypes.FiducialResult t : tags) {
            if (TARGET_TAG_ID != -1 && t.getFiducialId() != TARGET_TAG_ID) continue;
            if (minTagId >= 0 && (t.getFiducialId() < minTagId || t.getFiducialId() > maxTagId)) continue;
            if (best == null || Math.abs(t.getTargetXDegrees()) < Math.abs(best.getTargetXDegrees())) {
                best = t;
            }
        }
        if (best == null) return null;
        Position pos = best.getTargetPoseCameraSpace().getPosition().toUnit(DistanceUnit.INCH);
        tagDistIn = Math.sqrt(pos.x * pos.x + pos.y * pos.y + pos.z * pos.z);
        tagId = best.getFiducialId();
        return best.getTargetXDegrees();
    }

    /** power in [-1, 1]; positive = clockwise. Soft limits stop it driving into the cable limit. */
    private void setTurnPower(double power) {
        if (!CONTINUOUS) {
            double a = getAngleDeg();
            if ((a >= MAX_DEG && power > 0) || (a <= MIN_DEG && power < 0)) power = 0;
        }
        motor.setThrottle(Math.max(-1, Math.min(1, power)) * MOTOR_SIGN);
    }
}
