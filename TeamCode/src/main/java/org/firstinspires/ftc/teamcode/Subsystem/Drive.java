package org.firstinspires.ftc.teamcode.Subsystem;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;

/**
 * Field-centric mecanum drive with acceleration (slew-rate) limiting.
 * Call drive(...) every loop with raw stick values; periodic() does the rest.
 */
public class Drive implements Subsystem {

    public static final Drive INSTANCE = new Drive();
    private Drive() { }

    // Motor names/directions match the old Tele.java (left side reversed).
    private final MotorEx fl = new MotorEx("fl").reversed();
    private final MotorEx bl = new MotorEx("bl").reversed();
    private final MotorEx fr = new MotorEx("fr");
    private final MotorEx br = new MotorEx("br");

    private IMU imu;

    // Max change in power per second. 4.0 = 0 -> full in 0.25 s. Raise until wheels slip, then back off.
    public static double MAX_ACCEL = 4.0;
    // Rotation can usually be limited less than translation.
    public static double MAX_TURN_ACCEL = 8.0;
    // Counteracts imperfect mecanum strafing (same fudge as the old Tele).
    public static double STRAFE_SCALE = 1.1;
    // If true, ramp-down is not limited so letting go of the stick stops quickly.
    public static boolean INSTANT_STOP = true;

    // Raw targets from the driver
    private double targetX, targetY, targetRx;
    // Last limited outputs
    private double lastX, lastY, lastRx;

    private final ElapsedTime timer = new ElapsedTime();

    @Override
    public void initialize() {
        imu = ActiveOpMode.hardwareMap().get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        timer.reset();
    }

    /** Pass raw stick values: y = forward (already negated), x = strafe right, rx = turn. */
    public void drive(double y, double x, double rx) {
        targetY = y;
        targetX = x;
        targetRx = rx;
    }

    /** Re-zero field-centric heading. */
    public void resetHeading() {
        imu.resetYaw();
    }

    private double slew(double target, double last, double dt, double maxAccel) {
        if (INSTANT_STOP && Math.abs(target) < Math.abs(last)) {
            return target;
        }
        double step = maxAccel * dt;
        return last + Math.max(-step, Math.min(step, target - last));
    }

    @Override
    public void periodic() {
        double dt = timer.seconds();
        timer.reset();

        double y = slew(targetY, lastY, dt, MAX_ACCEL);
        double x = slew(targetX, lastX, dt, MAX_ACCEL);
        double rx = slew(targetRx, lastRx, dt, MAX_TURN_ACCEL);
        lastY = y; lastX = x; lastRx = rx;

        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
        double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);
        rotX *= STRAFE_SCALE;

        double denom = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

        fl.setPower((rotY + rotX + rx) / denom);
        bl.setPower((rotY - rotX + rx) / denom);
        fr.setPower((rotY - rotX - rx) / denom);
        br.setPower((rotY + rotX - rx) / denom);
    }
}
