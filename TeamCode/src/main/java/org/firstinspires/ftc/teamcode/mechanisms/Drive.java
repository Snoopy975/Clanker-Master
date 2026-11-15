
package org.firstinspires.ftc.teamcode.mechanisms;

import org.firstinspires.ftc.teamcode.data.Config;

import dev.nextftc.hardware.RobotController;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;

public class Drive implements Mechanism {
    public Drive(){
        frontRight.setDirection(NextMotor.Direction.REVERSE);
        backRight.setDirection(NextMotor.Direction.REVERSE);
    }
    public final NextMotor frontLeft = new NextMotor(RobotController.controlHub(), Config.fl);
    public final NextMotor frontRight = new NextMotor(RobotController.expansionHub(), Config.fr);
    public final NextMotor backLeft = new NextMotor(RobotController.controlHub(), Config.bl);
    public final NextMotor backRight =  new NextMotor(RobotController.expansionHub(), Config.br);

    /** Robot-relative power: forward, strafe LEFT, turn CCW. Right-side motors are already reversed in the constructor. */
    public void setPowers(double forward, double left, double ccw) {
        double fl = forward - left - ccw;
        double fr = forward + left + ccw;
        double bl = forward + left - ccw;
        double br = forward - left + ccw;
        double max = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(fr)), Math.max(Math.abs(bl), Math.abs(br))));
        frontLeft.setThrottle(fl / max);
        frontRight.setThrottle(fr / max);
        backLeft.setThrottle(bl / max);
        backRight.setThrottle(br / max);
    }

    public void stop() {
        setPowers(0, 0, 0);
    }

}