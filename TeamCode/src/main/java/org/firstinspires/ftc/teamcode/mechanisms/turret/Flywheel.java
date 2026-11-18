package org.firstinspires.ftc.teamcode.mechanisms.turret;

import org.firstinspires.ftc.teamcode.data.Config;

import dev.nextftc.hardware.RobotController;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;

public class Flywheel implements Mechanism {
    public final NextMotor motor = new NextMotor(RobotController.expansionHub(), Config.FlywheelMotor);

    private double throttle = 0;

    public void setThrottle(double t) {
        throttle = t;
        motor.setThrottle(t);
    }

    public double getThrottle() { return throttle; }

    public void stop() { setThrottle(0); }
}