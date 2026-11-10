package org.firstinspires.ftc.teamcode.Mechanisms;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;

public class Flywheel implements Mechanism {

    public static final Flywheel INSTANCE = new Flywheel();

    NextMotor motor = new NextMotor("outL");
    NextMotor motor2 = new NextMotor("outR");

}