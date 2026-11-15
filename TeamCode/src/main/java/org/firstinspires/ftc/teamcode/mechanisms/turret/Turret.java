package org.firstinspires.ftc.teamcode.mechanisms.turret;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.data.Config;

import dev.nextftc.hardware.RobotController;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.hardware.actuators.NextServo;
import dev.nextftc.robot.Mechanism;

public class Turret implements Mechanism {

    private Turret() {}

    public final NextServo servo1 = new NextServo(RobotController.controlHub(), Config.turret1);
    public final NextServo servo2 = new NextServo(RobotController.controlHub(), Config.turret2);
    private double kP = 0.0001;

}
