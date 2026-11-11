package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.Servo;

import dev.nextftc.robot.Mechanism;

public class Turret implements Mechanism {

    public static final Turret INSTANCE = new Turret();

    private Turret() {}

    private Servo turretServo;
    private double kP = 0.0001;

}
