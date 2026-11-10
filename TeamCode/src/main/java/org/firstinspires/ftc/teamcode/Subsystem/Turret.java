package org.firstinspires.ftc.teamcode.Subsystem;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import dev.nextftc.robot.Mechanism;

public class Turret implements Mechanism {

    public static final Turret INSTANCE = new Turret();

    private Turret() {}

    private Servo turretServo;
    private double kP = 0.0001;

}
