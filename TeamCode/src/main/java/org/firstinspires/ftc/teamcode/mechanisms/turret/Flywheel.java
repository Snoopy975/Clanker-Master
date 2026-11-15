package org.firstinspires.ftc.teamcode.mechanisms.turret;

import com.qualcomm.robotcore.hardware.HardwareMap;

import dev.nextftc.hardware.actuators.NextMotor;

public class Flywheel {
    NextMotor motor = new NextMotor("out");

    private double encoderCPM = 28;

    private double kV, kS, kP;

    private void init(HardwareMap hwmap){

    }
}