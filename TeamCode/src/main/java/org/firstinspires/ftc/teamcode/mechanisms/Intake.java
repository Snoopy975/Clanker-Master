package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.data.Config;

import dev.nextftc.hardware.RobotController;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.hardware.sensors.NextDigitalSensor;
import dev.nextftc.robot.Mechanism;

public class Intake implements Mechanism {
    NextMotor motor = new NextMotor(RobotController.controlHub(), Config.intake);
    NextDigitalSensor beamBreak = new NextDigitalSensor(RobotController.controlHub(), Config.intakeSensor);

    @Override
    public void periodic() {
        boolean raw = beamBreak.getRawState();
        if (beamBreak.isTriggered()) {

        }
    }

    public Command run() {
        return infinite(() -> motor.setThrottle(1.0));
    }
    public Command out(){
        return instant(() -> motor.setThrottle(-1.0));
    }
    public Command stop(){
        return instant(() -> motor.setThrottle(0));
    }
}