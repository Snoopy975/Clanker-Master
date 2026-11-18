package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;

import org.firstinspires.ftc.teamcode.data.Config;

import dev.nextftc.hardware.RobotController;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.hardware.sensors.NextDigitalSensor;
import dev.nextftc.robot.Mechanism;

public class Intake implements Mechanism {
    NextMotor motor = new NextMotor(RobotController.controlHub(), Config.intake);
    NextDigitalSensor beamBreak = new NextDigitalSensor(RobotController.controlHub(), Config.intakeSensor);

    public static int CAPACITY = 4;
    private int count = 0;
    private boolean wasTriggered = false;

    @Override
    public void periodic() {
        boolean raw = beamBreak.getRawState();
        boolean triggered = beamBreak.isTriggered();
        if (triggered && !wasTriggered) count++; // one piece per clear -> blocked edge
        wasTriggered = triggered;
    }

    public int getCount() { return count; }
    public void resetCount() { count = 0; }
    public boolean isFull() { return count >= CAPACITY; }

    /** Immediate stop (for OpMode end, where no command will run again). */
    public void halt() { motor.setThrottle(0); }

    public Command run() {
        return Commands.infinite(() -> motor.setThrottle(1.0));
    }
    public Command out(){
        return Commands.instant(() -> motor.setThrottle(-1.0));
    }
    public Command stop(){
        return Commands.instant(() -> motor.setThrottle(0));
    }
}