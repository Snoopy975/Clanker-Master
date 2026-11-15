package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.mechanisms.Drive;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;

import java.util.Set;

import dev.nextftc.robot.Mechanism;
import dev.nextftc.robot.NextRobot;

public class Robot implements NextRobot {

    public final Drive drive = new Drive();
    public final Intake Intake = new Intake();


    @Override
    public Set<Mechanism> getMechanisms() {
        return Set.of(drive, Intake);
    }
}
