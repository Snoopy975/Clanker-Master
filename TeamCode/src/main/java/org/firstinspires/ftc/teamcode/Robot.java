package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.mechanisms.Drive;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.turret.Flywheel;
import org.firstinspires.ftc.teamcode.mechanisms.turret.Turret;

import java.util.Set;

import dev.nextftc.robot.Mechanism;
import dev.nextftc.robot.NextRobot;

public class Robot implements NextRobot {

    public final Drive drive = new Drive();
    public final Intake Intake = new Intake();
    public final Flywheel flywheel = new Flywheel();
    public final Turret turret = new Turret();


    @Override
    public Set<Mechanism> getMechanisms() {
        return Set.of(drive, Intake, flywheel, turret);
    }
}
