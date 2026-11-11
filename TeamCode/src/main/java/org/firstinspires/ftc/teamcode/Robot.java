package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.Mechanisms.Drive;

import java.util.Set;

import dev.nextftc.robot.Mechanism;
import dev.nextftc.robot.NextRobot;

/**
 * The ONE NextRobot implementation. NextFTC finds it by scanning the classpath and hands the same
 * instance to every NextOpMode. It needs a public no-arg constructor.
 * Add every Mechanism here or its periodic() never runs (e.g. Flywheel, Turret).
 */
public class Robot implements NextRobot {

    public final Drive drive = new Drive();

    @Override
    public Set<Mechanism> getMechanisms() {
        return Set.of(drive);
    }
}
