package org.firstinspires.ftc.teamcode.opmodes.auto;

import org.firstinspires.ftc.teamcode.Robot;

import dev.nextftc.robot.opmode.NextAutonomous;

@NextAutonomous(name = "Red Rear Side")
public class RedRearAuto extends BaseAuto {
    public RedRearAuto(Robot robot) {
        super(robot, true, false);
    }
}
