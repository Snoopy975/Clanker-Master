package org.firstinspires.ftc.teamcode.opmodes.auto;

import org.firstinspires.ftc.teamcode.Robot;

import dev.nextftc.robot.opmode.NextAutonomous;

@NextAutonomous(name = "Red Auto")
public class RedAuto extends BaseAuto {
    public RedAuto(Robot robot) {
        super(robot, true);
    }
}
