package org.firstinspires.ftc.teamcode.opmodes.auto;

import org.firstinspires.ftc.teamcode.Robot;

import dev.nextftc.robot.opmode.NextAutonomous;

@NextAutonomous(name = "Red Scoring Side")
public class RedScoringAuto extends BaseAuto {
    public RedScoringAuto(Robot robot) {
        super(robot, true, false);
    }
}
