package org.firstinspires.ftc.teamcode.opmodes.auto;

import org.firstinspires.ftc.teamcode.Robot;

import dev.nextftc.robot.opmode.NextAutonomous;

@NextAutonomous(name = "Red Audience Side")
public class RedAudienceAuto extends BaseAuto {
    public RedAudienceAuto(Robot robot) {
        super(robot, true, true);
    }
}
