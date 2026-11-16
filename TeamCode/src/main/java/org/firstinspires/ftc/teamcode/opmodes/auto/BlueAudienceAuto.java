package org.firstinspires.ftc.teamcode.opmodes.auto;

import org.firstinspires.ftc.teamcode.Robot;

import dev.nextftc.robot.opmode.NextAutonomous;

@NextAutonomous(name = "Blue Audience Side")
public class BlueAudienceAuto extends BaseAuto {
    public BlueAudienceAuto(Robot robot) {
        super(robot, false, true);
    }
}
