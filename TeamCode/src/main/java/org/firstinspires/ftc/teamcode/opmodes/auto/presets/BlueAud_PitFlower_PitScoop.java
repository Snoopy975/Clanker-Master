package org.firstinspires.ftc.teamcode.opmodes.auto.presets;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.opmodes.auto.BaseAuto;

import dev.nextftc.robot.opmode.NextAutonomous;

@NextAutonomous(name = "Blue Audience Start | PitFlower + PitScoop")
public class BlueAud_PitFlower_PitScoop extends BaseAuto {
    public BlueAud_PitFlower_PitScoop(Robot robot) {
        super(robot, false, true, false, true, false, true, false);
    }
}
