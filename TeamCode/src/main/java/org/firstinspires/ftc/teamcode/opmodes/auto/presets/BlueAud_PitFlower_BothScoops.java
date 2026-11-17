package org.firstinspires.ftc.teamcode.opmodes.auto.presets;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.opmodes.auto.BaseAuto;

import dev.nextftc.robot.opmode.NextAutonomous;

@NextAutonomous(name = "Blue Audience Start | PitFlower + BothScoops")
public class BlueAud_PitFlower_BothScoops extends BaseAuto {
    public BlueAud_PitFlower_BothScoops(Robot robot) {
        super(robot, false, true, false, true, false, true, true);
    }
}
