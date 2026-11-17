package org.firstinspires.ftc.teamcode.opmodes.auto.presets;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.opmodes.auto.BaseAuto;

import dev.nextftc.robot.opmode.NextAutonomous;

@NextAutonomous(name = "Red Audience Start | PitFlower + BothScoops")
public class RedAud_PitFlower_BothScoops extends BaseAuto {
    public RedAud_PitFlower_BothScoops(Robot robot) {
        super(robot, true, true, false, true, false, true, true);
    }
}
