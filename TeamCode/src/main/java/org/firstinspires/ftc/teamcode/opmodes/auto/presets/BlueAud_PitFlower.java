package org.firstinspires.ftc.teamcode.opmodes.auto.presets;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.opmodes.auto.BaseAuto;

import dev.nextftc.robot.opmode.NextAutonomous;

@NextAutonomous(name = "Blue Audience Start | PitFlower + Scoops")
public class BlueAud_PitFlower extends BaseAuto {
    public BlueAud_PitFlower(Robot robot) {
        super(robot, false, true, false, true, false);
    }
}
