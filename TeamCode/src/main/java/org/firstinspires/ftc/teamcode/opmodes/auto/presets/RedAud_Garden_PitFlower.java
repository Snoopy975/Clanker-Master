package org.firstinspires.ftc.teamcode.opmodes.auto.presets;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.opmodes.auto.BaseAuto;

import dev.nextftc.robot.opmode.NextAutonomous;

@NextAutonomous(name = "Red Audience Start | Garden + PitFlower + Scoops")
public class RedAud_Garden_PitFlower extends BaseAuto {
    public RedAud_Garden_PitFlower(Robot robot) {
        super(robot, true, true, true, true, false);
    }
}
