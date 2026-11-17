package org.firstinspires.ftc.teamcode.opmodes.auto.presets;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.opmodes.auto.BaseAuto;

import dev.nextftc.robot.opmode.NextAutonomous;

@NextAutonomous(name = "Red Audience Start | Garden + AudFlower + PitScoop")
public class RedAud_Garden_AudFlower_PitScoop extends BaseAuto {
    public RedAud_Garden_AudFlower_PitScoop(Robot robot) {
        super(robot, true, true, true, false, true, true, false);
    }
}
