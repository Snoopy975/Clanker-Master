package org.firstinspires.ftc.teamcode.opmodes.auto.presets;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.opmodes.auto.BaseAuto;

import dev.nextftc.robot.opmode.NextAutonomous;

@NextAutonomous(name = "Red Audience Start | PitFlower + AudScoop")
public class RedAud_PitFlower_AudScoop extends BaseAuto {
    public RedAud_PitFlower_AudScoop(Robot robot) {
        super(robot, true, true, false, true, false, false, true);
    }
}
