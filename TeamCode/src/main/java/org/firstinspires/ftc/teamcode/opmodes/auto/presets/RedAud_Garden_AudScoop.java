package org.firstinspires.ftc.teamcode.opmodes.auto.presets;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.opmodes.auto.BaseAuto;

import dev.nextftc.robot.opmode.NextAutonomous;

@NextAutonomous(name = "Red Audience Start | Garden + AudScoop")
public class RedAud_Garden_AudScoop extends BaseAuto {
    public RedAud_Garden_AudScoop(Robot robot) {
        super(robot, true, true, true, false, false, false, true);
    }
}
