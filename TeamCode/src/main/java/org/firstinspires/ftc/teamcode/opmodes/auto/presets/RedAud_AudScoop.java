package org.firstinspires.ftc.teamcode.opmodes.auto.presets;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.opmodes.auto.BaseAuto;

import dev.nextftc.robot.opmode.NextAutonomous;

@NextAutonomous(name = "Red Audience Start | AudScoop")
public class RedAud_AudScoop extends BaseAuto {
    public RedAud_AudScoop(Robot robot) {
        super(robot, true, true, false, false, false, false, true);
    }
}
