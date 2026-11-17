package org.firstinspires.ftc.teamcode.opmodes.auto.presets;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.opmodes.auto.BaseAuto;

import dev.nextftc.robot.opmode.NextAutonomous;

@NextAutonomous(name = "Blue Audience Start | AudScoop")
public class BlueAud_AudScoop extends BaseAuto {
    public BlueAud_AudScoop(Robot robot) {
        super(robot, false, true, false, false, false, false, true);
    }
}
