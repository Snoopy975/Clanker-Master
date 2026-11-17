package org.firstinspires.ftc.teamcode.opmodes.auto.presets;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.opmodes.auto.BaseAuto;

import dev.nextftc.robot.opmode.NextAutonomous;

@NextAutonomous(name = "Blue Audience Start | Scoop Only")
public class BlueAud_ScoopOnly extends BaseAuto {
    public BlueAud_ScoopOnly(Robot robot) {
        super(robot, false, true, false, false, false);
    }
}
