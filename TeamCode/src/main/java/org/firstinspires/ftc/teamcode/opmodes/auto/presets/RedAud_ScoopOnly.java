package org.firstinspires.ftc.teamcode.opmodes.auto.presets;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.opmodes.auto.BaseAuto;

import dev.nextftc.robot.opmode.NextAutonomous;

@NextAutonomous(name = "Red Audience Start | Scoop Only", group = "Red Audience Start")
public class RedAud_ScoopOnly extends BaseAuto {
    public RedAud_ScoopOnly(Robot robot) {
        super(robot, true, true, false, false, false);
    }
}
