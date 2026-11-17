package org.firstinspires.ftc.teamcode.opmodes.auto.presets;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.opmodes.auto.BaseAuto;

import dev.nextftc.robot.opmode.NextAutonomous;

@NextAutonomous(name = "Red Audience Start | BothScoops")
public class RedAud_BothScoops extends BaseAuto {
    public RedAud_BothScoops(Robot robot) {
        super(robot, true, true, false, false, false, true, true);
    }
}
