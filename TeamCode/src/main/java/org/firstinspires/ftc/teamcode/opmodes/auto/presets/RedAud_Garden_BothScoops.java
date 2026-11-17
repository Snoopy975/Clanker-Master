package org.firstinspires.ftc.teamcode.opmodes.auto.presets;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.opmodes.auto.BaseAuto;

import dev.nextftc.robot.opmode.NextAutonomous;

@NextAutonomous(name = "Red Audience Start | Garden + BothScoops")
public class RedAud_Garden_BothScoops extends BaseAuto {
    public RedAud_Garden_BothScoops(Robot robot) {
        super(robot, true, true, true, false, false, true, true);
    }
}
