package org.firstinspires.ftc.teamcode.opmodes.auto.presets;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.opmodes.auto.BaseAuto;

import dev.nextftc.robot.opmode.NextAutonomous;

@NextAutonomous(name = "Red Pit Start | Garden + AudFlower + BothScoops")
public class RedPit_Garden_AudFlower_BothScoops extends BaseAuto {
    public RedPit_Garden_AudFlower_BothScoops(Robot robot) {
        super(robot, true, false, true, false, true, true, true);
    }
}
