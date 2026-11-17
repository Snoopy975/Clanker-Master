package org.firstinspires.ftc.teamcode.opmodes.auto.presets;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.opmodes.auto.BaseAuto;

import dev.nextftc.robot.opmode.NextAutonomous;

@NextAutonomous(name = "Blue Pit Start | Garden + AudFlower + BothScoops")
public class BluePit_Garden_AudFlower_BothScoops extends BaseAuto {
    public BluePit_Garden_AudFlower_BothScoops(Robot robot) {
        super(robot, false, false, true, false, true, true, true);
    }
}
