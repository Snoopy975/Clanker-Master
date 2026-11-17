package org.firstinspires.ftc.teamcode.opmodes.auto.presets;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.opmodes.auto.BaseAuto;

import dev.nextftc.robot.opmode.NextAutonomous;

@NextAutonomous(name = "Blue Pit Start | BothScoops")
public class BluePit_BothScoops extends BaseAuto {
    public BluePit_BothScoops(Robot robot) {
        super(robot, false, false, false, false, false, true, true);
    }
}
