package org.firstinspires.ftc.teamcode.opmodes.auto.presets;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.opmodes.auto.BaseAuto;

import dev.nextftc.robot.opmode.NextAutonomous;

@NextAutonomous(name = "Blue Pit Start | Garden + Scoops")
public class BluePit_Garden extends BaseAuto {
    public BluePit_Garden(Robot robot) {
        super(robot, false, false, true, false, false);
    }
}
