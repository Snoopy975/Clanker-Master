package org.firstinspires.ftc.teamcode.opmodes.auto.presets;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.opmodes.auto.BaseAuto;

import dev.nextftc.robot.opmode.NextAutonomous;

@NextAutonomous(name = "Blue Pit Start | Preload Only")
public class BluePit_PreloadOnly extends BaseAuto {
    public BluePit_PreloadOnly(Robot robot) {
        super(robot, false, false, false, false, false, false, false);
    }
}
