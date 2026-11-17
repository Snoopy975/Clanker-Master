package org.firstinspires.ftc.teamcode.opmodes.auto.presets;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.opmodes.auto.BaseAuto;

import dev.nextftc.robot.opmode.NextAutonomous;

@NextAutonomous(name = "Red Pit Start | Preload Only")
public class RedPit_PreloadOnly extends BaseAuto {
    public RedPit_PreloadOnly(Robot robot) {
        super(robot, true, false, false, false, false, false, false);
    }
}
