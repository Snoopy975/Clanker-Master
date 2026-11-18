package org.firstinspires.ftc.teamcode.opmodes.auto.presets;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.opmodes.auto.BaseAuto;

import dev.nextftc.robot.opmode.NextAutonomous;

@NextAutonomous(name = "Red Pit Start | Scoop Only", group = "Red Pit Start")
public class RedPit_ScoopOnly extends BaseAuto {
    public RedPit_ScoopOnly(Robot robot) {
        super(robot, true, false, false, false, false);
    }
}
