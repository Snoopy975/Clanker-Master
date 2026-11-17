package org.firstinspires.ftc.teamcode.opmodes.auto.presets;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.opmodes.auto.BaseAuto;

import dev.nextftc.robot.opmode.NextAutonomous;

@NextAutonomous(name = "Red Pit Start | Garden")
public class RedPit_Garden extends BaseAuto {
    public RedPit_Garden(Robot robot) {
        super(robot, true, false, true, false, false, false, false);
    }
}
