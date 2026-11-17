package org.firstinspires.ftc.teamcode.opmodes.auto.presets;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.opmodes.auto.BaseAuto;

import dev.nextftc.robot.opmode.NextAutonomous;

@NextAutonomous(name = "Red Pit Start | PitScoop")
public class RedPit_PitScoop extends BaseAuto {
    public RedPit_PitScoop(Robot robot) {
        super(robot, true, false, false, false, false, true, false);
    }
}
