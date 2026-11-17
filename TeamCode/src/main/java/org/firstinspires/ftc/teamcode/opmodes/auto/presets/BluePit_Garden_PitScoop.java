package org.firstinspires.ftc.teamcode.opmodes.auto.presets;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.opmodes.auto.BaseAuto;

import dev.nextftc.robot.opmode.NextAutonomous;

@NextAutonomous(name = "Blue Pit Start | Garden + PitScoop")
public class BluePit_Garden_PitScoop extends BaseAuto {
    public BluePit_Garden_PitScoop(Robot robot) {
        super(robot, false, false, true, false, false, true, false);
    }
}
