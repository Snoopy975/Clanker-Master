package org.firstinspires.ftc.teamcode.opmodes.auto.presets;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.opmodes.auto.BaseAuto;

import dev.nextftc.robot.opmode.NextAutonomous;

@NextAutonomous(name = "Blue Pit Start | PitScoop")
public class BluePit_PitScoop extends BaseAuto {
    public BluePit_PitScoop(Robot robot) {
        super(robot, false, false, false, false, false, true, false);
    }
}
