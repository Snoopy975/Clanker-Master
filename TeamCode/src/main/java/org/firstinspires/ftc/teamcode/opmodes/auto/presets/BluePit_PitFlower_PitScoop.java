package org.firstinspires.ftc.teamcode.opmodes.auto.presets;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.opmodes.auto.BaseAuto;

import dev.nextftc.robot.opmode.NextAutonomous;

@NextAutonomous(name = "Blue Pit Start | PitFlower + PitScoop")
public class BluePit_PitFlower_PitScoop extends BaseAuto {
    public BluePit_PitFlower_PitScoop(Robot robot) {
        super(robot, false, false, false, true, false, true, false);
    }
}
