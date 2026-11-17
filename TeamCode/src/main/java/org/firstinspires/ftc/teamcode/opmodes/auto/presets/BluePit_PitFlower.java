package org.firstinspires.ftc.teamcode.opmodes.auto.presets;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.opmodes.auto.BaseAuto;

import dev.nextftc.robot.opmode.NextAutonomous;

@NextAutonomous(name = "Blue Pit Start | PitFlower")
public class BluePit_PitFlower extends BaseAuto {
    public BluePit_PitFlower(Robot robot) {
        super(robot, false, false, false, true, false, false, false);
    }
}
