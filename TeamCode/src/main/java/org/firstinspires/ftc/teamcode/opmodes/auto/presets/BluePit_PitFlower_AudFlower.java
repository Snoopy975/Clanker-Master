package org.firstinspires.ftc.teamcode.opmodes.auto.presets;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.opmodes.auto.BaseAuto;

import dev.nextftc.robot.opmode.NextAutonomous;

@NextAutonomous(name = "Blue Pit Start | PitFlower + AudFlower")
public class BluePit_PitFlower_AudFlower extends BaseAuto {
    public BluePit_PitFlower_AudFlower(Robot robot) {
        super(robot, false, false, false, true, true, false, false);
    }
}
