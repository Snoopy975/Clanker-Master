package org.firstinspires.ftc.teamcode.opmodes.auto.presets;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.opmodes.auto.BaseAuto;

import dev.nextftc.robot.opmode.NextAutonomous;

@NextAutonomous(name = "Blue Pit Start | Garden + PitFlower + AudScoop")
public class BluePit_Garden_PitFlower_AudScoop extends BaseAuto {
    public BluePit_Garden_PitFlower_AudScoop(Robot robot) {
        super(robot, false, false, true, true, false, false, true);
    }
}
