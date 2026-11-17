package org.firstinspires.ftc.teamcode.opmodes.auto.presets;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.opmodes.auto.BaseAuto;

import dev.nextftc.robot.opmode.NextAutonomous;

@NextAutonomous(name = "Red Pit Start | Garden + PitFlower + AudScoop")
public class RedPit_Garden_PitFlower_AudScoop extends BaseAuto {
    public RedPit_Garden_PitFlower_AudScoop(Robot robot) {
        super(robot, true, false, true, true, false, false, true);
    }
}
