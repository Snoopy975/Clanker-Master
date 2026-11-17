package org.firstinspires.ftc.teamcode.opmodes.auto.presets;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.opmodes.auto.BaseAuto;

import dev.nextftc.robot.opmode.NextAutonomous;

@NextAutonomous(name = "Red Pit Start | Garden + PitFlower + AudFlower + PitScoop")
public class RedPit_Garden_PitFlower_AudFlower_PitScoop extends BaseAuto {
    public RedPit_Garden_PitFlower_AudFlower_PitScoop(Robot robot) {
        super(robot, true, false, true, true, true, true, false);
    }
}
