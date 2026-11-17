package org.firstinspires.ftc.teamcode.opmodes.auto.presets;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.opmodes.auto.BaseAuto;

import dev.nextftc.robot.opmode.NextAutonomous;

@NextAutonomous(name = "Red Pit Start | PitFlower + AudFlower + PitScoop")
public class RedPit_PitFlower_AudFlower_PitScoop extends BaseAuto {
    public RedPit_PitFlower_AudFlower_PitScoop(Robot robot) {
        super(robot, true, false, false, true, true, true, false);
    }
}
