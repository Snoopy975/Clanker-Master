package org.firstinspires.ftc.teamcode.opmodes.auto.presets;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.opmodes.auto.BaseAuto;

import dev.nextftc.robot.opmode.NextAutonomous;

@NextAutonomous(name = "Red Pit Start | PitFlower + AudFlower + AudScoop")
public class RedPit_PitFlower_AudFlower_AudScoop extends BaseAuto {
    public RedPit_PitFlower_AudFlower_AudScoop(Robot robot) {
        super(robot, true, false, false, true, true, false, true);
    }
}
