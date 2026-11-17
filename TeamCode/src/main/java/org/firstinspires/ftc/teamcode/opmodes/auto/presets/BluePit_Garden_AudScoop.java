package org.firstinspires.ftc.teamcode.opmodes.auto.presets;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.opmodes.auto.BaseAuto;

import dev.nextftc.robot.opmode.NextAutonomous;

@NextAutonomous(name = "Blue Pit Start | Garden + AudScoop")
public class BluePit_Garden_AudScoop extends BaseAuto {
    public BluePit_Garden_AudScoop(Robot robot) {
        super(robot, false, false, true, false, false, false, true);
    }
}
