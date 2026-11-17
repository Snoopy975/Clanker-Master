package org.firstinspires.ftc.teamcode.opmodes.auto.presets;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.opmodes.auto.BaseAuto;

import dev.nextftc.robot.opmode.NextAutonomous;

@NextAutonomous(name = "Blue Pit Start | AudScoop")
public class BluePit_AudScoop extends BaseAuto {
    public BluePit_AudScoop(Robot robot) {
        super(robot, false, false, false, false, false, false, true);
    }
}
