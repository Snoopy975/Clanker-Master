package org.firstinspires.ftc.teamcode.opmodes.auto.presets;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.opmodes.auto.BaseAuto;

import dev.nextftc.robot.opmode.NextAutonomous;

@NextAutonomous(name = "Blue Pit Start | AudFlower + Scoops", group = "Blue Pit Start")
public class BluePit_AudFlower extends BaseAuto {
    public BluePit_AudFlower(Robot robot) {
        super(robot, false, false, false, false, true);
    }
}
