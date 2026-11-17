package org.firstinspires.ftc.teamcode.opmodes.auto.presets;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.opmodes.auto.BaseAuto;

import dev.nextftc.robot.opmode.NextAutonomous;

@NextAutonomous(name = "Blue Audience Start | AudFlower + Scoops")
public class BlueAud_AudFlower extends BaseAuto {
    public BlueAud_AudFlower(Robot robot) {
        super(robot, false, true, false, false, true);
    }
}
