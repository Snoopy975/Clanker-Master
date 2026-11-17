package org.firstinspires.ftc.teamcode.opmodes.auto.presets;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.opmodes.auto.BaseAuto;

import dev.nextftc.robot.opmode.NextAutonomous;

@NextAutonomous(name = "Blue Audience Start | Garden + AudFlower + Scoops")
public class BlueAud_Garden_AudFlower extends BaseAuto {
    public BlueAud_Garden_AudFlower(Robot robot) {
        super(robot, false, true, true, false, true);
    }
}
