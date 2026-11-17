package org.firstinspires.ftc.teamcode.opmodes.auto.presets;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.opmodes.auto.BaseAuto;

import dev.nextftc.robot.opmode.NextAutonomous;

@NextAutonomous(name = "Blue Audience Start | Preload Only")
public class BlueAud_PreloadOnly extends BaseAuto {
    public BlueAud_PreloadOnly(Robot robot) {
        super(robot, false, true, false, false, false, false, false);
    }
}
