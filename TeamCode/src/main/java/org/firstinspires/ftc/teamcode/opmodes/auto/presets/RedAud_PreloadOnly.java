package org.firstinspires.ftc.teamcode.opmodes.auto.presets;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.opmodes.auto.BaseAuto;

import dev.nextftc.robot.opmode.NextAutonomous;

@NextAutonomous(name = "Red Audience Start | Preload Only")
public class RedAud_PreloadOnly extends BaseAuto {
    public RedAud_PreloadOnly(Robot robot) {
        super(robot, true, true, false, false, false, false, false);
    }
}
