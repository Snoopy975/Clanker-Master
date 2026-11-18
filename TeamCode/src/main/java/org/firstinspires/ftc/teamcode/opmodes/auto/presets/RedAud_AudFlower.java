package org.firstinspires.ftc.teamcode.opmodes.auto.presets;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.opmodes.auto.BaseAuto;

import dev.nextftc.robot.opmode.NextAutonomous;

@NextAutonomous(name = "Red Audience Start | AudFlower + Scoops", group = "Red Audience Start")
public class RedAud_AudFlower extends BaseAuto {
    public RedAud_AudFlower(Robot robot) {
        super(robot, true, true, false, false, true);
    }
}
