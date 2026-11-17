package org.firstinspires.ftc.teamcode.opmodes.auto.presets;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.opmodes.auto.BaseAuto;

import dev.nextftc.robot.opmode.NextAutonomous;

@NextAutonomous(name = "Red Audience Start | PitScoop")
public class RedAud_PitScoop extends BaseAuto {
    public RedAud_PitScoop(Robot robot) {
        super(robot, true, true, false, false, false, true, false);
    }
}
