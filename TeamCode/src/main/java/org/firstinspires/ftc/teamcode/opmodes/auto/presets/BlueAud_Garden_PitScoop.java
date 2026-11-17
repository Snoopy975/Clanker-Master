package org.firstinspires.ftc.teamcode.opmodes.auto.presets;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.opmodes.auto.BaseAuto;

import dev.nextftc.robot.opmode.NextAutonomous;

@NextAutonomous(name = "Blue Audience Start | Garden + PitScoop")
public class BlueAud_Garden_PitScoop extends BaseAuto {
    public BlueAud_Garden_PitScoop(Robot robot) {
        super(robot, false, true, true, false, false, true, false);
    }
}
