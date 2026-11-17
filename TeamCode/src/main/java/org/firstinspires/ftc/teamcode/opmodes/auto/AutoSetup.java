package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Run this BEFORE the match, then press STOP and select the auto.
 * gamepad1: A = garden, B = pit flower, X = audience flower, Y = pit scoop, right bumper = audience scoop.
 */
@TeleOp(name = "Auto Setup", group = "Setup")
public class AutoSetup extends LinearOpMode {
    @Override
    public void runOpMode() {
        boolean a = false, b = false, x = false, y = false, rb = false;
        while (!isStopRequested()) {
            if (gamepad1.a && !a) AutoOptions.garden = !AutoOptions.garden;
            if (gamepad1.b && !b) AutoOptions.flowerPit = !AutoOptions.flowerPit;
            if (gamepad1.x && !x) AutoOptions.flowerAudience = !AutoOptions.flowerAudience;
            if (gamepad1.y && !y) AutoOptions.scoopPit = !AutoOptions.scoopPit;
            if (gamepad1.right_bumper && !rb) AutoOptions.scoopAudience = !AutoOptions.scoopAudience;
            a = gamepad1.a; b = gamepad1.b; x = gamepad1.x; y = gamepad1.y; rb = gamepad1.right_bumper;

            telemetry.addLine("A garden | B pit flower | X audience flower | Y pit scoop | RB audience scoop");
            telemetry.addData("Garden", AutoOptions.garden ? "YES" : "no");
            telemetry.addData("Pit flower", AutoOptions.flowerPit ? "YES" : "no");
            telemetry.addData("Audience flower", AutoOptions.flowerAudience ? "YES" : "no");
            telemetry.addData("Pit scoop", AutoOptions.scoopPit ? "YES" : "no");
            telemetry.addData("Audience scoop", AutoOptions.scoopAudience ? "YES" : "no");
            telemetry.addLine("Press STOP when done, then pick the auto.");
            telemetry.update();
        }
    }
}
