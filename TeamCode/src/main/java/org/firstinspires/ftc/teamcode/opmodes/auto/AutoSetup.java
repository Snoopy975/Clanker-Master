package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Run this BEFORE the match, then press STOP and select a "Side" auto.
 * gamepad1: A = garden, B = pit flower, X = audience flower. Scoops are always on.
 */
@TeleOp(name = "Auto Setup", group = "Setup")
public class AutoSetup extends LinearOpMode {
    @Override
    public void runOpMode() {
        boolean a = false, b = false, x = false;
        while (!isStopRequested()) {
            if (gamepad1.a && !a) AutoOptions.garden = !AutoOptions.garden;
            if (gamepad1.b && !b) AutoOptions.flowerPit = !AutoOptions.flowerPit;
            if (gamepad1.x && !x) AutoOptions.flowerAudience = !AutoOptions.flowerAudience;
            a = gamepad1.a; b = gamepad1.b; x = gamepad1.x;

            telemetry.addLine("A garden | B pit flower | X audience flower (scoops always on)");
            telemetry.addData("Garden", AutoOptions.garden ? "YES" : "no");
            telemetry.addData("Pit flower", AutoOptions.flowerPit ? "YES" : "no");
            telemetry.addData("Audience flower", AutoOptions.flowerAudience ? "YES" : "no");
            telemetry.addLine("Press STOP when done, then pick the auto.");
            telemetry.update();
        }
    }
}
