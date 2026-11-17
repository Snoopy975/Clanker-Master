package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Run this BEFORE the match (talk to your alliance partner first), then press STOP and select the auto.
 * gamepad1: A = flower, B = garden, X = hunt loose pollen. Choices stay until the app restarts.
 */
@TeleOp(name = "Auto Setup", group = "Setup")
public class AutoSetup extends LinearOpMode {
    @Override
    public void runOpMode() {
        boolean a = false, b = false, x = false;
        while (!isStopRequested()) {
            if (gamepad1.a && !a) AutoOptions.grabFlower = !AutoOptions.grabFlower;
            if (gamepad1.b && !b) AutoOptions.grabGarden = !AutoOptions.grabGarden;
            if (gamepad1.x && !x) AutoOptions.huntLoose = !AutoOptions.huntLoose;
            a = gamepad1.a; b = gamepad1.b; x = gamepad1.x;

            telemetry.addLine("A flower | B garden | X hunt loose pollen");
            telemetry.addData("Flower", AutoOptions.grabFlower ? "YES" : "no");
            telemetry.addData("Garden", AutoOptions.grabGarden ? "YES" : "no");
            telemetry.addData("Hunt", AutoOptions.huntLoose ? "YES" : "no");
            telemetry.addLine("Press STOP when done, then pick the auto.");
            telemetry.update();
        }
    }
}
