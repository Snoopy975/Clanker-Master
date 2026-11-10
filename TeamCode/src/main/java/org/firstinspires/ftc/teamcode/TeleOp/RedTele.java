package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystem.Drive;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@TeleOp(name = "Tele", group = "TeleOp")
public class Tele extends NextFTCOpMode {

    public Tele() {
        addComponents(
                new SubsystemComponent(Drive.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    @Override
    public void onUpdate() {
        if (gamepad1.y) {
            Drive.INSTANCE.resetHeading();
        }

        // Hold left trigger for slow mode
        Drive.INSTANCE.setSlowMode(gamepad1.left_trigger > 0.5);

        Drive.INSTANCE.drive(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x
        );
    }
}