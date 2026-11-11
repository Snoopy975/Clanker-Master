package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import org.firstinspires.ftc.teamcode.Robot;

import dev.nextftc.robot.opmode.BulkReadHook;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;

@NextTeleop(name = "RedTele", group = "TeleOp")
public class RedTele extends NextOpMode {

    private final Robot robot;

    public RedTele(Robot robot) {
        super(robot, BulkReadHook.INSTANCE);
        this.robot = robot;
        // v2 has no init(): hardwareMap lookups go in the constructor
        robot.drive.init(hardwareMap);
    }

    @Override
    public void start() {
        robot.drive.startDrive(gamepad1);
    }

    @Override
    public void periodic() {
        if (gamepad1.yWasPressed()) {
            robot.drive.resetHeading();
        }

        // Hold left trigger for slow mode
        robot.drive.setSlowMode(gamepad1.left_trigger > 0.5);

        // Example auto-align (set a real pose first, then uncomment):
        // if (gamepad1.bWasPressed()) robot.drive.goTo(new Pose(72, 72, Math.toRadians(90)));

        telemetry.addData("Pose", robot.drive.getPose());
    }
}