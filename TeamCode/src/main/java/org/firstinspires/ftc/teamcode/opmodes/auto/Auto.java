package org.firstinspires.ftc.teamcode.opmodes.auto;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "AutoPath", group = "Autonomous")
public class Auto extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56, 7, 90);
    private final Pose curveforgarden = poseFactory.of(36, 6.815, 180);
    private final Pose curveforgardenControl1 = poseFactory.of(46.5, 22, 0);
    private final Pose garden = poseFactory.of(7.2, 7, 180);
    private final Pose shootStart = poseFactory.of(7.2, 7, 180);
    private final Pose shoot = poseFactory.of(57, 134, 270);
    private final Pose drivetoflowerStart = poseFactory.of(57, 134, 270);
    private final Pose drivetoflower = poseFactory.of(21, 46, 180);
    private final Pose flower = poseFactory.of(11, 46, 180);
    private final Pose shoot2Start = poseFactory.of(11, 46, 180);
    private final Pose shoot2 = poseFactory.of(57, 134, 270);
    private final Pose drivetoflower2Start = poseFactory.of(57, 134, 270);
    private final Pose drivetoflower2 = poseFactory.of(47, 121, 90);
    private final Pose flower2 = poseFactory.of(47, 129, 90);
    private final Pose shoot3Start = poseFactory.of(47, 129, 90);
    private final Pose shoot3 = poseFactory.of(57, 134, 270);
    private final Pose shoot3Control1 = poseFactory.of(72.7275, 86.5953, 0);
    private final Pose parkStart = poseFactory.of(57, 134, 270);
    private final Pose park = poseFactory.of(8.4356, 91.8094, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, curveforgarden()),
                follow(follower, garden()),
                follow(follower, shoot()),
                follow(follower, drivetoflower()),
                follow(follower, flower()),
                follow(follower, shoot2()),
                follow(follower, drivetoflower2()),
                follow(follower, flower2()),
                follow(follower, shoot3()),
                follow(follower, park())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }

    public Path curveforgarden() {
        return Paths.curve(start, curveforgardenControl1, curveforgarden).linear(start, curveforgarden);
    }

    public Path garden() {
        return Paths.line(curveforgarden, garden).constant(garden);
    }

    public Path shoot() {
        return Paths.line(shootStart, shoot).linear(shootStart, shoot);
    }

    public Path drivetoflower() {
        return Paths.line(drivetoflowerStart, drivetoflower).linear(drivetoflowerStart, drivetoflower);
    }

    public Path flower() {
        return Paths.line(drivetoflower, flower).constant(flower);
    }

    public Path shoot2() {
        return Paths.line(shoot2Start, shoot2).linear(shoot2Start, shoot2);
    }

    public Path drivetoflower2() {
        return Paths.line(drivetoflower2Start, drivetoflower2).linear(drivetoflower2Start, drivetoflower2);
    }

    public Path flower2() {
        return Paths.line(drivetoflower2, flower2).constant(flower2);
    }

    public Path shoot3() {
        return Paths.curve(shoot3Start, shoot3Control1, shoot3).linear(shoot3Start, shoot3);
    }

    public Path park() {
        return Paths.line(parkStart, park).linear(parkStart, park);
    }
}
