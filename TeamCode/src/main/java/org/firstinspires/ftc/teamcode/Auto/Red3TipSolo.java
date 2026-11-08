package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous(name = "Red3TipSolo", group = "Autonomous")
public class Red3TipSolo extends NextFTCOpMode {

    public Red3TipSolo() {
        addComponents(
                new SubsystemComponent(Intake.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56, 8, 90);
    private final Pose shoot = poseFactory.of(56, 12, 90);
    private final Pose runtopollen = poseFactory.of(9.4307, 17.2526, 270);
    private final Pose pickuppollen = poseFactory.of(9.4307, 8.2064, 270);
    private final Pose runtoflower = poseFactory.of(47.5037, 113.3182, 90);
    private final Pose runtoflowerControl1 = poseFactory.of(34.2191, 102.1222, 0);
    private final Pose pickupflower = poseFactory.of(47.7496, 128.4881, 90);
    private final Pose backupfromflower = poseFactory.of(47.5037, 113.3182, 90);
    private final Pose runtoflower2 = poseFactory.of(94.6125, 23.3882, 270);
    private final Pose runtoflower2Control1 = poseFactory.of(62.2459, 109.0939, 0);
    private final Pose runtoflower2Control2 = poseFactory.of(75.2444, 40.2116, 0);
    private final Pose pickupflower2 = poseFactory.of(94.5909, 12.968, 270);
    private final Pose backupfromflower2 = poseFactory.of(94.6125, 23.3882, 270);
    private final Pose park = poseFactory.of(9.1244, 97.3331, 0);
    private final Pose parkControl1 = poseFactory.of(11.6818, 29.342, 0);
    private final Pose parkControl2 = poseFactory.of(28.0343, 79.5574, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, shoot()),
                follow(follower, runtopollen()),
                follow(follower, pickuppollen()),
                follow(follower, runtoflower()),
                follow(follower, pickupflower()),
                follow(follower, backupfromflower()),
                follow(follower, runtoflower2()),
                follow(follower, pickupflower2()),
                follow(follower, backupfromflower2()),
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

    public Path shoot() {
        return Paths.line(start, shoot).constant(shoot);
    }

    public Path runtopollen() {
        return Paths.line(shoot, runtopollen).linear(shoot, runtopollen);
    }

    public Path pickuppollen() {
        return Paths.line(runtopollen, pickuppollen).constant(pickuppollen);
    }

    public Path runtoflower() {
        return Paths.curve(pickuppollen, runtoflowerControl1, runtoflower).linear(pickuppollen, runtoflower);
    }

    public Path pickupflower() {
        return Paths.line(runtoflower, pickupflower).constant(pickupflower);
    }

    public Path backupfromflower() {
        return Paths.line(pickupflower, backupfromflower).constant(backupfromflower);
    }

    public Path runtoflower2() {
        return Paths.curve(backupfromflower, runtoflower2Control1, runtoflower2Control2, runtoflower2).linear(backupfromflower, runtoflower2);
    }

    public Path pickupflower2() {
        return Paths.line(runtoflower2, pickupflower2).constant(pickupflower2);
    }

    public Path backupfromflower2() {
        return Paths.line(pickupflower2, backupfromflower2).constant(backupfromflower2);
    }

    public Path park() {
        return Paths.curve(backupfromflower2, parkControl1, parkControl2, park).linear(backupfromflower2, park);
    }
}