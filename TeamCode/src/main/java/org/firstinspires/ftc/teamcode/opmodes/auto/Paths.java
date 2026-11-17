package org.firstinspires.ftc.teamcode.opmodes.auto;

import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import dev.nextftc.robot.Mechanism;

public class Paths implements Mechanism {

    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose scooppitsidestart = poseFactory.of(6.0588, 133.5613, 90);
    private final Pose scooppitside = poseFactory.of(60.4784, 104.8411, 1.282);
    private final Pose scooppitsideThrough1 = poseFactory.of(57.2116, 120.6418, 0);
    private final Pose scooppitsideThrough2 = poseFactory.of(14.9021, 108.4316, 0);
    private final Pose scoopaudiencesidestart = poseFactory.of(8.5382, 9.5939, 90);
    private final Pose scoopaudienceside = poseFactory.of(63.312, 40.0238, -1.0932);
    private final Pose scoopaudiencesideThrough1 = poseFactory.of(58.4513, 19.3427, 0);
    private final Pose scoopaudiencesideThrough2 = poseFactory.of(15.2563, 34.0511, 0);
    private final Pose floweraudiencesidestart = poseFactory.of(38, 46, 180);
    private final Pose floweraudienceside = poseFactory.of(16, 46, 180);
    private final Pose flowerpitsidestart = poseFactory.of(46.5, 106, 90);
    private final Pose flowerpitside = poseFactory.of(46.5, 130, 90);
    private final Pose gardenstart = poseFactory.of(6.75, 30, -90);
    private final Pose garden = poseFactory.of(6.75, 6, -90);

    public Path garden() {
        return Paths.line(gardenstart, garden).constant(garden);
    }

    public Path flowerpitside() {
        return Paths.line(flowerpitsidestart, flowerpitside).constant(flowerpitside);
    }
    public Path floweraudienceside() {
        return Paths.line(floweraudiencesidestart, floweraudienceside).constant(floweraudienceside);
    }
    public Path scooppitside() {
        return Paths.through(scooppitsidestart, scooppitsideThrough1, scooppitsideThrough2, scooppitside).tangent();
    }
    public Path scoopaudienceside() {
        return Paths.through(scoopaudiencesidestart, scoopaudiencesideThrough1, scoopaudiencesideThrough2, scoopaudienceside).tangent();
    }

    public Command Scoop_Pit(){
        return instant(() -> follow(follower, scooppitside()));
    }
    public Command Scoop_Audience(){
        return instant(() -> follow(follower, scoopaudienceside()));
    }
    public Command Flower_Audience(){
        return instant(() -> follow(follower, floweraudienceside()));
    }
    public Command Flower_Pit(){
        return instant(() -> follow(follower, flowerpitside()));
    }
    public Command Garden(){
        return instant(() -> follow(follower, garden()));
    }

}
