package org.firstinspires.ftc.teamcode.opmodes.auto;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.api.Paths.through;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import dev.nextftc.robot.Mechanism;

/**
 * Your paths. All the numbers below are your RED paths, unchanged. For BLUE the same paths are rotated 180
 * degrees about the field centre, and "pit side" / "audience side" always mean the physical side of the
 * field, so blue's pit side is your red audience-side path rotated (and the other way round).
 * Changes from your original file: the class takes (follower, red, field) so it can mirror for blue,
 * Paths.line/through call Pedro's Paths (this class has the same name), and the Command methods use
 * Commands.lazy so they actually follow the path.
 */
public class Paths implements Mechanism {

    private final Follower follower;
    private final boolean red;
    private final double field;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    public Paths(Follower follower, boolean red, double field) {
        this.follower = follower;
        this.red = red;
        this.field = field;
    }

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

    /** Red as built; blue rotated 180 degrees about the field centre. */
    private Pose t(Pose p) {
        if (red) return p;
        return poseFactory.of(field - p.x(), field - p.y(), Math.toDegrees(p.heading()) + 180);
    }

    // ---- start poses: the wall you start on (audience = bottom wall, pit = top wall) ----
    public Pose startPose(boolean audience) {
        return t(red == audience ? scoopaudiencesidestart : scooppitsidestart);
    }

    // ---- where each path begins (the robot drives here first) ----
    public Pose gardenStart() { return t(gardenstart); }
    public Pose flowerPitStart() { return t(red ? flowerpitsidestart : floweraudiencesidestart); }
    public Pose flowerAudienceStart() { return t(red ? floweraudiencesidestart : flowerpitsidestart); }
    public Pose scoopPitStart() { return t(red ? scooppitsidestart : scoopaudiencesidestart); }
    public Pose scoopAudienceStart() { return t(red ? scoopaudiencesidestart : scooppitsidestart); }

    public Path garden() {
        Pose s = t(gardenstart), e = t(garden);
        return line(s, e).constant(e);
    }

    public Path flowerpitside() {
        Pose s = t(red ? flowerpitsidestart : floweraudiencesidestart);
        Pose e = t(red ? flowerpitside : floweraudienceside);
        return line(s, e).constant(e);
    }
    public Path floweraudienceside() {
        Pose s = t(red ? floweraudiencesidestart : flowerpitsidestart);
        Pose e = t(red ? floweraudienceside : flowerpitside);
        return line(s, e).constant(e);
    }
    public Path scooppitside() {
        return red
                ? through(t(scooppitsidestart), t(scooppitsideThrough1), t(scooppitsideThrough2), t(scooppitside)).tangent()
                : through(t(scoopaudiencesidestart), t(scoopaudiencesideThrough1), t(scoopaudiencesideThrough2), t(scoopaudienceside)).tangent();
    }
    public Path scoopaudienceside() {
        return red
                ? through(t(scoopaudiencesidestart), t(scoopaudiencesideThrough1), t(scoopaudiencesideThrough2), t(scoopaudienceside)).tangent()
                : through(t(scooppitsidestart), t(scooppitsideThrough1), t(scooppitsideThrough2), t(scooppitside)).tangent();
    }

    public Command Scoop_Pit() { return Commands.lazy(() -> follow(follower, scooppitside())); }
    public Command Scoop_Audience() { return Commands.lazy(() -> follow(follower, scoopaudienceside())); }
    public Command Flower_Audience() { return Commands.lazy(() -> follow(follower, floweraudienceside())); }
    public Command Flower_Pit() { return Commands.lazy(() -> follow(follower, flowerpitside())); }
    public Command Garden() { return Commands.lazy(() -> follow(follower, garden())); }
}
