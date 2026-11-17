package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

/**
 * Your hand-built paths. The numbers below are the RED paths exactly as you made them. BLUE is the same
 * paths rotated 180 degrees about the field centre, so you never have to build them twice.
 *
 * "Audience" / "pit" are the physical start walls (audience = bottom wall, pit = top wall). The field is
 * rotationally symmetric, so blue's audience side is red's PIT paths rotated, and blue's pit side is red's
 * AUDIENCE paths rotated. {@code redData} below does that swap.
 */
public class AutoPaths {
    private final PoseFactory deg = PoseFactory.degrees();
    private final boolean red;
    private final boolean redData;      // true: use the red AUDIENCE-side numbers, false: the red PIT-side numbers
    private final double field;

    // ---- RED poses (from your Paths.java) ----
    private final Pose scooppitsidestart = deg.of(6.0588, 133.5613, 90);
    private final Pose scooppitside = deg.of(60.4784, 104.8411, 1.282);
    private final Pose scooppitsideThrough1 = deg.of(57.2116, 120.6418, 0);
    private final Pose scooppitsideThrough2 = deg.of(14.9021, 108.4316, 0);
    private final Pose scoopaudiencesidestart = deg.of(8.5382, 9.5939, 90);
    private final Pose scoopaudienceside = deg.of(63.312, 40.0238, -1.0932);
    private final Pose scoopaudiencesideThrough1 = deg.of(58.4513, 19.3427, 0);
    private final Pose scoopaudiencesideThrough2 = deg.of(15.2563, 34.0511, 0);
    private final Pose floweraudiencesidestart = deg.of(38, 46, 180);
    private final Pose floweraudienceside = deg.of(16, 46, 180);
    private final Pose flowerpitsidestart = deg.of(46.5, 106, 90);
    private final Pose flowerpitside = deg.of(46.5, 130, 90);
    private final Pose gardenstart = deg.of(6.75, 30, -90);
    private final Pose garden = deg.of(6.75, 6, -90);

    public AutoPaths(boolean red, boolean audience, double fieldSize) {
        this.red = red;
        this.redData = (red == audience);
        this.field = fieldSize;
    }

    /** Red as built; blue rotated 180 degrees about the field centre. */
    private Pose t(Pose p) {
        if (red) return p;
        return deg.of(field - p.x(), field - p.y(), Math.toDegrees(p.heading()) + 180);
    }

    // ---- where the robot starts, and where the scoop ends (the shoot spot in front of the hive) ----
    public Pose startPose() { return t(redData ? scoopaudiencesidestart : scooppitsidestart); }

    // ---- flower: the one on our side of the field ----
    public Pose flowerStart() { return t(redData ? floweraudiencesidestart : flowerpitsidestart); }
    public Path flower() {
        Pose s = flowerStart(), e = t(redData ? floweraudienceside : flowerpitside);
        return Paths.line(s, e).constant(e);
    }

    // ---- garden: only one per alliance (same for both start sides) ----
    public Pose gardenStart() { return t(gardenstart); }
    public Path garden() {
        Pose s = t(gardenstart), e = t(garden);
        return Paths.line(s, e).constant(e);
    }

    // ---- scoop: start wall, sweeping the alliance area, ending in front of the hive ----
    public Pose scoopStart() { return t(redData ? scoopaudiencesidestart : scooppitsidestart); }
    public Path scoop() {
        return redData
                ? Paths.through(t(scoopaudiencesidestart), t(scoopaudiencesideThrough1),
                        t(scoopaudiencesideThrough2), t(scoopaudienceside)).tangent()
                : Paths.through(t(scooppitsidestart), t(scooppitsideThrough1),
                        t(scooppitsideThrough2), t(scooppitside)).tangent();
    }
}
