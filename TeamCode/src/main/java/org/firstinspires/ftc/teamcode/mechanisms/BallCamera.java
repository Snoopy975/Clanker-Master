package org.firstinspires.ftc.teamcode.mechanisms;

import android.util.Size;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

import java.util.List;

import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.robot.Mechanism;

/**
 * Second camera (a webcam aimed at the floor) that finds POLLEN (yellow balls) so the auto can hunt
 * loose ones. Separate from the Limelight, which stays on the hive tags for the turret.
 * Uses the FTC SDK's ColorBlobLocatorProcessor, so no extra library is needed.
 */
public class BallCamera implements Mechanism {
    public static String CAMERA_NAME = "Webcam 1";
    public static int WIDTH = 320, HEIGHT = 240;
    public static double HFOV_DEG = 60;             // horizontal field of view of your webcam (check its spec)
    public static double MIN_AREA = 80, MAX_AREA = 20000;
    public static double MIN_CIRCULARITY = 0.6;
    public static double STALE_S = 0.25;            // a ball counts as lost after this long unseen

    private VisionPortal portal;
    private ColorBlobLocatorProcessor locator;

    private boolean seen = false;
    private double bearingDeg = 0, radiusPx = 0;
    private long lastSeenMs = 0;

    @Override
    public void initialize() {
        locator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.YELLOW)          // POLLEN is yellow
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())
                .setDrawContours(true)
                .setBlurSize(5)
                .build();
        try {
            portal = new VisionPortal.Builder()
                    .setCamera(ActiveOpMode.hardwareMap().get(WebcamName.class, CAMERA_NAME))
                    .setCameraResolution(new Size(WIDTH, HEIGHT))
                    .addProcessor(locator)
                    .enableLiveView(false)                       // R704.D: no Wi-Fi video stream
                    .build();
        } catch (Exception e) {
            portal = null;                                       // no webcam plugged in: auto skips hunting
            locator = null;
        }
    }

    /** The webcam was found and is running. */
    public boolean isAvailable() { return portal != null; }

    @Override
    public void periodic() {
        if (locator == null) return;
        List<ColorBlobLocatorProcessor.Blob> blobs = locator.getBlobs();
        ColorBlobLocatorProcessor.Util.filterByArea(MIN_AREA, MAX_AREA, blobs);
        ColorBlobLocatorProcessor.Util.filterByCircularity(MIN_CIRCULARITY, 1, blobs);

        ColorBlobLocatorProcessor.Blob best = null;
        for (ColorBlobLocatorProcessor.Blob b : blobs) {
            if (best == null || b.getCircle().getRadius() > best.getCircle().getRadius()) best = b; // biggest = closest
        }
        if (best != null) {
            // + = ball is to the right of the image centre
            bearingDeg = (best.getCircle().getX() - WIDTH / 2.0) / WIDTH * HFOV_DEG;
            radiusPx = best.getCircle().getRadius();
            lastSeenMs = System.currentTimeMillis();
        }
        seen = best != null || (System.currentTimeMillis() - lastSeenMs) < STALE_S * 1000 && lastSeenMs > 0;
    }

    /** A ball was seen within the last STALE_S seconds. */
    public boolean hasBall() { return seen; }

    /** Degrees to the ball: positive = to the right of the robot's heading. */
    public double getBearingDeg() { return bearingDeg; }

    /** Apparent radius in pixels (bigger = closer). */
    public double getRadiusPx() { return radiusPx; }

    public void close() { if (portal != null) portal.close(); }
}
