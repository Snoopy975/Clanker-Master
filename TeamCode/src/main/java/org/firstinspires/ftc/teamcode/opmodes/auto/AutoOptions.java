package org.firstinspires.ftc.teamcode.opmodes.auto;

/**
 * Pre-match toggles for the "Side" autos. Once an auto is INITIALIZED nothing may be touched, so set these
 * first with the "Auto Setup" OpMode, press STOP, then pick the auto. They stay until the app restarts.
 * Order: garden, pit flower, audience flower, then the scoops, which are always on (the up cell's AprilTag
 * decides which scoop path runs). A disabled item is skipped and the next one runs.
 */
public final class AutoOptions {
    private AutoOptions() { }

    public static boolean garden = true;
    public static boolean flowerPit = true;
    public static boolean flowerAudience = true;
}
