package org.firstinspires.ftc.teamcode.opmodes.auto;

/**
 * Pre-match toggles for the autonomous. Once an auto is INITIALIZED nothing may be touched, so set these
 * first with the "Auto Setup" OpMode, press STOP, then pick the auto. They stay until the app restarts.
 * The order is always: garden, pit flower, audience flower, pit scoop, audience scoop. A disabled item is
 * skipped and the next enabled one runs, so scooping always comes last.
 */
public final class AutoOptions {
    private AutoOptions() { }

    public static boolean garden = true;
    public static boolean flowerPit = true;
    public static boolean flowerAudience = true;
    public static boolean scoopPit = true;
    public static boolean scoopAudience = true;

    public static String summary() {
        return "garden=" + yn(garden) + " flowerPit=" + yn(flowerPit) + " flowerAud=" + yn(flowerAudience)
                + " scoopPit=" + yn(scoopPit) + " scoopAud=" + yn(scoopAudience);
    }

    private static String yn(boolean b) { return b ? "YES" : "no"; }
}
