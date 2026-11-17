package org.firstinspires.ftc.teamcode.opmodes.auto;

/**
 * Pre-match choices for the autonomous, so the same auto can be tuned to what the alliance partner
 * is doing. Once an auto is INITIALIZED nothing may be touched, so set these first with the
 * "Auto Setup" OpMode (stop it, then pick the auto). The values live as long as the Robot Controller
 * app stays running; defaults below apply after a restart.
 */
public final class AutoOptions {
    private AutoOptions() { }

    /** Collect from the FLOWER(S) on our half after the preload. */
    public static boolean grabFlower = true;
    /** Collect from our GARDEN after the flower. */
    public static boolean grabGarden = true;
    /** After the preset pickups, hunt loose pollen with the webcam if there is time. */
    public static boolean huntLoose = true;

    public static String summary() {
        return "flower=" + (grabFlower ? "YES" : "no")
                + "  garden=" + (grabGarden ? "YES" : "no")
                + "  hunt=" + (huntLoose ? "YES" : "no");
    }
}
