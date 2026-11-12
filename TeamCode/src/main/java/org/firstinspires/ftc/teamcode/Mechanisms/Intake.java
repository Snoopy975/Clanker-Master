package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import dev.nextftc.robot.Mechanism;

/**
 * Intake that runs by default, with reverse and stop commands, and stops itself after
 * collecting MAX_COUNT pollen/nectar.
 *
 * States:
 *   RUNNING   - rollers pull in (default after init). Each game piece seen by the sensor is counted.
 *   REVERSING - rollers spit out. Count resets to 0.
 *   STOPPED   - rollers off (manual stop() or auto-stop at MAX_COUNT).
 *
 * Call init(hardwareMap) from the OpMode constructor.
 */
public class Intake implements Mechanism {

    private enum State { RUNNING, REVERSING, STOPPED }

    // ---- Tunables ----
    public static String MOTOR_NAME = "intake";
    public static String SENSOR_NAME = "intakeSensor";   // distance sensor looking at the piece path
    public static double INTAKE_POWER = 1.0;
    public static double REVERSE_POWER = -1.0;
    public static double DETECT_DISTANCE_MM = 40;        // closer than this = piece present
    public static int MAX_COUNT = 4;

    private DcMotorEx motor;
    private DistanceSensor sensor;
    private boolean ready = false;

    private State state = State.RUNNING;
    private int count = 0;
    private boolean pieceWasPresent = false;

    /** Call once from the OpMode constructor. */
    public void init(HardwareMap map) {
        motor = map.get(DcMotorEx.class, MOTOR_NAME);
        motor.setDirection(DcMotorSimple.Direction.FORWARD); // flip if it spins the wrong way
        sensor = map.get(DistanceSensor.class, SENSOR_NAME);
        state = State.RUNNING;
        count = 0;
        pieceWasPresent = false;
        ready = true;
    }

    // ---- Commands ----

    /** Run inward again (also re-arms after an auto-stop). Resets the count. */
    public void start() {
        count = 0;
        pieceWasPresent = false;
        state = State.RUNNING;
    }

    /** Run outward to eject. Resets the count. */
    public void reverse() {
        count = 0;
        state = State.REVERSING;
    }

    /** Stop the rollers. */
    public void stop() {
        state = State.STOPPED;
    }

    public int getCount() {
        return count;
    }

    public boolean isFull() {
        return count >= MAX_COUNT;
    }

    public String getState() {
        return state.name();
    }

    // ---- Loop ----

    @Override
    public void periodic() {
        if (!ready) return;

        switch (state) {
            case RUNNING:
                // Count on the rising edge so one piece is counted once, not every loop.
                boolean present = sensor.getDistance(DistanceUnit.MM) < DETECT_DISTANCE_MM;
                if (present && !pieceWasPresent) count++;
                pieceWasPresent = present;

                if (count >= MAX_COUNT) {
                    state = State.STOPPED;
                    motor.setPower(0);
                } else {
                    motor.setPower(INTAKE_POWER);
                }
                break;
            case REVERSING:
                motor.setPower(REVERSE_POWER);
                break;
            case STOPPED:
                motor.setPower(0);
                break;
        }
    }
}