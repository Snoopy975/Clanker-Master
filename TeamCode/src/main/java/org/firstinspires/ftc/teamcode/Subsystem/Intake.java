package org.firstinspires.ftc.teamcode.Subsystem;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

public class Intake implements Subsystem {

    public static final Intake INSTANCE = new Intake();
    private Intake() { }

    private MotorEx motor = new MotorEx("Intake").reversed();

    private ControlSystem controlSystem = ControlSystem.builder()
            .velPid(0.0025, 0 , 0)
            // .basicFF(1, 0, 0)  // feedforward constants
            .build();

    // When true, periodic() cuts motor power to 0 directly instead of using PID.
    // This prevents the velocity PID from actively braking (reversing) the motor
    // when transitioning from full speed to stop, which would spit out artifacts.
    private boolean stopped = false;

    // Tracks the current commanded direction for EMI compensation in the turret.
    // +1 = intaking, -1 = outtaking, 0 = stopped
    private int direction = 0;

    /**
     * Helper: builds a command that sets the intake state on start() and
     * never finishes on its own (runs until interrupted by another command).
     * All side effects happen at command START, not at creation time, so
     * bindings don't trigger them prematurely.
     */
    private Command makeCommand(double velocityGoal, int dir, boolean stop) {
        return new LambdaCommand()
                .setStart(() -> {
                    stopped = stop;
                    direction = dir;
                    if (stop) {
                        controlSystem.setGoal(new KineticState(0, 0, 0));
                        controlSystem.reset();
                    } else {
                        controlSystem.setGoal(new KineticState(0, velocityGoal, 0));
                    }
                })
                .setIsDone(() -> false)  // runs forever until interrupted
                .requires(this);
    }

    /** Spin intake in */
    public Command In() {
        return makeCommand(500, 1, false);
    }

    /** Spin intake out */
    public Command Out() {
        return makeCommand(-5000, -1, false);
    }

    /** Stop intake — cuts power immediately, no PID braking */
    public Command Stop() {
        return makeCommand(0, 0, true);
    }

    /**
     * Returns the current intake direction for EMI compensation.
     * +1 = intaking, -1 = outtaking, 0 = stopped.
     */
    public int getDirection() {
        return direction;
    }

    /** Get current motor velocity for stall detection */
    public double getVelocity() {
        return motor.getState().getVelocity();
    }

    @Override
    public void periodic() {
        if (stopped) {
            motor.setPower(0);
        } else {
            motor.setPower(controlSystem.calculate(motor.getState()));
        }
    }
}
