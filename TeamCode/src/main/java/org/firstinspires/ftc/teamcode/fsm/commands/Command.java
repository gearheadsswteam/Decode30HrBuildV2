package org.firstinspires.ftc.teamcode.fsm.commands;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Abstract base class representing a Command in the command-based architecture.
 *
 * Subclasses must implement lifecycle methods to define custom robot actions.
 * Commands can be used in FSMs or run independently in TeleOp or Autonomous.
 */
public abstract class Command {
    /**
     * Optional telemetry reference for status output and debugging.
     */
    public Telemetry telemetry;

    /**
     * Called once when the command starts. Use this to initialize state.
     */
    public abstract void init();

    /**
     * Called repeatedly while the command is active. Define main behavior here.
     */
    public abstract void execute();

    /**
     * Determines if the command has completed.
     *
     * @return true if the command is complete, false otherwise
     */
    public abstract boolean isFinished();

    /**
     * Called once when the command ends, either naturally or via interruption.
     * Use this to clean up resources or stop motors.
     */
    public abstract void end();
}
