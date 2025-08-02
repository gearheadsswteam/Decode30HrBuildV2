package org.firstinspires.ftc.teamcode.fsm.commands;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.fsm.subsystems.ElevatorSubsystem;

/**
 * Command to move the elevator to a target position.
 */
public class ElevatorCommand extends Command {
    private final ElevatorSubsystem elevator;
    private final Level targetLevel;

    public ElevatorCommand(Telemetry telemetry, ElevatorSubsystem elevator, Level level) {
        this.telemetry = telemetry;
        this.elevator = elevator;
        this.targetLevel = level;
    }

    @Override
    public void init() {
        switch (targetLevel) {
            case LOW:
                elevator.goToLow();
                break;
            case MID:
                elevator.goToMid();
                break;
            case HIGH:
                elevator.goToHigh();
                break;
        }
    }

    @Override
    public void execute() {
        telemetry.addData("Elevator Target", targetLevel);
        telemetry.addData("At Target", elevator.isAtTarget());
        telemetry.update();
    }

    @Override
    public boolean isFinished() {
        return elevator.isAtTarget();
    }

    @Override
    public void end() {
        elevator.stop();
    }

    public enum Level {
        LOW, MID, HIGH
    }
}
