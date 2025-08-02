package org.firstinspires.ftc.teamcode.fsm.commands;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.fsm.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.fsm.subsystems.ElevatorSubsystemWithMotionProf;

/**
 * Command to move the elevator to a target position.
 */
public class ElevatorCommandWithMotionProf extends Command {
    private final ElevatorSubsystemWithMotionProf elevator;
    private final Level targetLevel;

    public ElevatorCommandWithMotionProf(Telemetry telemetry, ElevatorSubsystemWithMotionProf elevator, Level level) {
        this.telemetry = telemetry;
        this.elevator = elevator;
        this.targetLevel = level;
    }

    @Override
    public void init() {
        switch (targetLevel) {
            case LOW:
                elevator.goTo(ElevatorSubsystemWithMotionProf.Level.LOW);
                break;
            case MID:
                elevator.goTo(ElevatorSubsystemWithMotionProf.Level.MID);
                break;
            case HIGH:
                elevator.goTo(ElevatorSubsystemWithMotionProf.Level.HIGH);
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
