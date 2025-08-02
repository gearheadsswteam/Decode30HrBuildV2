package org.firstinspires.ftc.teamcode.fsm.statemachine;

//statemachine;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.fsm.commands.StateAwareTimedCommand;
import org.firstinspires.ftc.teamcode.fsm.subsystems.DeliverySubsystem;
import org.firstinspires.ftc.teamcode.fsm.subsystems.IntakeSubsystem;

/**
 * Utility class for building a finite state machine (FSM)
 * consisting of timed robot actions involving intake and delivery subsystems.
 */
public class StateBuilder {

    /**
     * Constructs a looping FSM with four states:
     * <ul>
     *     <li>State1: Intake forward for 1 second</li>
     *     <li>State2: Deliver forward + open servo for 2 seconds</li>
     *     <li>State3: Intake reverse for 2.5 seconds</li>
     *     <li>State4: Deliver reverse + open servo for 3 seconds</li>
     * </ul>
     * Transitions are manually triggered and loop cyclically.
     *
     * @param telemetry telemetry instance for displaying state updates
     * @param intake intake subsystem
     * @param delivery delivery subsystem
     * @return a configured RobotStateMachine instance
     */
    public static RobotStateMachine buildLoopingFSM(Telemetry telemetry, IntakeSubsystem intake, DeliverySubsystem delivery) {
        RobotStateMachine fsm = new RobotStateMachine();

        fsm.addState(new RobotState("State1", new StateAwareTimedCommand(telemetry, "State1", 1000) {
            @Override public void init() { super.init(); intake.intake(1.0); }
            @Override public void execute() { super.execute(); }
            @Override public void end() { intake.stop(); super.end(); }
        }));

        fsm.addState(new RobotState("State2", new StateAwareTimedCommand(telemetry,"State2", 2000) {
            @Override public void init() {
                super.init();
                delivery.deliver(0.5);
                delivery.openServo();
            }
            @Override public void execute() { super.execute(); }
            @Override public void end() {
                delivery.stop();
                delivery.closeServo();
                super.end();
            }
        }));

        fsm.addState(new RobotState("State3", new StateAwareTimedCommand(telemetry,"State3", 2500) {
            @Override public void init() { super.init(); intake.intake(-1.0); }
            @Override public void execute() { super.execute(); }
            @Override public void end() { intake.stop(); super.end(); }
        }));

        fsm.addState(new RobotState("State4", new StateAwareTimedCommand(telemetry,"State4", 3000) {
            @Override public void init() {
                super.init();
                delivery.deliver(-0.5);
                delivery.openServo();
            }
            @Override public void execute() { super.execute(); }
            @Override public void end() {
                delivery.stop();
                delivery.closeServo();
                super.end();
            }
        }));

        return fsm;
    }
}
