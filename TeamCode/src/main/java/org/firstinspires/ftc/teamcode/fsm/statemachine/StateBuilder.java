
package org.firstinspires.ftc.teamcode.fsm.statemachine;

//statemachine;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.fsm.commands.StateAwareTimedCommand;
import org.firstinspires.ftc.teamcode.fsm.subsystems.DeliverySubsystem;
import org.firstinspires.ftc.teamcode.fsm.subsystems.IntakeSubsystem;

public class StateBuilder {
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
