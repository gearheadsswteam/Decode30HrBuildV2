package org.firstinspires.ftc.teamcode.fsm.statemachine;

import org.firstinspires.ftc.teamcode.fsm.commands.Command;

/**
 * A RobotState defines a named action to be run by the FSM.
 */
public class RobotState {
    public final String name;
    public final Command command;

    public RobotState(String name, Command command) {
        this.name = name;
        this.command = command;
    }
}
