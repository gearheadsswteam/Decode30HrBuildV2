package org.firstinspires.ftc.teamcode.fsm.commands;

import java.util.ArrayList;
import java.util.List;

/**
 * A simple runner for executing commands manually (outside FSM),
 * such as in TeleOp (button-triggered) or Autonomous (sequential).
 */
public class RobotCommandRunner {
    private final List<Command> activeCommands = new ArrayList<>();

    /**
     * Schedules a command to run.
     * The command's init() is called immediately.
     */
    public void schedule(Command command) {
        command.init();
        activeCommands.add(command);
    }

    /**
     * Must be called every loop to drive command lifecycle.
     * It calls execute(), checks isFinished(), and then calls end().
     */
    public void update() {
        List<Command> finished = new ArrayList<>();
        for (Command command : activeCommands) {
            command.execute();
            if (command.isFinished()) {
                command.end();
                finished.add(command);
            }
        }
        activeCommands.removeAll(finished);
    }

    /**
     * Indicates if any commands are still running.
     */
    public boolean isBusy() {
        return !activeCommands.isEmpty();
    }
}
