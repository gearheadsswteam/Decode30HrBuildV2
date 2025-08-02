package org.firstinspires.ftc.teamcode.fsm.statemachine;

import org.firstinspires.ftc.teamcode.fsm.commands.Command;
import java.util.ArrayList;
import java.util.List;

/**
 * FSM that:
 * 1. Executes each state for a prescribed timeout
 * 2. Ends command only after timeout
 * 3. Transitions to next state only if:
 *      - timeout is over
 *      - "a" was pressed AFTER timeout
 * 4. Sequence: 1 → 2 → 3 → 4 → 1 → ...
 */
public class RobotStateMachine {
    private final List<RobotState> states = new ArrayList<>();
    private int currentIndex = -1;
    private boolean commandEnded = false;
    private long commandEndTime = 0;
    private boolean transitionRequestedAfterTimeout = false;

    public void addState(RobotState state) {
        states.add(state);
    }

    public void triggerNext() {
        // Only allow transition request AFTER command has ended
        if (commandEnded) {
            transitionRequestedAfterTimeout = true;
        }
    }

    public void update() {
        if (states.isEmpty()) return;

        if (currentIndex == -1) {
            // Start FSM
            currentIndex = 0;
            states.get(currentIndex).command.init();
            commandEnded = false;
            commandEndTime = 0;
            transitionRequestedAfterTimeout = false;
        }

        RobotState current = states.get(currentIndex);
        Command command = current.command;

        if (!commandEnded) {
            command.execute();

            if (command.isFinished()) {
                command.end();
                commandEnded = true;
                commandEndTime = System.currentTimeMillis();
                transitionRequestedAfterTimeout = false;  // Clear early presses
            }
        }

        // Allow state transition only if:
        // (a) current command ended, and
        // (b) "a" was pressed AFTER command finished
        if (commandEnded && transitionRequestedAfterTimeout) {
            currentIndex = (currentIndex + 1) % states.size();
            states.get(currentIndex).command.init();
            commandEnded = false;
            commandEndTime = 0;
            transitionRequestedAfterTimeout = false;
        }
    }

    public String getCurrentStateName() {
        if (currentIndex >= 0 && currentIndex < states.size()) {
            return states.get(currentIndex).name;
        }
        return "None";
    }
}
