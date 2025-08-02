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
    /**
     * List of states in the FSM.
     */
    private final List<RobotState> states = new ArrayList<>();

    /**
     * Index of the currently active state.
     */
    private int currentIndex = -1;

    /**
     * Flag indicating if the current command has finished.
     */
    private boolean commandEnded = false;

    /**
     * Time (in ms) when the last command ended.
     */
    private long commandEndTime = 0;

    /**
     * Flag indicating if a transition was requested after the command ended.
     */
    private boolean transitionRequestedAfterTimeout = false;

    /**
     * Adds a state to the FSM sequence.
     *
     * @param state the RobotState to add
     */
    public void addState(RobotState state) {
        states.add(state);
    }

    /**
     * Requests a transition to the next state.
     * Only allowed if the current command has ended.
     */
    public void triggerNext() {
        // Only allow transition request AFTER command has ended
        if (commandEnded) {
            transitionRequestedAfterTimeout = true;
        }
    }

    /**
     * Updates the FSM.
     * Handles command execution, completion, and conditional state transitions.
     */
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

    /**
     * Returns the name of the currently active state.
     *
     * @return state name or "None" if not started
     */
    public String getCurrentStateName() {
        if (currentIndex >= 0 && currentIndex < states.size()) {
            return states.get(currentIndex).name;
        }
        return "None";
    }
}
