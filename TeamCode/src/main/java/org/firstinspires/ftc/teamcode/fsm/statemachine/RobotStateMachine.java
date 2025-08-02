package org.firstinspires.ftc.teamcode.fsm.statemachine;

import org.firstinspires.ftc.teamcode.fsm.commands.Command;

import java.util.ArrayList;
import java.util.List;

public class RobotStateMachine {
    private final List<RobotState> states = new ArrayList<>();
    private int currentIndex = -1;
    private boolean isRunning = false;

    public void addState(RobotState state) {
        states.add(state);
    }

    public void triggerNext() {
        // Finish previous
        if (currentIndex >= 0 && currentIndex < states.size()) {
            states.get(currentIndex).command.end();
        }

        currentIndex = (currentIndex + 1) % states.size(); // wrap around
        states.get(currentIndex).command.init();
        isRunning = true;
    }

    public void update() {
        if (!isRunning || currentIndex < 0 || currentIndex >= states.size()) return;

        RobotState current = states.get(currentIndex);
        current.command.execute();
    }

    public String getCurrentStateName() {
        if (currentIndex >= 0 && currentIndex < states.size()) {
            return states.get(currentIndex).name;
        }
        return "None";
    }
}
