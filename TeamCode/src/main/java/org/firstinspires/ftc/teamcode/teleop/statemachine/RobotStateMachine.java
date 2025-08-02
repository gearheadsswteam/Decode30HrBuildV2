package org.firstinspires.ftc.teamcode.teleop.statemachine;

import org.firstinspires.ftc.teamcode.GHRobot;

import java.util.HashMap;
import java.util.Map;

public class RobotStateMachine {
    private RobotState currentState = GHRobotStates.IDLE;
    private final Map<RobotState, RobotState> transitionTable = new HashMap<>();
    private final Map<RobotState, Long> transitionDurations = new HashMap<>();
    private long stateStartTime;
    private boolean pendingTransition = false;
    private GHRobot robot;

    public RobotStateMachine(GHRobot robot) {
        // Define state transitions
        transitionTable.put(GHRobotStates.IDLE, GHRobotStates.INTAKE);
        transitionTable.put(GHRobotStates.INTAKE, GHRobotStates.SHOOT);
        transitionTable.put(GHRobotStates.SHOOT, GHRobotStates.ENDGAME);
        transitionTable.put(GHRobotStates.ENDGAME, GHRobotStates.IDLE);

        // Define durations in milliseconds
        transitionDurations.put(GHRobotStates.IDLE, 0L);
        transitionDurations.put(GHRobotStates.INTAKE, 3000L);
        transitionDurations.put(GHRobotStates.SHOOT, 2000L);
        transitionDurations.put(GHRobotStates.ENDGAME, 4000L);

        stateStartTime = System.currentTimeMillis();
        this.robot = robot;
    }

    public RobotState getCurrentState() {
        return currentState;
    }

    public long getElapsedTime() {
        return System.currentTimeMillis() - stateStartTime;
    }

    public long getRequiredDuration() {
        return transitionDurations.getOrDefault(currentState, 0L);
    }

    public boolean isTransitionComplete() {
        return getElapsedTime() >= getRequiredDuration();
    }

    // Called on button press
    public void requestAdvance() {
        pendingTransition = true;
    }

    // Call this periodically in loop
    public boolean update() {
        if (pendingTransition && isTransitionComplete()) {
            currentState = transitionTable.getOrDefault(currentState, GHRobotStates.IDLE);
            stateStartTime = System.currentTimeMillis();

            //TODO = add Robot
            currentState.onEnter(robot);
            pendingTransition = false;
            return true;
        }
        return false;
    }
}
