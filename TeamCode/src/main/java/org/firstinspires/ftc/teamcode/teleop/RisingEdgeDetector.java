package org.firstinspires.ftc.teamcode.teleop;

import java.util.HashMap;
import java.util.Map;

public class RisingEdgeDetector {
    private final Map<String, Boolean> lastStates = new HashMap<>();

    /**
     * Detects rising edge (false → true) for a named signal.
     * @param name Unique name of the button or signal (e.g., "a", "b", "dpad_up")
     * @param currentState Current boolean state of the signal
     * @return true if rising edge occurred
     */
    public boolean isRisingEdge(String name, boolean currentState) {
        boolean lastState = lastStates.getOrDefault(name, false);
        boolean risingEdge = !lastState && currentState;
        lastStates.put(name, currentState);
        return risingEdge;
    }

    /**
     * Optional: Reset all tracked states
     */
    public void reset() {
        lastStates.clear();
    }
}
