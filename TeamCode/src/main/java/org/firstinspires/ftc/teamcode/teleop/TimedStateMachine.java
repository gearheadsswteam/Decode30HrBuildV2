package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

public class TimedStateMachine {

    /**
     * Represents a single step in the state machine.
     * Each step has optional start/loop/stop actions,
     * a max duration in milliseconds, and an optional condition to end early.
     */
    public static class Step {
        public Runnable onStart = () -> {
        };              // Action to run once when the step begins
        public Runnable onLoop = () -> {
        };              // Action to run repeatedly while in this step
        public Runnable onStop = () -> {
        };              // Action to run once when the step ends
        public double maxDurationMS = 0.0;               // Max time allowed for this step (0 = unlimited)
        public BooleanSupplier doneWhen = () -> false;   // Condition to end the step early

        // Builder-style setters so steps can be chained neatly
        public Step onStart(Runnable r) {
            this.onStart = (r != null ? r : this.onStart);
            return this;
        }

        public Step onLoop(Runnable r) {
            this.onLoop = (r != null ? r : this.onLoop);
            return this;
        }

        public Step onStop(Runnable r) {
            this.onStop = (r != null ? r : this.onStop);
            return this;
        }

        public Step maxDurationMS(double ms) {
            this.maxDurationMS = Math.max(0, ms);
            return this;
        }

        public Step doneWhen(BooleanSupplier c) {
            this.doneWhen = (c != null ? c : this.doneWhen);
            return this;
        }
    }

    // List of steps that make up the state machine
    private final List<Step> steps = new ArrayList<>();
    // Timer used to track how long the current step has been active
    private final ElapsedTime timer = new ElapsedTime();

    // Index of the current step
    private int idx = 0;
    // Whether the FSM is running or idle
    private boolean running = false;

    // Flag: true when a step has finished and we are waiting for driver input to advance
    private boolean awaitingAdvance = false;

    /**
     * Add a step to the state machine
     */
    public TimedStateMachine add(Step s) {
        steps.add(s);
        return this;
    }

    /**
     * Clear all steps and reset state
     */
    public TimedStateMachine clear() {
        steps.clear();
        idx = 0;
        running = false;
        awaitingAdvance = false;
        return this;
    }

    /**
     * Start execution from step 0 (only works if FSM is idle and has steps).
     */
    public void start() {
        if (steps.isEmpty() || running) return;
        idx = 0;
        running = true;
        awaitingAdvance = false;
        timer.reset();
        steps.get(0).onStart.run(); // trigger first step’s onStart
    }

    /**
     * Cancel immediately. Calls onStop() for the current step if running.
     */
    public void cancel() {
        if (!running) return;
        steps.get(idx).onStop.run();
        running = false;
        awaitingAdvance = false;
    }

    /**
     * Call this continuously in your OpMode loop.
     * Runs onLoop() for the current step.
     * If the step completes (time or condition), marks it finished and waits for advance().
     */
    public void update() {
        if (!running) return;

        if (awaitingAdvance) {
            // Current step is done; waiting for driver press (advance) to move on
            return;
        }

        Step s = steps.get(idx);
        s.onLoop.run(); // repeat action while step is active

        // Check if step is complete by timeout or condition
        boolean timeUp = s.maxDurationMS > 0 && timer.milliseconds() >= s.maxDurationMS;
        if (s.doneWhen.getAsBoolean() || timeUp) {
            s.onStop.run();       // end step
            awaitingAdvance = true; // now wait for driver to advance
        }
    }

    /**
     * Advance to the next step, but only if the current step has finished.
     * Called when the driver presses X (or another bound button).
     */
    public void advance() {
        if (!running || !awaitingAdvance) return; // ignore presses while a step is still active

        idx++;
        if (idx >= steps.size()) {
            // Reached the end of the step list
            running = false;
            awaitingAdvance = false;
        } else {
            // Move into the next step
            awaitingAdvance = false;
            timer.reset();
            steps.get(idx).onStart.run();
        }
    }

    /**
     * Convenience wrapper for button presses.
     * If FSM is idle, start from step 0.
     * If FSM is waiting for advance, go to next step.
     * Otherwise (step is still running), ignore the press.
     */
    public void onTrigger() {
        if (!running) {
            start();          // if idle/finished -> start from step 0
        } else if (awaitingAdvance) {
            advance();        // only advance if current step has finished
        }                      // else ignore press while step is still running
    }

    // -------- Getters for telemetry/debug --------
    public boolean isRunning() {
        return running;
    }

    public boolean isAwaitingAdvance() {
        return awaitingAdvance;
    }  // true if waiting for driver to press X

    public int currentIndex() {
        return running ? idx : -1;
    }        // -1 means idle/finished

    public int size() {
        return steps.size();
    }                      // total number of steps

    public double stateTime() {
        return timer.seconds();
    }           // elapsed time in current step
}
