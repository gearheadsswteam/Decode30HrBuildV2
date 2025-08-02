package org.firstinspires.ftc.teamcode.fsm.commands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Timed command with state name awareness and both console + dashboard telemetry logging.
 *
 * This abstract class represents a command that automatically completes
 * after a fixed timeout and logs lifecycle events with state identifiers.
 */
public abstract class StateAwareTimedCommand extends Command {
    /**
     * Timeout duration in milliseconds.
     */
    private final long timeoutMillis;

    /**
     * Name of the state this command represents (for logging).
     */
    private final String stateName;

    /**
     * System time in milliseconds when the command started.
     */
    private long startTime;

    /**
     * Dashboard instance used to send telemetry packets.
     */
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    /**
     * Constructs a timed command with state name and timeout duration.
     *
     * @param telemetry Telemetry instance for Driver Station logging
     * @param stateName Name of the state for tracking/logging
     * @param timeoutMillis Duration the command should run in milliseconds
     */
    public StateAwareTimedCommand(Telemetry telemetry, String stateName, long timeoutMillis) {
        this.stateName = stateName;
        this.timeoutMillis = timeoutMillis;
        this.telemetry = telemetry;
    }

    /**
     * Initializes the command. Logs "STARTED" status to console, telemetry, and dashboard.
     */
    @Override
    public void init() {
        startTime = System.currentTimeMillis();
        System.out.println("[FSM] STARTED: " + stateName + " | Timeout: " + timeoutMillis + "ms");

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("FSM Event", "STARTED");
        packet.put("State", stateName);
        packet.put("Timeout", timeoutMillis);
        dashboard.sendTelemetryPacket(packet);

        telemetry.addData("FSM Event", "STARTED");
        telemetry.addData("State", stateName);;
        telemetry.addData("Timeout", timeoutMillis);
        telemetry.update();
    }

    /**
     * Executes the command. Logs "EXECUTING" status to console, telemetry, and dashboard.
     */
    @Override
    public void execute() {
        System.out.println("[FSM] EXECUTING: " + stateName);

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("FSM Event", "EXECUTING");
        packet.put("State", stateName);
        packet.put("Elapsed", System.currentTimeMillis() - startTime);
        dashboard.sendTelemetryPacket(packet);

        telemetry.addData("FSM Event", "EXECUTING");
        telemetry.addData("State ", stateName);;
        telemetry.addData("Elapsed ", System.currentTimeMillis() - startTime);
        telemetry.update();
    }

    /**
     * Checks whether the command's timeout has elapsed.
     *
     * @return true if the command has timed out; false otherwise
     */
    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() - startTime > timeoutMillis;
    }

    /**
     * Called once after the command ends. Logs "ENDED" status and elapsed time.
     */
    @Override
    public void end() {
        long elapsed = System.currentTimeMillis() - startTime;
        System.out.println("[FSM] ENDED: " + stateName + " | Duration: " + elapsed + "ms");
        telemetry.addData("[FSM] ENDED: " + stateName ,  " | Duration: " + elapsed + "ms");
        telemetry.update();

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("FSM Event", "ENDED");
        packet.put("State", stateName);
        packet.put("Duration", elapsed);
        dashboard.sendTelemetryPacket(packet);

        telemetry.addData("FSM Event", "ENDED");
        telemetry.addData("State", stateName);
        telemetry.addData("Duration", elapsed);
        telemetry.update();
    }
}
