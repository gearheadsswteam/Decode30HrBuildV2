package org.firstinspires.ftc.teamcode.fsm.commands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Timed command with state name awareness and both console + dashboard telemetry logging.
 */
public abstract class StateAwareTimedCommand extends Command {
    private final long timeoutMillis;
    private final String stateName;
    private long startTime;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    public StateAwareTimedCommand(Telemetry telemetry, String stateName, long timeoutMillis) {
        this.stateName = stateName;
        this.timeoutMillis = timeoutMillis;
        this.telemetry = telemetry;
    }

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

    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() - startTime > timeoutMillis;
    }

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
