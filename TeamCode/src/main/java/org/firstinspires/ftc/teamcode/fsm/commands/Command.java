
package org.firstinspires.ftc.teamcode.fsm.commands;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class Command {
    public Telemetry telemetry;

    public abstract void init();

    public abstract void execute();

    public abstract boolean isFinished();

    public abstract void end();
}
