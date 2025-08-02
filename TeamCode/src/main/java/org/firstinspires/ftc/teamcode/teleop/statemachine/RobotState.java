package org.firstinspires.ftc.teamcode.teleop.statemachine;

import org.firstinspires.ftc.teamcode.GHRobot;

public abstract class RobotState {
    public abstract String getName();
    public abstract void onEnter(GHRobot robot);
}