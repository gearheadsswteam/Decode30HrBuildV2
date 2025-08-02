package org.firstinspires.ftc.teamcode.teleop.statemachine;

import org.firstinspires.ftc.teamcode.GHRobot;

public class EndgameState extends RobotState {
    @Override
    public String getName() {
        return "ENDGAME";
    }

    @Override
    public void onEnter(GHRobot robot) {
        //robot.armServo.setPosition(1.0);
    }
}
