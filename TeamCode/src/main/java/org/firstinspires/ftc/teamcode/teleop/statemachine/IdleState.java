package org.firstinspires.ftc.teamcode.teleop.statemachine;

import org.firstinspires.ftc.teamcode.GHRobot;

public class IdleState extends RobotState {
    @Override
    public String getName() {
        return "IDLE";
    }

    @Override
    public void onEnter(GHRobot robot) {
//        robot.intakeMotor.setPower(0);
//        robot.shooterMotor.setPower(0);
//        robot.armServo.setPosition(0.0);
    }
}