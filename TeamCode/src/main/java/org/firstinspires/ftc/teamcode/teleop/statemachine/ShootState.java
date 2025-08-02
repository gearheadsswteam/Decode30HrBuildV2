package org.firstinspires.ftc.teamcode.teleop.statemachine;

import org.firstinspires.ftc.teamcode.GHRobot;

public class ShootState extends RobotState {
    @Override
    public String getName() {
        return "SHOOT";
    }

    @Override
    public void onEnter(GHRobot robot) {
//        robot.shooterMotor.setPower(1.0);
//        robot.armServo.setPosition(0.6);
    }
}