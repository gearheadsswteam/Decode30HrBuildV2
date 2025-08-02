
package org.firstinspires.ftc.teamcode.fsm.subsystems;

///robot/subsystems;

import org.firstinspires.ftc.teamcode.fsm.hardware.GHRobot;

public class IntakeSubsystem {
    private final GHRobot robot;

    public IntakeSubsystem(GHRobot robot) {
        this.robot = robot;
    }

    public void intake(double power) {
        //robot.intakeMotor.setPower(power);
    }

    public void stop() {
        //robot.intakeMotor.setPower(0.0);
    }
}
