
package org.firstinspires.ftc.teamcode.fsm.subsystems;

//robot/subsystems;

import org.firstinspires.ftc.teamcode.fsm.hardware.GHRobot;

public class DeliverySubsystem {
    private final GHRobot robot;

    public DeliverySubsystem(GHRobot robot) {
        this.robot = robot;
    }

    public void deliver(double power) {
        //robot.deliveryMotor.setPower(power);
    }

    public void openServo() {
        //robot.deliveryServo.setPosition(1.0);
    }

    public void closeServo() {
        //robot.deliveryServo.setPosition(0.0);
    }

    public void stop() {
        //robot.deliveryMotor.setPower(0.0);
    }
}
