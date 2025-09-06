package org.firstinspires.ftc.teamcode.actionparts;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {

    private Servo clawServo;

    // Tunable positions — adjust these for your claw hardware
    private static final double OPEN_POSITION = 0.8;
    private static final double CLOSED_POSITION = 0.2;

    // Track current state
    private boolean isOpen = false;

    public Claw(HardwareMap hardwareMap, String servoName) {
        clawServo = hardwareMap.get(Servo.class, servoName);
        close(); // default starting state
    }

    /** Opens the claw */
    public void open() {
        clawServo.setPosition(OPEN_POSITION);
        isOpen = true;
    }

    /** Closes the claw */
    public void close() {
        clawServo.setPosition(CLOSED_POSITION);
        isOpen = false;
    }

    /** Toggle claw state */
    public void toggle() {
        if (isOpen) {
            close();
        } else {
            open();
        }
    }

    /** Get the servo position (for telemetry/debug) */
    public double getPosition() {
        return clawServo.getPosition();
    }

    /** Returns true if the claw is open */
    public boolean isOpen() {
        return isOpen;
    }
}
