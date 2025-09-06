package org.firstinspires.ftc.teamcode.actionparts;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Class the represents the Intake System
 */
public class Intakesystem {
    //DC motor used by the intake system
    public DcMotor intakeMotor;

    /**
     * Constructor
     * @param intakeMotor
     */
    public Intakesystem(DcMotor intakeMotor) {
        this.intakeMotor = intakeMotor;
    }

    /**
     * Initialize the system
     */
    public void initialize(){
        //Add code
    }

    /**
     * Start the intake system
     */
    public void startInTake() {
        intakeMotor.setPower(-1);
    }

    /**
     * Stop the intake system
     */
    public void stopInTake() {
        intakeMotor.setPower(0);
    }

    /**
     * Start the intake system
     */
    public void startReverseInTake() {
        intakeMotor.setPower(1.0);
    }
}