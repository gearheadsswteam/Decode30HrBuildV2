package org.firstinspires.ftc.teamcode.fsm.input;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.HashMap;

/**
 * Detects rising edge button presses — true only on the frame
 * the button is pressed (not held).
 */
public class RisingEdgeDetector {
    private final HashMap<String, Boolean> lastState = new HashMap<>();

    /**
     * Returns true only on the rising edge (button just pressed).
     */
    public boolean isPressed(Gamepad gamepad, String buttonName) {
        boolean current = false;

        switch (buttonName) {
            case "a":
                current = gamepad.a;
                break;
            case "b":
                current = gamepad.b;
                break;
            case "x":
                current = gamepad.x;
                break;
            case "y":
                current = gamepad.y;
                break;
            case "dpad_up":
                current = gamepad.dpad_up;
                break;
            case "dpad_down":
                current = gamepad.dpad_down;
                break;
            case "dpad_left":
                current = gamepad.dpad_left;
                break;
            case "dpad_right":
                current = gamepad.dpad_right;
                break;
            case "left_bumper":
                current = gamepad.left_bumper;
                break;
            case "right_bumper":
                current = gamepad.right_bumper;
                break;
            case "start":
                current = gamepad.start;
                break;
            case "back":
                current = gamepad.back;
                break;
            default:
                current = false;
        }

        return current;
    }
}
