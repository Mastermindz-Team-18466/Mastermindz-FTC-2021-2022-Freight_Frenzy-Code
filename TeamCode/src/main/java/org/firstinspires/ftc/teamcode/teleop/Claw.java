package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    HardwareMap hardwareMap;
    Servo claw;
    Gamepad gamepad;

    public enum State {
        OPEN,
        CLOSE
    }

    public Claw(Gamepad gamepad) {
        this.gamepad = gamepad;

        claw = hardwareMap.get(Servo.class, "claw");
    }

    public void control(State state) {
        if (state == State.OPEN) {
            claw.setDirection(Servo.Direction.FORWARD);
            claw.setPosition(0.7);
        } if (state == State.CLOSE) {
            claw.setDirection(Servo.Direction.REVERSE);
            claw.setPosition(0.5);
        }
    }
}