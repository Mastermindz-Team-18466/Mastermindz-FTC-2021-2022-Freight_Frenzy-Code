package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    HardwareMap hardwareMap;
    Servo claw;
    Gamepad gamepad;

    public Claw(Gamepad gamepad) {
        this.gamepad = gamepad;

        claw = hardwareMap.get(Servo.class, "claw");
    }

    public void control() {
        if (gamepad.x) {
            claw.setDirection(Servo.Direction.FORWARD);
            claw.setPosition(1);
        } if (gamepad.b) {
            claw.setDirection(Servo.Direction.REVERSE);
            claw.setPosition(0.2);
        }
    }
}