package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

public class Intake {
    Hardware hardware = new Hardware();
    Gamepad gamepad;

    private int counter = 0;

    public Intake(Gamepad gamepad, double intakePower) {
        hardware.init(hardware.hardwareMap);
        this.hardware.intakePower = intakePower;
        this.gamepad = gamepad;
    }

    public void control() {
        if (gamepad.right_bumper && counter == 0) {
            start();
            counter = 1;
        }

        if (gamepad.right_bumper && counter == 1) {
            finish();
            counter = 0;
        }
    }

    public void start() {
        hardware.intake_motor.setPower(hardware.intakePower);
    }

    public void finish() {
        hardware.intake_motor.setPower(0);
    }

}
