package org.firstinspires.ftc.teamcode;

public class Intake {
    Hardware hardware = new Hardware();

    private int counter = 0;

    public void intake(double intakePower) {
        hardware.init(hardware.hardwareMap);
        this.hardware.intakePower = intakePower;

        if (hardware.gamepad.right_bumper && counter == 0) {
            start();
            counter = 1;
        }

        if (hardware.gamepad.right_bumper && counter == 1) {
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
