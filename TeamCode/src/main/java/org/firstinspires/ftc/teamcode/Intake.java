package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    HardwareMap hardwareMap;
    Gamepad gamepad;
    DcMotor intake_motor;
    double intakePower;

    private int counter = 0;

    public Intake(Gamepad gamepad, double intakePower) {
        //Intake
        intake_motor = hardwareMap.get(DcMotor.class, "intake_motor");

        this.intakePower = intakePower;
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
        intake_motor.setPower(intakePower);
    }

    public void finish() {
        intake_motor.setPower(0);
    }

}
