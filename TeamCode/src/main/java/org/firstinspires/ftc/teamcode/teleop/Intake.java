package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    HardwareMap hardwareMap;
    Gamepad gamepad;
    DcMotor intake_motor;
    double intakePower;


    public Intake(Gamepad gamepad, double intakePower) {
        //Intake
        intake_motor = hardwareMap.get(DcMotor.class, "intake_motor");

        this.intakePower = intakePower;
        this.gamepad = gamepad;
    }

    public void control() {
        if (gamepad.right_bumper) {
            start();
        }
        else {
            stop();
        }
    }

    public void start() {
        intake_motor.setPower(intakePower);
    }

    public void stop() {
        intake_motor.setPower(0);
    }

}
