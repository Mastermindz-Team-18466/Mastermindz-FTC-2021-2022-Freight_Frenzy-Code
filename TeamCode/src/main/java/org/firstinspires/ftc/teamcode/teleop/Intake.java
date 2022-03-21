package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    Gamepad gamepad;
    DcMotor intake_motor;

    enum Switch {
        ON,
        OFF
    }


    public Intake(Gamepad gamepad, HardwareMap hardwareMap) {
        //Intake
        intake_motor = hardwareMap.get(DcMotor.class, "intake_motor");

        this.gamepad = gamepad;
    }

    public void control(Switch sw, double power) {
        if (sw == sw.ON) {
            start(power);
        }
        else if (sw == sw.OFF) {
            stop();
        }
    }

    public void start(double power) {
        intake_motor.setPower(power);
    }

    public void stop() {
        intake_motor.setPower(0);
    }

}
