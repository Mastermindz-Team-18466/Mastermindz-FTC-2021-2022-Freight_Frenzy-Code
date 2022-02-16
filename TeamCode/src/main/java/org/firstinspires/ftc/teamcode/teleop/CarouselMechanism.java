package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class CarouselMechanism {
    HardwareMap hardwareMap;
    Gamepad gamepad;
    CRServo carouselServo1;
    CRServo carouselServo2;

    public CarouselMechanism(Gamepad gamepad){
        carouselServo1 = hardwareMap.crservo.get("carouselServo1");
        carouselServo2 = hardwareMap.crservo.get("carouselServo2");

        this.gamepad = gamepad;
    }

    public void control() {
        if (gamepad.x) {
            start();
        }
        else if (false) {
            stop();
        }
    }

    public void start() {
        carouselServo1.setPower(1);
        carouselServo2.setPower(-1);
    }

    public void stop() {
        carouselServo1.setPower(0);
        carouselServo2.setPower(0);
    }
}