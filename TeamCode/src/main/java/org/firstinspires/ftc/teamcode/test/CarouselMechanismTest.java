package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class CarouselMechanismTest extends LinearOpMode {
    CRServo carouselServo1;
    CRServo carouselServo2;

    @Override
    public void runOpMode() throws InterruptedException {
        carouselServo1 = hardwareMap.crservo.get("carouselServo1");
        carouselServo2 = hardwareMap.crservo.get("carouselServo2");

        waitForStart();

        while(opModeIsActive()) {
            if (gamepad1.x) {
                carouselServo1.setPower(1);
                carouselServo2.setPower(-1);
            }
            else {
                carouselServo1.setPower(0);
                carouselServo2.setPower(0);
            }
        }
    }
}
