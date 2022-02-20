package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Concept: ClawTest", group = "Test")
public class ClawTest extends LinearOpMode {
    Servo claw;

    @Override
    public void runOpMode() throws InterruptedException {
        claw = hardwareMap.get(Servo.class, "claw");

        waitForStart();

        while(opModeIsActive()) {
            if (gamepad1.x) {
                claw.setDirection(Servo.Direction.FORWARD);
                claw.setPosition(0.7);
            } if (gamepad1.b) {
                claw.setDirection(Servo.Direction.REVERSE);
                claw.setPosition(0.5);
            }
        }
    }
}
