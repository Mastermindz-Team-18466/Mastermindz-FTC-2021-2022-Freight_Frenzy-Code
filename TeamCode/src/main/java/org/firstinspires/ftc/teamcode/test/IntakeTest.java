package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp (name = "Intake", group = "Test")

public class IntakeTest extends LinearOpMode {
    DcMotor intake_motor;
    double intakePower = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        intake_motor = hardwareMap.get(DcMotor.class, "intake_motor");

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.right_bumper) {
                intake_motor.setPower(intakePower);
            }
            else {
                intake_motor.setPower(0);
            }
        }
    }
}
