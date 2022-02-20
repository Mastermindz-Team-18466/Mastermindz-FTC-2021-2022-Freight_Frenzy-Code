package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.Arrays;
import java.util.List;

@TeleOp (name = "Slides", group = "Test")
public class SlidesTest extends LinearOpMode {
    DcMotor left_linear_slide, right_linear_slide;
    public static double kp = 0.04;
    double targetPosition = 500;

    @Override
    public void runOpMode() throws InterruptedException {
        left_linear_slide = hardwareMap.get(DcMotor.class, "leftLinear_slide");
        left_linear_slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_linear_slide.setDirection(DcMotorSimple.Direction.FORWARD);

        //Right Linear Slide
        right_linear_slide = hardwareMap.get(DcMotor.class, "rightLinear_slide");
        right_linear_slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_linear_slide.setDirection(DcMotorSimple.Direction.FORWARD);

        left_linear_slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_linear_slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_linear_slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_linear_slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpad_down) {
                targetPosition = 0;
                move(0);
            } else if (gamepad1.dpad_left) {
                targetPosition = -90;
                move(-90);
            } else if (gamepad1.dpad_right) {
                targetPosition = 530;
                move(530);
            } else if (gamepad1.dpad_up) {
                targetPosition = 600;
                move(600);
            }
        }
    }

    public void setLiftMotorPower(double power) {
        left_linear_slide.setPower(power);
        right_linear_slide.setPower(-power);

        
        move(targetPosition);
    }

    public List<Integer> getCurrentPosition() {
        return Arrays.asList(left_linear_slide.getCurrentPosition(), right_linear_slide.getCurrentPosition());
    }

    public void move(double targetPosition) {
        double averagePosition = (getCurrentPosition().get(0) + (getCurrentPosition().get(1) * -1)) / 2 / 19.5;
        double p = kp * (targetPosition - averagePosition);

        telemetry.addData("Position", averagePosition);
        telemetry.update();

        left_linear_slide.setPower(p);
        right_linear_slide.setPower(-p);

        if (averagePosition + 5 < targetPosition) {
            move(targetPosition);
        }
    }
}
