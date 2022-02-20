package org.firstinspires.ftc.teamcode.sequences;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Arrays;
import java.util.List;

@TeleOp(name = "Concept: IntakeSequence", group = "Test")
public class IntakeSequence extends LinearOpMode {
    Servo rightV4B, leftV4B, claw;
    DcMotor left_linear_slide, right_linear_slide, intake_motor;
    public static double kp = 0.04;
    double targetPosition = 500;
    double intakePower = 1;
    int count = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        rightV4B = hardwareMap.get(Servo.class, "rightV4B");
        leftV4B = hardwareMap.get(Servo.class, "leftV4B");
        claw = hardwareMap.get(Servo.class, "claw");
        left_linear_slide = hardwareMap.get(DcMotor.class, "left_linear_slide");
        right_linear_slide = hardwareMap.get(DcMotor.class, "right_linear_slide");
        intake_motor = hardwareMap.get(DcMotor.class, "intake_motor");

        left_linear_slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_linear_slide.setDirection(DcMotorSimple.Direction.FORWARD);

        right_linear_slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_linear_slide.setDirection(DcMotorSimple.Direction.FORWARD);

        left_linear_slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_linear_slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_linear_slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_linear_slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.x) {
                claw.setDirection(Servo.Direction.REVERSE);
                claw.setPosition(0.5);
                intake_motor.setPower(intakePower);
                rightV4B.setDirection(Servo.Direction.FORWARD);
                leftV4B.setDirection(Servo.Direction.REVERSE);
                rightV4B.setPosition(0);
                leftV4B.setPosition(0);
                Thread.sleep(1000);
                claw.setDirection(Servo.Direction.FORWARD);
                claw.setPosition(0.7);
            }

            if (gamepad1.b) {
                claw.setDirection(Servo.Direction.REVERSE);
                claw.setPosition(0.5);
                intake_motor.setPower(0);
                rightV4B.setDirection(Servo.Direction.FORWARD);
                leftV4B.setDirection(Servo.Direction.REVERSE);
                rightV4B.setPosition(0.75);
                leftV4B.setPosition(0.75);
                claw.setDirection(Servo.Direction.FORWARD);
            }

            if (gamepad1.dpad_up) {
                targetPosition = -90;
                move(-90);
            }

            if (gamepad1.left_bumper) {
                if (count == 0) {
                    claw.setDirection(Servo.Direction.REVERSE);
                    claw.setPosition(0.5);
                    Thread.sleep(1000);
                    count = 1;
                }

                intake_motor.setPower(0);
                rightV4B.setDirection(Servo.Direction.FORWARD);
                leftV4B.setDirection(Servo.Direction.REVERSE);
                rightV4B.setPosition(0.75);
                leftV4B.setPosition(0.75);
                targetPosition = -90;
                move(-90);
            }

            if (gamepad1.dpad_down) {
                targetPosition = 0;
                move(0);
            }

            if (gamepad1.y) {
                claw.setDirection(Servo.Direction.FORWARD);
                claw.setPosition(0.7);
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
    }
}
