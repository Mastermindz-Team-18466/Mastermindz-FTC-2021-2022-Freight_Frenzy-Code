package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;
import java.util.List;

@Config
public class SlidesPID {
    HardwareMap hardwareMap;

    DcMotor left_linear_slide, right_linear_slide;

    public static double kp = 0.007;

    public enum GoalPosition {
        BOTTOM,
        MIDDLE,
        TOP,
        PEAK
    }

    double targetPosition = 0;

    public SlidesPID(GoalPosition position) {
        left_linear_slide = hardwareMap.get(DcMotor.class, "leftLinear_slide");
        left_linear_slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_linear_slide.setDirection(DcMotorSimple.Direction.FORWARD);

        //Right Linear Slide
        right_linear_slide = hardwareMap.get(DcMotor.class, "rightLinear_slide");
        right_linear_slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_linear_slide.setDirection(DcMotorSimple.Direction.FORWARD);

        switch (position) {
            case BOTTOM:
                targetPosition = 0;
                break;

            case MIDDLE:
                targetPosition = 240;
                break;

            case TOP:
                targetPosition = 530;
                break;

            case PEAK:
                targetPosition = 600;
                break;
        }

        left_linear_slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_linear_slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_linear_slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_linear_slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setLiftMotorPower(double power) {
        left_linear_slide.setPower(power);
        right_linear_slide.setPower(power);
    }

    public List<Integer> getCurrentPosition() {
        return Arrays.asList(left_linear_slide.getCurrentPosition(), right_linear_slide.getCurrentPosition());
    }

    public void control() {
        double averagePosition = (getCurrentPosition().get(0) + getCurrentPosition().get(1)) / 2;
        double p = kp * (targetPosition - averagePosition);
        setLiftMotorPower(p);
    }

    public void reset() {
        left_linear_slide.setPower(0);
        right_linear_slide.setPower(0);
    }
}
