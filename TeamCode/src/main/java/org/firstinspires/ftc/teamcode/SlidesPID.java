package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Arrays;
import java.util.List;

@Config
public class SlidesPID {
    Hardware hardware = new Hardware();

    public static double kp = 0.007;

    public enum GoalPosition {
        BOTTOM,
        MIDDLE,
        TOP,
        PEAK
    }

    double targetPosition = 0;

    public SlidesPID(GoalPosition position) {
        hardware.init(hardware.hardwareMap);

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

        hardware.left_linear_slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.left_linear_slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardware.right_linear_slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.right_linear_slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setLiftMotorPower(double power) {
        hardware.left_linear_slide.setPower(power);
        hardware.right_linear_slide.setPower(power);
    }

    public List<Integer> getCurrentPosition() {
        return Arrays.asList(hardware.left_linear_slide.getCurrentPosition(), hardware.right_linear_slide.getCurrentPosition());
    }

    public void control() {
        double averagePosition = (getCurrentPosition().get(0) + getCurrentPosition().get(1)) / 2;
        double p = kp * (targetPosition - averagePosition);
        setLiftMotorPower(p);
    }

    public void reset() {
        hardware.left_linear_slide.setPower(0);
        hardware.right_linear_slide.setPower(0);
    }
}
