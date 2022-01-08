package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class SlidesPID {
    Hardware hardware = new Hardware();

    double currentPosition = 0;
    public static double kp = 0.007;

    public enum GoalPosition {
        BOTTOM,
        MIDDLE,
        TOP,
        PEAK
    }

    double goal = 0;

    public SlidesPID(double x, GoalPosition position) {
        hardware.init(hardware.hardwareMap);

        this.currentPosition = x;

        switch (position) {
            case BOTTOM:
                goal = 0;
                break;

            case MIDDLE:
                goal = 240;
                break;

            case TOP:
                goal = 530;
                break;

            case PEAK:
                goal = 600;
                break;
        }
    }

    public void adjust() {
        hardware.left_linear_slide.setPower(control(currentPosition, goal));
        hardware.right_linear_slide.setPower(control(currentPosition, goal));
        currentPosition += control(currentPosition, goal);

        if (currentPosition == goal) {
            reset();
        }
    }

    public double getCurrentPosition() {
        return currentPosition;
    }

    public double control(double x, double goal) {
        double error = goal - x;
        return error * kp;
    }

    public void reset() {
        hardware.left_linear_slide.setPower(0);
        hardware.right_linear_slide.setPower(0);
    }
}
