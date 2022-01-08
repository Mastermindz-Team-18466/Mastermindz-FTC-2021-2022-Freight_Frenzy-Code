package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
public class SlidesPID {

    DcMotor linear_slide;
    double currentPosition = 0;
    public static double kp = 0.007;

    public enum GoalPosition {
        BOTTOM,
        MIDDLE,
        TOP,
        PEAK
    }

    double goal = 0;

    public SlidesPID(DcMotor ls, double x, GoalPosition position) {
        this.linear_slide = ls;

        linear_slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linear_slide.setDirection(DcMotorSimple.Direction.FORWARD);

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
        linear_slide.setPower(control(currentPosition, goal));
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
        linear_slide.setPower(0);
    }
}
