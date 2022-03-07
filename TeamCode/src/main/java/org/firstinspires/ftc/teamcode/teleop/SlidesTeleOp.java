package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;
import java.util.List;

public class SlidesTeleOp {
    HardwareMap hardwareMap;

    DcMotor left_linear_slide, right_linear_slide;
    Gamepad gamepad;
    public static double kp = 0.007;

    public enum State {
        BOTTOM,
        MID,
        UP,
        SHARED,
        PEAK,
        TSE
    }

    public SlidesTeleOp(Gamepad gamepad) {
        left_linear_slide = hardwareMap.get(DcMotor.class, "leftLinear_slide");
        left_linear_slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_linear_slide.setDirection(DcMotorSimple.Direction.FORWARD);

        //Right Linear Slide
        right_linear_slide = hardwareMap.get(DcMotor.class, "rightLinear_slide");
        right_linear_slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_linear_slide.setDirection(DcMotorSimple.Direction.FORWARD);

        this.gamepad = gamepad;
    }

    public void control(State state) {
        if (state == State.BOTTOM) {
            move(0);
        } else if (state == State.UP) {
            move(-90);
        }
    }

    public void setLiftMotorPower(double power) {
        left_linear_slide.setPower(power);
        right_linear_slide.setPower(power);
    }

    public List<Integer> getCurrentPosition() {
        return Arrays.asList(left_linear_slide.getCurrentPosition(), right_linear_slide.getCurrentPosition());
    }

    public void move(double targetPosition) {
        double averagePosition = (getCurrentPosition().get(0) + getCurrentPosition().get(1)) / 2;
        double p = kp * (targetPosition - averagePosition);
        setLiftMotorPower(p);
    }

    public int getPosition() {
        return (getCurrentPosition().get(0) + getCurrentPosition().get(1)) / 2;
    }
}
