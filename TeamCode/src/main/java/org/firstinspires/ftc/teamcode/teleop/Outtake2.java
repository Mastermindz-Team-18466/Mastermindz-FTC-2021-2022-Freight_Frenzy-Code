package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;
import java.util.List;


public class Outtake2 {
    public static double kp = 0.1;
    public static double bottom = 0;
    public static double mid = -40;
    public static double top = -60;

    public enum liftPos {
        BOTTOM,
        MID,
        TOP,
        PEAK
    }

    public enum outtakePos {
        BOTTOM_CLOSE,
        BOTTOM_OPEN,
        MID,
        TOP,
        PEAK,
        TSE
    }

    private liftPos targetLiftPos;
    private DcMotor left_linear_slide, right_linear_slide;

    HardwareMap hardwareMap;

    public outtakePos currentPos = outtakePos.BOTTOM_CLOSE;
    private long start_time = System.currentTimeMillis();
    private long prevClick = System.currentTimeMillis();

    public Outtake2(HardwareMap hardwareMap) {
        targetLiftPos = liftPos.BOTTOM;

        //Left Linear Slide
        left_linear_slide = hardwareMap.get(DcMotor.class, "leftLinear_slide");
        left_linear_slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_linear_slide.setDirection(DcMotorSimple.Direction.FORWARD);

        //Right Linear Slide
        right_linear_slide = hardwareMap.get(DcMotor.class, "rightLinear_slide");
        right_linear_slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_linear_slide.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void update() {

        double targetPos;
        switch(targetLiftPos){
            case MID:
                targetPos = mid;
                break;
            case TOP:
                targetPos = top;
                break;
            default:
                targetPos = bottom;
                break;
        }

        double averagePos = (getCurrentPosition().get(0) + (getCurrentPosition().get(1) * -1)) / 2 / 19.5;
        double p = kp * (targetPos - averagePos);

        setLiftMotorPower(p);

        if (-(Math.abs(left_linear_slide.getCurrentPosition())) == targetPos || -(Math.abs(right_linear_slide.getCurrentPosition())) == targetPos){
            stop();
        }

        if (p > 0){
            p = 0;
        }

    }

    public void setTargetLiftPos(liftPos pos) {
        targetLiftPos = pos;
    }

    public void setLiftMotorPower(double power) {
        left_linear_slide.setPower(power);
        right_linear_slide.setPower(-power);
    }

    public List<Integer> getCurrentPosition() {
        return Arrays.asList(left_linear_slide.getCurrentPosition(), right_linear_slide.getCurrentPosition());
    }

    public void stop() {
        left_linear_slide.setPower(0);
        right_linear_slide.setPower(0);
    }


}
