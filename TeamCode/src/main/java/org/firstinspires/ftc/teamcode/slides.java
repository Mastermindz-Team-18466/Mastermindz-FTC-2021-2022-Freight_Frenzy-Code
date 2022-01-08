package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;
import java.util.List;

@Config
public class slides {
    public static double kp = 0.007;
    public static double bottom = 0;
    public static double mid = 240;
    public static double up = 530;
    public static double peak = 600;

    public enum liftPos {
        BOTTOM,
        MID,
        UP,
        PEAK
    }

    private liftPos targetLiftPos;
    private DcMotor lift1;
    private DcMotor lift2;
    private HardwareMap hw;
    public double p;

    public slides(HardwareMap ahw){
        targetLiftPos = liftPos.BOTTOM;
        hw = ahw;
        lift1 = hw.get(DcMotor.class, "lift1");
        lift2 = hw.get(DcMotor.class, "lift2");
        lift1.setDirection(DcMotorSimple.Direction.REVERSE);
        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setTargetLiftPos(liftPos pos) {
        targetLiftPos = pos;
    }

    public void setLiftMotorPower(double power) {
        lift1.setPower(power);
        lift2.setPower(power);
    }

    public List<Integer> getCurrentPosition(){
        return Arrays.asList(lift1.getCurrentPosition(), lift2.getCurrentPosition());
    }

    public void update() {
        double targetPos;
        switch (targetLiftPos) {
            case BOTTOM:
                targetPos = bottom;
                break;
            case MID:
                targetPos = mid;
                break;
            case UP:
                targetPos = up;
                break;
            case PEAK:
                targetPos = peak;
            default:
                targetPos = bottom;
                break;
        }
        double averagePos = (getCurrentPosition().get(0) + getCurrentPosition().get(1))/2;
        p = kp * (targetPos - averagePos);

        setLiftMotorPower(p);
    }
}




