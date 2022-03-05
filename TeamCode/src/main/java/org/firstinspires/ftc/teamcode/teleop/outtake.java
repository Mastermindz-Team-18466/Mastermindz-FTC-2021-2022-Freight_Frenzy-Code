package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Arrays;
import java.util.List;

public class outtake {

    public static double kp = 0.01;
    public static double bottom = 0;
    public static double mid  = 0;
    public static double up  = 500;
    public static double shared  = 0;
    public static double tse  = 0;


    public static double openClaw = 0.7;
    public static double closedClaw = 0.5;

    public static double v4bOpen = 0.7;
    public static double v4bClose = 0;
    public static double v4bShared = 1;

    public static double minP = -0.1;

    public static int MS_between_presses = 250;

    public static double p;

    public enum liftPos {
        BOTTOM,
        MID,
        UP,
        SHARED,
        TSE
    }

    public enum outtakePos {
        IN_OPEN_START,
        IN_OPEN_END,
        IN_CLOSED,
        OUT_CLOSED_START,
        OUT_CLOSED_PIVOT,
        OUT_OPEN,
        IN_OPEN_UP
    }

    public enum clawPos {
        OPEN,
        CLOSE
    }

    public enum v4bPos{
        OPEN,
        CLOSE,
        SHARED
    }

    private enum outtakedirection {
        forward,
        backward,
        none
    }

    private static outtakedirection outtakeDirection;

    private liftPos targetLiftPos;
    private DcMotor lift1;
    private DcMotor lift2;

    private Servo v4bRight;
    private Servo v4bLeft;

    private Servo outtakeClaw;

    private HardwareMap hw;

    public outtakePos currentPos = outtakePos.IN_OPEN_START;

    private long start_time = System.currentTimeMillis();
    private long prevClick = System.currentTimeMillis();

    public outtake(HardwareMap ahw) {
        targetLiftPos = liftPos.BOTTOM;
        hw = ahw;

        outtakeClaw = hw.get(Servo.class, "outtakeClaw");

        lift1 = hw.get(DcMotor.class, "lift1");
        lift2 = hw.get(DcMotor.class, "lift2");
        lift1.setDirection(DcMotorSimple.Direction.REVERSE);
        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
            default:
                targetPos = bottom;
                break;
        }

        double averagePos = (getCurrentPosition().get(0) + getCurrentPosition().get(1))/2;
        p = kp * (targetPos - averagePos);
        if (p < minP) {
            p = minP;
        }
        setLiftMotorPower(p);

    }

    public void setTargetLiftPos(liftPos pos) {
        targetLiftPos = pos;
    }

    public void setLiftMotorPower(double power) {

        lift1.setPower(power);
        lift2.setPower(power);
    }

    public List<Integer> getCurrentPosition() {
        return Arrays.asList(lift1.getCurrentPosition(), lift2.getCurrentPosition());
    }

    public void iterateOuttakeForward() {
        if ((System.currentTimeMillis() - prevClick) < MS_between_presses) return;
        outtakeDirection = outtakedirection.forward;

        prevClick = System.currentTimeMillis();
        start_time = System.currentTimeMillis();
        switch (currentPos) {
            case IN_OPEN_START:
                currentPos = outtakePos.IN_CLOSED;
                break;
            case IN_OPEN_END:
                currentPos = outtakePos.IN_CLOSED;
                break;
            case IN_CLOSED:
                start_time = System.currentTimeMillis();
                currentPos = outtakePos.OUT_CLOSED_START;
                break;
            case OUT_CLOSED_START:
                currentPos = outtakePos.OUT_OPEN;
                break;
            case OUT_CLOSED_PIVOT:
                currentPos = outtakePos.OUT_OPEN;
                break;
            case OUT_OPEN:
                currentPos = outtakePos.IN_OPEN_UP;
                break;
            case IN_OPEN_UP:
                currentPos = outtakePos.IN_OPEN_START;
                break;
        }
        setOuttake(currentPos);
    }

    public void iterateOuttakeBackward() {
        if ((System.currentTimeMillis() - prevClick) < MS_between_presses) return;
        outtakeDirection = outtakedirection.backward;
        prevClick = System.currentTimeMillis();
        start_time = System.currentTimeMillis();
        switch (currentPos) {
            case IN_OPEN_START:
                currentPos = outtakePos.IN_OPEN_UP;
                break;
            case IN_OPEN_END:
                currentPos = outtakePos.IN_OPEN_START;
                break;
            case IN_CLOSED:
                currentPos = outtakePos.IN_OPEN_END;
                break;
            case OUT_CLOSED_START:
                currentPos = outtakePos.IN_OPEN_END;
                break;
            case OUT_CLOSED_PIVOT:
                currentPos = outtakePos.OUT_CLOSED_START;
                break;
            case OUT_OPEN:
                currentPos = outtakePos.OUT_CLOSED_PIVOT;
                break;
            case IN_OPEN_UP:
                currentPos = outtakePos.OUT_OPEN;
        }
        setOuttake(currentPos);

    }




    public void setOuttake(outtakePos pos) {
        currentPos = pos;
        start_time = System.currentTimeMillis();
        switch (pos) {
            case IN_OPEN_START:

                outtakeClaw.setPosition(closedClaw);

                setTargetLiftPos(liftPos.BOTTOM);
            case IN_OPEN_END:

                outtakeClaw.setPosition(openClaw);

                setTargetLiftPos(liftPos.BOTTOM);
                break;
            case IN_CLOSED:

                outtakeClaw.setPosition(closedClaw);

                break;
            case OUT_CLOSED_START:

                outtakeClaw.setPosition(closedClaw);

                break;
            case OUT_CLOSED_PIVOT:

                outtakeClaw.setPosition(closedClaw);

                setTargetLiftPos(liftPos.UP);
                break;
            case OUT_OPEN:

                outtakeClaw.setPosition(openClaw);

                break;
            case IN_OPEN_UP:

                outtakeClaw.setPosition(openClaw);

        }

    }


}


