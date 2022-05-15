package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;
import java.util.List;

@Config
public class Outtake2 {
    public static double kp = 0.1;
    public static double bottom = 0;
    public static double mid = -40;
    public static double top = -60;
    private double targetPos;
    public final Claw claw;
    public final V4B v4b;

    public enum liftPos {
        BOTTOM,
        MID,
        TOP,
        PEAK
    }

    public enum outtakePosEnum {
        CLOSE,
        OPEN,
        SHARED,
        TSE
    }

    public enum outtakeInstructionsEnum {
        CLAW_OPEN,
        CLAW_CLOSED,
        V4B_OPEN,
        V4B_CLOSED,
        CLAW_WAIT_OPEN
    }

    private liftPos targetLiftPos;
    private outtakePosEnum outtakePos;
    private outtakeInstructionsEnum outtakeInstructions;
    private DcMotor left_linear_slide, right_linear_slide;

    HardwareMap hardwareMap;

    private long startTime = System.currentTimeMillis();
    private long prevAction = System.currentTimeMillis();
    public long currentTime = 0;

    public Outtake2(HardwareMap hardwareMap, Claw claw, V4B v4b) {
        targetLiftPos = liftPos.BOTTOM;
        this.claw = claw;
        this.v4b = v4b;

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

        switch (targetLiftPos) {
            case BOTTOM:
                targetPos = bottom;
                break;
            case MID:
                targetPos = mid;
                break;
            case TOP:
                targetPos = top;
                break;
            default:
                break;
        }

        double averagePos = getPosition();
        double p = kp * (targetPos - averagePos);


        setLiftMotorPower(p);

        switch (outtakePos) {
            case OPEN:
                switch (outtakeInstructions) {
                    case CLAW_CLOSED:
                        claw.control(Claw.State.CLOSE);
                        prevAction = System.currentTimeMillis();
                        outtakeInstructions = outtakeInstructions.V4B_OPEN;
                        break;
                    case V4B_OPEN:
                        if (System.currentTimeMillis() - prevAction > 250) {
                            v4b.control(V4B.State.OPEN);
                        }
                        break;
                }
                break;
            case CLOSE:
                switch (outtakeInstructions) {
                    case CLAW_OPEN:
                        claw.control(Claw.State.OPEN);
                        prevAction = System.currentTimeMillis();
                        outtakeInstructions = outtakeInstructions.CLAW_CLOSED;
                        break;
                    case CLAW_CLOSED:
                        if (System.currentTimeMillis() - prevAction > 300) {
                            claw.control(Claw.State.CLOSE);
                            v4b.control(V4B.State.CLOSE);
                            prevAction = System.currentTimeMillis();
                            outtakeInstructions = outtakeInstructions.CLAW_WAIT_OPEN;
                        }
                        break;
                    case CLAW_WAIT_OPEN:
                        if (System.currentTimeMillis() - prevAction > 1000) {
                            claw.control(Claw.State.OPEN);
                        }
                        break;
                }
                break;
        }


    }

    public void setTargetLiftPos(liftPos pos) {
        targetLiftPos = pos;
    }

    public void setOuttakePos(outtakePosEnum pos) {
        outtakePos = pos;
    }

    public void setOuttakeInstructions(outtakeInstructionsEnum instruction) {
        outtakeInstructions = instruction;
    }

    public double getPosition() {
        return (getCurrentPosition().get(0) + (getCurrentPosition().get(1) * -1)) / 2 / 19.5;
    }

    public void setLiftMotorPower(double power) {
        left_linear_slide.setPower(power);
        right_linear_slide.setPower(-power);
    }

    public List<Integer> getCurrentPosition() {
        return Arrays.asList(left_linear_slide.getCurrentPosition(), right_linear_slide.getCurrentPosition());
    }


}
