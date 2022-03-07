package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Outtake {
    // Components
    private final SlidesTeleOp lift;
    private final V4B v4b;
    private final Claw claw;

    // Enums
    public enum Position {
        START,
        BOTTOM_FORWARD,
        MIDDLE_FORWARD,
        TOP_FORWARD,
        PEAK_FORWARD,
        BACK
    }

    private enum Direction {
        FORWARD,
        BACKWARD,
        NONE
    }

    private static Direction outtakeDirection;
    public Position currentPosition = Position.START;

    // Time
    public static int MS_between_presses = 250;
    private long startTime = System.currentTimeMillis();
    private long previousClick = System.currentTimeMillis();
    public long currentTime = 0;

    //Gamepad
    public Gamepad gamepad;

    // Constructor
    public Outtake(SlidesTeleOp lift, V4B v4b, Claw claw, Gamepad gamepad) {
        this.lift = lift;
        this.v4b = v4b;
        this.claw = claw;
        this.gamepad = gamepad;

    }

    public void control() {
        if (gamepad.b) {
            set(Position.BACK);
        }

        if (gamepad.dpad_up) {
            set(Position.TOP_FORWARD);
        }

        if (gamepad.dpad_down) {
            set(Position.BOTTOM_FORWARD);
        }
    }


    public void set(Position position) {
        currentPosition = position;
        startTime = System.currentTimeMillis();

        switch (position) {
            case BACK:
                //Time
                startTime = System.currentTimeMillis();
                currentTime = System.currentTimeMillis();

                while (currentTime - startTime < 250) {
                    // Refresh currentTime until wait reached
                    currentTime = System.currentTimeMillis();
                }

                //Close claw
                claw.control(Claw.State.CLOSE);

                //God mode
                v4b.control(V4B.State.CLOSE);
                lift.control(SlidesTeleOp.State.BOTTOM);

                //Time reset
                startTime = System.currentTimeMillis();
                currentTime = System.currentTimeMillis();

                while (currentTime - startTime < 500) {
                    // Refresh currentTime until wait reached
                    currentTime = System.currentTimeMillis();
                }

                //Open claw
                claw.control(Claw.State.OPEN);

            case BOTTOM_FORWARD:
                //Time
                startTime = System.currentTimeMillis();
                currentTime = System.currentTimeMillis();

                //Close claw
                claw.control(Claw.State.CLOSE);

                while (currentTime - startTime < 250) {
                    // Refresh currentTime until wait reached
                    currentTime = System.currentTimeMillis();
                }

                //God mode
                v4b.control(V4B.State.OPEN);
                lift.control(SlidesTeleOp.State.BOTTOM);

                //Time reset
                startTime = System.currentTimeMillis();
                currentTime = System.currentTimeMillis();

                while (currentTime - startTime < 500) {
                    // Refresh currentTime until wait reached
                    currentTime = System.currentTimeMillis();
                }

                //Open claw
                claw.control(Claw.State.OPEN);

            case MIDDLE_FORWARD:
                //Time
                startTime = System.currentTimeMillis();
                currentTime = System.currentTimeMillis();

                //Close claw
                claw.control(Claw.State.CLOSE);

                while (currentTime - startTime < 250) {
                    // Refresh currentTime until wait reached
                    currentTime = System.currentTimeMillis();
                }

                //God mode
                v4b.control(V4B.State.OPEN);
                lift.control(SlidesTeleOp.State.MID);

                //Time reset
                startTime = System.currentTimeMillis();
                currentTime = System.currentTimeMillis();

                while (currentTime - startTime < 500) {
                    // Refresh currentTime until wait reached
                    currentTime = System.currentTimeMillis();
                }

                //Open claw
                claw.control(Claw.State.OPEN);

            case TOP_FORWARD:
                //Time
                startTime = System.currentTimeMillis();
                currentTime = System.currentTimeMillis();

                //Close claw
                claw.control(Claw.State.CLOSE);

                while (currentTime - startTime < 250) {
                    // Refresh currentTime until wait reached
                    currentTime = System.currentTimeMillis();
                }

                //God mode
                v4b.control(V4B.State.OPEN);
                lift.control(SlidesTeleOp.State.UP);

                //Time reset
                startTime = System.currentTimeMillis();
                currentTime = System.currentTimeMillis();

                while (currentTime - startTime < 500) {
                    // Refresh currentTime until wait reached
                    currentTime = System.currentTimeMillis();
                }

                //Open claw
                claw.control(Claw.State.OPEN);

            case PEAK_FORWARD:
                //Time
                startTime = System.currentTimeMillis();
                currentTime = System.currentTimeMillis();

                //Close claw
                claw.control(Claw.State.CLOSE);

                while (currentTime - startTime < 250) {
                    // Refresh currentTime until wait reached
                    currentTime = System.currentTimeMillis();
                }

                //God mode
                v4b.control(V4B.State.OPEN);
                lift.control(SlidesTeleOp.State.PEAK);

                //Time reset
                startTime = System.currentTimeMillis();
                currentTime = System.currentTimeMillis();

                while (currentTime - startTime < 500) {
                    // Refresh currentTime until wait reached
                    currentTime = System.currentTimeMillis();
                }

                //Open claw
                claw.control(Claw.State.OPEN);
        }
    }
}


