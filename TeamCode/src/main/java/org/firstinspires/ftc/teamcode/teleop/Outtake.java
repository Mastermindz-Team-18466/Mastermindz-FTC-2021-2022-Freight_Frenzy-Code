package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.Gamepad;

public class Outtake {
    // Components
    private final SlidesTeleOp lift;
    private final V4B v4b;
    private final Claw claw;

    // Enums
    public enum Position {
        START,
        BOTTOM_FORWARD,
        AUTO_START,
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

    public void set(Position position) {
        currentPosition = position;
        startTime = System.currentTimeMillis();

        if (position == Position.BACK) {
            //Time
            startTime = System.currentTimeMillis();
            currentTime = System.currentTimeMillis();

            //Close claw
            claw.control(Claw.State.CLOSE);

            while (claw.claw.getPosition() != 0.5) {
                // Refresh currentTime until wait reached
                currentTime = System.currentTimeMillis();
            }

            //God mode
            v4b.control(V4B.State.CLOSE);
            while (Math.ceil(lift.getPosition()) < 0) {
                lift.control(SlidesTeleOp.State.BOTTOM);
            }

            lift.stop();

            //Time reset
            startTime = System.currentTimeMillis();
            currentTime = System.currentTimeMillis();

            while (currentTime - startTime < 1000) {
                // Refresh currentTime until wait reached
                currentTime = System.currentTimeMillis();
            }

            //Open claw
            claw.control(Claw.State.OPEN);
        } else if(position == Position.AUTO_START){
            //Time
            startTime = System.currentTimeMillis();
            currentTime = System.currentTimeMillis();

            //Close claw
            claw.control(Claw.State.CLOSE);

            while (claw.claw.getPosition() != 0.5) {
                // Refresh currentTime until wait reached
                currentTime = System.currentTimeMillis();
            }

            //God mode
            v4b.control(V4B.State.CLOSE);
            while (Math.ceil(lift.getPosition()) < 0) {
                lift.control(SlidesTeleOp.State.BOTTOM);
            }

            lift.stop();

            //Time reset
            startTime = System.currentTimeMillis();
            currentTime = System.currentTimeMillis();

            while (currentTime - startTime < 1000) {
                // Refresh currentTime until wait reached
                currentTime = System.currentTimeMillis();
            }

            //Open claw
            claw.control(Claw.State.CLOSE);

        } else if (position == Position.BOTTOM_FORWARD) {
            //Time
            startTime = System.currentTimeMillis();
            currentTime = System.currentTimeMillis();

            //Close claw
            claw.control(Claw.State.CLOSE);

            while (claw.claw.getPosition() != 0.5) {
                // Refresh currentTime until wait reached
                currentTime = System.currentTimeMillis();
            }

            while (currentTime - startTime < 250) {
                // Refresh currentTime until wait reached
                currentTime = System.currentTimeMillis();
            }

            //God mode
            v4b.control(V4B.State.OPEN);
            while (Math.ceil(lift.getPosition()) < 0) {
                lift.control(SlidesTeleOp.State.BOTTOM);
            }

            lift.stop();
        } else if(position == Position.MIDDLE_FORWARD){
            //Time
            startTime = System.currentTimeMillis();
            currentTime = System.currentTimeMillis();

            //Close claw
            claw.control(Claw.State.CLOSE);

            while (claw.claw.getPosition() != 0.5) {
                // Refresh currentTime until wait reached
                currentTime = System.currentTimeMillis();
            }

            while (currentTime - startTime < 250) {
                // Refresh currentTime until wait reached
                currentTime = System.currentTimeMillis();
            }

            //God mode
            while (Math.floor(lift.getPosition()) > -40) {
                v4b.control(V4B.State.OPEN);
                lift.control(SlidesTeleOp.State.MID);
            }

            lift.stop();

        } else if (position == Position.TOP_FORWARD) {
            //Time
            startTime = System.currentTimeMillis();
            currentTime = System.currentTimeMillis();

            //Close claw
            claw.control(Claw.State.CLOSE);

            while (claw.claw.getPosition() != 0.5) {
                // Refresh currentTime until wait reached
                currentTime = System.currentTimeMillis();
            }

            while (currentTime - startTime < 250) {
                // Refresh currentTime until wait reached
                currentTime = System.currentTimeMillis();
            }

            //God mode
            while (Math.floor(lift.getPosition()) > -60) {
                v4b.control(V4B.State.OPEN);
                lift.control(SlidesTeleOp.State.UP);
            }

            lift.stop();
        }
    }
}
