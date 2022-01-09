package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class Trajectories {
    static Hardware hardware = new Hardware();
    public static Pose2d POSE = new Pose2d(0, 0, 0);

    public Trajectories() {
        hardware.init(hardware.hardwareMap);
    }

    interface Action {
        void move();
    }

    public static void moveForward(double inches) {
        Trajectory moveForward = hardware.drivetrain.trajectoryBuilder(POSE)
                .forward(inches)
                .build();

        hardware.drivetrain.followTrajectory(moveForward);
        POSE = moveForward.end();
    }

    public static void moveBackward(double inches) {
        Trajectory moveBackward = hardware.drivetrain.trajectoryBuilder(POSE)
                .back(inches)
                .build();

        hardware.drivetrain.followTrajectory(moveBackward);
        POSE = moveBackward.end();
    }

    public static void strafeLeft(double inches) {
        Trajectory strafeLeft = hardware.drivetrain.trajectoryBuilder(POSE)
                .strafeLeft(inches)
                .build();

        hardware.drivetrain.followTrajectory(strafeLeft);
        POSE = strafeLeft.end();
    }

    public static void strafeRight(double inches) {
        Trajectory strafeRight = hardware.drivetrain.trajectoryBuilder(POSE)
                .strafeRight(inches)
                .build();

        hardware.drivetrain.followTrajectory(strafeRight);
        POSE = strafeRight.end();
    }

    public static void lineToPosition(Vector2d vector) {
        Trajectory lineToPosition = hardware.drivetrain.trajectoryBuilder(POSE)
                .lineTo(vector)
                .build();

        hardware.drivetrain.followTrajectory(lineToPosition);
        POSE = lineToPosition.end();
    }

    public static void strafeToPosition(Vector2d vector) {
        Trajectory strafeToPosition = hardware.drivetrain.trajectoryBuilder(POSE)
                .strafeTo(vector)
                .build();

        hardware.drivetrain.followTrajectory(strafeToPosition);
        POSE = strafeToPosition.end();
    }

    public static void splineToPosition(Vector2d vector, double heading) {
        Trajectory splineToPosition = hardware.drivetrain.trajectoryBuilder(POSE)
                .splineTo(vector, heading)
                .build();

        hardware.drivetrain.followTrajectory(splineToPosition);
        POSE = splineToPosition.end();
    }

    public static void turnDirection(double degrees) {
        TrajectorySequence turnDirection = hardware.drivetrain.trajectorySequenceBuilder(POSE)
                .turn(Math.toRadians(degrees))
                .build();

        hardware.drivetrain.followTrajectorySequence(turnDirection);
        POSE = turnDirection.end();
    }

    public static void wait(double seconds) {
        TrajectorySequence wait = hardware.drivetrain.trajectorySequenceBuilder(POSE)
                .waitSeconds(seconds)
                .build();

        hardware.drivetrain.followTrajectorySequence(wait);
        POSE = wait.end();
    }
}
