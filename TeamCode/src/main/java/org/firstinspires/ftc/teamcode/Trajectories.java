package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class Trajectories {
    HardwareMap hardwareMap;
    public static SampleMecanumDrive drivetrain;

    public static Pose2d POSE = new Pose2d(0, 0, 0);

    public Trajectories() {
        drivetrain = new SampleMecanumDrive(hardwareMap);
    }

    interface Action {
        void move();
    }

    public static void moveForward(double inches) {
        Trajectory moveForward = drivetrain.trajectoryBuilder(POSE)
                .forward(inches)
                .build();

        drivetrain.followTrajectory(moveForward);
        POSE = moveForward.end();
    }

    public static void strafeLeft(double inches) {
        Trajectory strafeLeft = drivetrain.trajectoryBuilder(POSE)
                .strafeLeft(inches)
                .build();

        drivetrain.followTrajectory(strafeLeft);
        POSE = strafeLeft.end();
    }

    public static void strafeRight(double inches) {
        Trajectory strafeRight = drivetrain.trajectoryBuilder(POSE)
                .strafeRight(inches)
                .build();

        drivetrain.followTrajectory(strafeRight);
        POSE = strafeRight.end();
    }

    public static void lineToPosition(Vector2d vector) {
        Trajectory lineToPosition = drivetrain.trajectoryBuilder(POSE)
                .lineTo(vector)
                .build();

        drivetrain.followTrajectory(lineToPosition);
        POSE = lineToPosition.end();
    }

    public static void strafeToPosition(Vector2d vector) {
        Trajectory strafeToPosition = drivetrain.trajectoryBuilder(POSE)
                .strafeTo(vector)
                .build();

        drivetrain.followTrajectory(strafeToPosition);
        POSE = strafeToPosition.end();
    }

    public static void splineToPosition(Vector2d vector, double heading) {
        Trajectory splineToPosition = drivetrain.trajectoryBuilder(POSE)
                .splineTo(vector, heading)
                .build();

        drivetrain.followTrajectory(splineToPosition);
        POSE = splineToPosition.end();
    }

    public static void turnDirection(double degrees) {
        TrajectorySequence turnDirection = drivetrain.trajectorySequenceBuilder(POSE)
                .turn(Math.toRadians(degrees))
                .build();

        drivetrain.followTrajectorySequence(turnDirection);
        POSE = turnDirection.end();
    }

    public static void wait(double seconds) {
        TrajectorySequence wait = drivetrain.trajectorySequenceBuilder(POSE)
                .waitSeconds(seconds)
                .build();

        drivetrain.followTrajectorySequence(wait);
        POSE = wait.end();
    }
}
