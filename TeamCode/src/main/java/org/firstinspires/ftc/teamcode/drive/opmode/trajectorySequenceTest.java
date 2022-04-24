package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class trajectorySequenceTest extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(new Pose2d(11.5, -60, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-1, -50, Math.toRadians(-68)))
                .splineTo(new Vector2d(46, -62.5), Math.toRadians(0))
                .setReversed(true)
                .splineTo(new Vector2d(-1, -50), Math.toRadians(-68 + 180))
                .setReversed(false)
                .build();

        waitForStart();

        while (opModeIsActive()) {
            drive.followTrajectorySequence(trajSeq);
        }




    }
}