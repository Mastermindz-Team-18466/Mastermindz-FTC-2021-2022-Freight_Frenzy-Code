package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

@Config
public class TeleOpFieldCentric {
    SampleMecanumDrive drive;
    Gamepad gamepad;

    BNO055IMU imu;
    Orientation angles;

    TeleOpMode teleOpMode;



    public TeleOpFieldCentric(HardwareMap hardwareMap, SampleMecanumDrive drive, Gamepad gamepad) {
        this.drive = new SampleMecanumDrive(hardwareMap);
        this.gamepad = gamepad;

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        drive.setPoseEstimate(PoseStorage.currentPose);

        //IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

    }


    public void move() {
        Pose2d poseEstimate = drive.getPoseEstimate();

        // Create a vector from the gamepad x/y inputs
        // Then, rotate that vector by the inverse of that heading
        Vector2d input = new Vector2d(
                    -gamepad.left_stick_y,
                    -gamepad.left_stick_x
        ).rotated(-poseEstimate.getHeading());

        // Pass in the rotated input + right stick value for rotation
        // Rotation is not part of the rotated input thus must be passed in separately
        drive.setWeightedDrivePower(
                new Pose2d(
                        input.getX(),
                        input.getY(),
                        -gamepad.right_stick_x
                )
        );

        if (gamepad.a) {
            drive.setPoseEstimate(new Pose2d (poseEstimate.getX(), poseEstimate.getY(), 0));
        }


//        // Create a vector from the gamepad x/y inputs
//        // Then, rotate that vector by the inverse of that heading
//        Vector2d input = new Vector2d(
//                -gamepad.left_stick_y,
//                -gamepad.left_stick_x
//        ).rotated(-angles.firstAngle);
//
//        // Pass in the rotated input + right stick value for rotation
//        // Rotation is not part of the rotated input thus must be passed in separately
//        drive.setWeightedDrivePower(
//                new Pose2d(
//                        input.getX(),
//                        input.getY(),
//                        -gamepad.right_stick_x
//                )
//        );


        drive.update();
    }
}
