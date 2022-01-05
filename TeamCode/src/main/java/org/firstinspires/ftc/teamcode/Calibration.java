package org.firstinspires.ftc.teamcode;

import androidx.annotation.RequiresPermission;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

@Autonomous
public class Calibration extends LinearOpMode {
    DcMotor backLeftMotor, frontLeftMotor, backRightMotor, frontRightMotor;
    DcMotor leftEncoder, rightEncoder, middleEncoder;
    BNO055IMU imu;
    ElapsedTime timer = new ElapsedTime();

    static final double calibrationSpeed = 0.5;
    static final double TICKS_PER_REV = 8192;
    static final double WHEEL_DIAMETER = 100/25.4;
    static final double GEAR_RATIO = 1;

    static final double TICKS_PER_INCH = WHEEL_DIAMETER * Math.PI * GEAR_RATIO / TICKS_PER_REV;

    File sideWheelsSeparationFile = AppUtil.getInstance().getSettingsFile("sideWheelsSeparationFile");
    File middleTickOffsetFile = AppUtil.getInstance().getSettingsFile("middleTickOffsetFile");


    @Override
    public void runOpMode() throws InterruptedException {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        //Left back motor
        backLeftMotor = hardwareMap.get(DcMotor.class, "leftBack_drive");
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        //Right Back Motor
        backRightMotor = hardwareMap.get(DcMotor.class, "rightBack_drive");
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //Left Front Motor
        frontLeftMotor = hardwareMap.get(DcMotor.class, "leftFront_drive");
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        //Right Back Motor
        frontRightMotor = hardwareMap.get(DcMotor.class, "rightFront_drive");
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //Encoders
        leftEncoder = hardwareMap.get(DcMotor.class, "leftEncoder");
        rightEncoder = hardwareMap.get(DcMotor.class, "rightEncoder");
        middleEncoder = hardwareMap.get(DcMotor.class, "middleEncoder");

        resetEncoders();

        waitForStart();

        while(imu.getAngularOrientation().firstAngle < 90 && opModeIsActive()) {
            frontRightMotor.setPower(-calibrationSpeed);
            backRightMotor.setPower(-calibrationSpeed);
            frontLeftMotor.setPower(calibrationSpeed);
            backLeftMotor.setPower(calibrationSpeed);

            if (imu.getAngularOrientation().firstAngle < 60) {
                frontRightMotor.setPower(-calibrationSpeed);
                backRightMotor.setPower(-calibrationSpeed);
                frontLeftMotor.setPower(calibrationSpeed);
                backLeftMotor.setPower(calibrationSpeed);
            } else {
                frontRightMotor.setPower(-calibrationSpeed / 2);
                backRightMotor.setPower(-calibrationSpeed / 2);
                frontLeftMotor.setPower(calibrationSpeed / 2);
                backLeftMotor.setPower(calibrationSpeed / 2);
            }
        }

        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);

        timer.reset();
        while (timer.seconds() < 1 && opModeIsActive()) {

        }

        double angle = imu.getAngularOrientation().firstAngle;
        double encoderDifference = Math.abs(Math.abs(leftEncoder.getCurrentPosition()) - Math.abs(rightEncoder.getCurrentPosition()));
        double sideEncoderTickOffset = encoderDifference / angle;
        double sideWheelsSeparation = (180 * sideEncoderTickOffset) / (TICKS_PER_INCH * Math.PI);
        double middleWheelOffset = middleEncoder.getCurrentPosition() / Math.toRadians(imu.getAngularOrientation().firstAngle);

        ReadWriteFile.writeFile(sideWheelsSeparationFile, String.valueOf(sideWheelsSeparation));
        ReadWriteFile.writeFile(middleTickOffsetFile, String.valueOf(middleWheelOffset));

        telemetry.addData("Done", "True");
        telemetry.update();

    }

    private void resetEncoders() {
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
