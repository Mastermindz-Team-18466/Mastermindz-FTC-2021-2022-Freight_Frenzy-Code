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
    Hardware hardware = new Hardware();
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
        hardware.init(hardwareMap);

        resetEncoders();

        waitForStart();

        while(hardware.imu.getAngularOrientation().firstAngle < 90 && opModeIsActive()) {
            hardware.right_front_driver.setPower(-calibrationSpeed);
            hardware.right_back_driver.setPower(-calibrationSpeed);
            hardware.left_front_driver.setPower(calibrationSpeed);
            hardware.left_back_driver.setPower(calibrationSpeed);

            if (hardware.imu.getAngularOrientation().firstAngle < 60) {
                hardware.right_front_driver.setPower(-calibrationSpeed);
                hardware.right_back_driver.setPower(-calibrationSpeed);
                hardware.left_front_driver.setPower(calibrationSpeed);
                hardware.left_back_driver.setPower(calibrationSpeed);
            } else {
                hardware.right_front_driver.setPower(-calibrationSpeed / 2);
                hardware.right_back_driver.setPower(-calibrationSpeed / 2);
                hardware.left_front_driver.setPower(calibrationSpeed / 2);
                hardware.left_back_driver.setPower(calibrationSpeed / 2);
            }
        }

        hardware.right_front_driver.setPower(0);
        hardware.right_back_driver.setPower(0);
        hardware.left_front_driver.setPower(0);
        hardware.left_back_driver.setPower(0);

        timer.reset();
        while (timer.seconds() < 1 && opModeIsActive()) {

        }

        double angle = hardware.imu.getAngularOrientation().firstAngle;
        double encoderDifference = Math.abs(Math.abs(hardware.left_encoder.getCurrentPosition()) - Math.abs(hardware.right_encoder.getCurrentPosition()));
        double sideEncoderTickOffset = encoderDifference / angle;
        double sideWheelsSeparation = (180 * sideEncoderTickOffset) / (TICKS_PER_INCH * Math.PI);
        double middleWheelOffset = hardware.middle_encoder.getCurrentPosition() / Math.toRadians(hardware.imu.getAngularOrientation().firstAngle);

        ReadWriteFile.writeFile(sideWheelsSeparationFile, String.valueOf(sideWheelsSeparation));
        ReadWriteFile.writeFile(middleTickOffsetFile, String.valueOf(middleWheelOffset));

        telemetry.addData("Done", "True");
        telemetry.update();

    }

    private void resetEncoders() {
        hardware.left_front_driver.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.left_back_driver.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.right_front_driver.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.right_back_driver.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        hardware.left_front_driver.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.left_back_driver.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.right_front_driver.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.right_back_driver.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
